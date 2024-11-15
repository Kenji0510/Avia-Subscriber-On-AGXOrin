#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>

#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include <ev.h>

#include <quiche.h>

#include "send_data.h"

#define LOCAL_CONN_ID_LEN 16

#define MAX_DATAGRAM_SIZE 1400

struct conn_io {
    ev_timer timer;

    int sock;

    struct sockaddr_storage local_addr;
    socklen_t local_addr_len;

    quiche_conn *conn;

    size_t header_size;
    size_t is_sent_header;
    uint8_t *header_data;
    uint8_t *file_data;
    size_t file_size;
    size_t file_offset;
};

/*
static void debug_log(const char *line, void *argp) {
    fprintf(stderr, "%s\n", line);
}
*/

static void flush_egress(struct ev_loop *loop, struct conn_io *conn_io) {
    static uint8_t out[MAX_DATAGRAM_SIZE];

    quiche_send_info send_info;

    while (1) {
        ssize_t written = quiche_conn_send(conn_io->conn, out, sizeof(out),
                                           &send_info);

        if (written == QUICHE_ERR_DONE) {
            //fprintf(stderr, "done writing\n");
            break;
        }

        if (written < 0) {
            fprintf(stderr, "failed to create packet: %zd\n", written);
            return;
        }

        ssize_t sent = sendto(conn_io->sock, out, written, 0,
                              (struct sockaddr *) &send_info.to,
                              send_info.to_len);

        if (sent != written) {
            perror("failed to send");
            return;
        }

        //fprintf(stderr, "sent %zd bytes\n", sent);
    }

    double t = quiche_conn_timeout_as_nanos(conn_io->conn) / 1e9f;
    //fprintf(stderr, "timeout in %f\n", t);

    // When timeout time was set 0, To prevent stack for send process, 
    const double epsilon = 1e-7;
    if (fabs(t) < epsilon) {
        fprintf(stderr, "timeout in %f\n", t);
        ev_break(EV_A_ EVBREAK_ONE);
    }
    conn_io->timer.repeat = t;
    ev_timer_again(loop, &conn_io->timer);
}

static void recv_cb(EV_P_ ev_io *w, int revents) {
    //fprintf(stderr, "recv_cv()\n");

    //static bool req_sent = false;

    struct conn_io *conn_io = w->data;

    static uint8_t buf[65535];

    while (1) {
        struct sockaddr_storage peer_addr;
        socklen_t peer_addr_len = sizeof(peer_addr);
        memset(&peer_addr, 0, peer_addr_len);

        ssize_t read = recvfrom(conn_io->sock, buf, sizeof(buf), 0,
                                (struct sockaddr *) &peer_addr,
                                &peer_addr_len);

        if (read < 0) {
            if ((errno == EWOULDBLOCK) || (errno == EAGAIN)) {
                //fprintf(stderr, "recv would block\n");
                break;
            }

            perror("failed to read");
            return;
        }

        quiche_recv_info recv_info = {
            (struct sockaddr *) &peer_addr,
            peer_addr_len,

            (struct sockaddr *) &conn_io->local_addr,
            conn_io->local_addr_len,
        };

        ssize_t done = quiche_conn_recv(conn_io->conn, buf, read, &recv_info);

        if (done < 0) {
            fprintf(stderr, "failed to process packet\n");
            continue;
        }

        //fprintf(stderr, "recv %zd bytes\n", done);
    }

    //fprintf(stderr, "done reading\n");

    if (quiche_conn_is_closed(conn_io->conn)) {
        fprintf(stderr, "connection closed\n");

        ev_break(EV_A_ EVBREAK_ONE);
        return;
    }

    if (quiche_conn_is_established(conn_io->conn)) {
        while (conn_io->file_offset < conn_io->file_size) {
            const uint8_t *app_proto;
            size_t app_proto_len;

            quiche_conn_application_proto(conn_io->conn, &app_proto, &app_proto_len);

            // Sent header first.
            if (conn_io->is_sent_header == 0) {
                uint64_t error_code;
                ssize_t sent = quiche_conn_stream_send(conn_io->conn, 4, conn_io->header_data, conn_io->header_size, false, &error_code);

                if (sent < 0) {
                    if (sent == QUICHE_ERR_BUFFER_TOO_SHORT || sent == QUICHE_ERR_DONE) {
                        // The buffer is full, stop sending for now and try again later.
                        break;
                    }
                    fprintf(stderr, "failed to send header: %" PRIu64 "\n", error_code);
                    return;
                }

                conn_io->is_sent_header = 1;
                fprintf(stderr, "sent header\n");
            }

            // Send file data.
            size_t remaining = conn_io->file_size - conn_io->file_offset;
            size_t to_send = remaining < MAX_DATAGRAM_SIZE ? remaining : MAX_DATAGRAM_SIZE;
            uint64_t error_code;
            ssize_t sent = quiche_conn_stream_send(conn_io->conn, 4, conn_io->file_data + conn_io->file_offset, to_send, false, &error_code);

            if (sent < 0) {
                if (sent == QUICHE_ERR_BUFFER_TOO_SHORT || sent == QUICHE_ERR_DONE) {
                    // The buffer is full, stop sending for now and try again later.
                    break;
                }
                fprintf(stderr, "failed to send data: %" PRIu64 "\n", error_code);
                return;
            }

            //size_t payload_size = quiche_conn_max_send_udp_payload_size(conn_io->conn);
            //fprintf(stdout, "payload size: %zu\n", payload_size);

            conn_io->file_offset += sent;
            //fprintf(stderr, "sent %zd bytes, offset: %zu\n", sent, conn_io->file_offset);

            if (conn_io->file_offset == conn_io->file_size) {
                uint8_t r = 0;
                if (quiche_conn_stream_send(conn_io->conn, 4, &r, sizeof(r), true, &error_code) < 0) {
                    fprintf(stderr, "failed to send FIN: %" PRIu64 "\n", error_code);
                    return;
                }
                fprintf(stderr, "sent FIN\n");
                break;
            }
        }
    }

    
    if (quiche_conn_is_established(conn_io->conn)) {
        uint64_t s = 0;

        quiche_stream_iter *readable = quiche_conn_readable(conn_io->conn);

        while (quiche_stream_iter_next(readable, &s)) {
            fprintf(stderr, "stream %" PRIu64 " is readable\n", s);

            bool fin = false;
            uint64_t error_code;
            ssize_t recv_len = quiche_conn_stream_recv(conn_io->conn, s,
                                                       buf, sizeof(buf),
                                                       &fin, &error_code);
            if (recv_len < 0) {
                break;
            }

            printf("%.*s", (int) recv_len, buf);

            if (fin) {
                if (quiche_conn_close(conn_io->conn, true, 0, NULL, 0) < 0) {
                    fprintf(stderr, "failed to close connection\n");
                }
                fprintf(stderr, "finished sending\n");
                //ev_break(EV_A_ EVBREAK_ONE);
            }
        }

        quiche_stream_iter_free(readable);
    }
    

    flush_egress(loop, conn_io);
}

static void timeout_cb(EV_P_ ev_timer *w, int revents) {
    struct conn_io *conn_io = w->data;
    quiche_conn_on_timeout(conn_io->conn);

    fprintf(stderr, "timeout\n");

    flush_egress(loop, conn_io);

    if (quiche_conn_is_closed(conn_io->conn)) {
        quiche_stats stats;
        quiche_path_stats path_stats;

        quiche_conn_stats(conn_io->conn, &stats);
        quiche_conn_path_stats(conn_io->conn, 0, &path_stats);

        fprintf(stderr, "connection closed, recv=%zu sent=%zu lost=%zu rtt=%" PRIu64 "ns\n",
                stats.recv, stats.sent, stats.lost, path_stats.rtt);

        ev_break(EV_A_ EVBREAK_ONE);
        return;
    }
}

int send_data(const char* ip_addr, const char* port_num, data_packet* packet_ptr) {
    fprintf(stderr, "Sending data to %s:%s\n", ip_addr, port_num);

    const char *host = ip_addr;
    const char *port = port_num;

    struct timespec start, end;

    const struct addrinfo hints = {
        .ai_family = PF_UNSPEC,
        .ai_socktype = SOCK_DGRAM,
        .ai_protocol = IPPROTO_UDP
    };

    //quiche_enable_debug_logging(debug_log, NULL);

    struct addrinfo *peer;
    if (getaddrinfo(host, port, &hints, &peer) != 0) {
        perror("failed to resolve host");
        return -1;
    }

    int sock = socket(peer->ai_family, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("failed to create socket");
        return -1;
    }

    int sndbuf = 1024 * 1024; // 1MB
    if (setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
        perror("failed to set socket send buffer size");
        return -1;
    }

    if (fcntl(sock, F_SETFL, O_NONBLOCK) != 0) {
        perror("failed to make socket non-blocking");
        return -1;
    }

    quiche_config *config = quiche_config_new(0xbabababa);
    if (config == NULL) {
        fprintf(stderr, "failed to create config\n");
        return -1;
    }

    quiche_config_set_application_protos(config,
        (uint8_t *) "\x0ahq-interop\x05hq-29\x05hq-28\x05hq-27\x08http/0.9", 38);

    quiche_config_set_max_idle_timeout(config, 3000);
    quiche_config_set_max_recv_udp_payload_size(config, MAX_DATAGRAM_SIZE);
    quiche_config_set_max_send_udp_payload_size(config, MAX_DATAGRAM_SIZE);
    quiche_config_set_initial_max_data(config, 10*1024*1024);
    quiche_config_set_initial_max_stream_data_bidi_local(config, 1024*1024);
    quiche_config_set_initial_max_stream_data_uni(config, 1024*1024);
    quiche_config_set_initial_max_stream_data_bidi_remote(config, 1024*1024);
    /*
    quiche_config_set_initial_max_streams_bidi(config, 100);
    quiche_config_set_initial_max_streams_uni(config, 100);
    */
    quiche_config_set_initial_max_streams_bidi(config, 10);
    quiche_config_set_initial_max_streams_uni(config, 10);
    quiche_config_set_disable_active_migration(config, true);
    quiche_config_set_cc_algorithm(config, QUICHE_CC_BBR2);
    quiche_config_enable_hystart(config, true);

    if (getenv("SSLKEYLOGFILE")) {
      quiche_config_log_keys(config);
    }

    uint8_t scid[LOCAL_CONN_ID_LEN];
    int rng = open("/dev/urandom", O_RDONLY);
    if (rng < 0) {
        perror("failed to open /dev/urandom");
        return -1;
    }

    ssize_t rand_len = read(rng, &scid, sizeof(scid));
    if (rand_len < 0) {
        perror("failed to create connection ID");
        return -1;
    }

    struct conn_io *conn_io = malloc(sizeof(*conn_io));
    if (conn_io == NULL) {
        fprintf(stderr, "failed to allocate connection IO\n");
        return -1;
    }

    conn_io->local_addr_len = sizeof(conn_io->local_addr);
    if (getsockname(sock, (struct sockaddr *)&conn_io->local_addr,
                    &conn_io->local_addr_len) != 0)
    {
        perror("failed to get local address of socket");
        return -1;
    };

    quiche_conn *conn = quiche_connect(host, (const uint8_t *) scid, sizeof(scid),
                                       (struct sockaddr *) &conn_io->local_addr,
                                       conn_io->local_addr_len,
                                       peer->ai_addr, peer->ai_addrlen, config);

    if (conn == NULL) {
        fprintf(stderr, "failed to create connection\n");
        return -1;
    }

    conn_io->sock = sock;
    conn_io->conn = conn;

    conn_io->header_size = sizeof(data_packet);
    conn_io->header_data = (uint8_t *)malloc(conn_io->header_size);
    if (conn_io->header_data == NULL) {
        perror("failed to allocate file data");
        return -1;
    }

    memcpy(conn_io->header_data, packet_ptr, conn_io->header_size);

    conn_io->file_size = (packet_ptr->num_points_of_cloud_registered + packet_ptr->num_points_of_laser_map) * sizeof(double) * 3;
    conn_io->file_data = (uint8_t *)packet_ptr->float_array_ptr;
    conn_io->is_sent_header = 0;
    conn_io->file_offset = 0;

    ev_io watcher;

    struct ev_loop *loop = ev_default_loop(0);

    ev_io_init(&watcher, recv_cb, conn_io->sock, EV_READ);
    ev_io_start(loop, &watcher);
    watcher.data = conn_io;

    ev_init(&conn_io->timer, timeout_cb);
    conn_io->timer.data = conn_io;
    //ev_timer_again(loop, &conn_io->timer);
    
    clock_gettime(CLOCK_MONOTONIC, &start);

    flush_egress(loop, conn_io);

    //ev_loop(loop, 0);
    ev_run(loop, 0);

    clock_gettime(CLOCK_MONOTONIC, &end);
    
    double elapsed_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    printf("Data transfer took %.3f seconds\n", elapsed_time);
    
    ev_io_stop(loop, &watcher);
    ev_break(loop, EVBREAK_ALL);
    ev_loop_destroy(loop);

    // printf("Num points: %ld\n", packet_ptr->num_points);
    // printf("First float: %f\n", packet_ptr->float_array_ptr[0]);


    //free(conn_io->file_data);
    //free(conn_io);

    freeaddrinfo(peer);

    quiche_conn_free(conn);

    quiche_config_free(config);

    return 0;
}
