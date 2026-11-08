#ifndef UDP_LISTENER_H
#define UDP_LISTENER_H

/* 启动 UDP 监听任务，绑定到指定核心 */
void udp_listener_start(int core_id, int port);

#endif