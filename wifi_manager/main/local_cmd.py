import socket


def main():
    # ---------- 1. 输入目标地址 ----------
    ip = input("请输入 ESP32 的 IP（例如 192.168.1.188）: ").strip()
    port_str = input("请输入端口（例如 8080）: ").strip()
    try:
        port = int(port_str)
    except ValueError:
        print("端口必须是数字，退出。")
        return

    # ---------- 2. 创建 UDP socket ----------
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)          # 等回复最多 2 秒，避免卡死
    addr = (ip, port)

    print(f"\n已就绪：目标 {ip}:{port}")
    print("输入要发送的内容，回车发送")
    print("  F = 前进   B = 后退   S = 停止")
    print("  也可以发任意字符串，例如 hello")
    print("  输入 exit 退出\n")

    # ---------- 3. 主循环 ----------
    while True:
        try:
            msg = input(">>> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已退出。")
            break

        if not msg:
            continue

        if msg.lower() in ("exit", "quit", "q"):
            print("已退出。")
            break

        # 发送
        try:
            sock.sendto(msg.encode("utf-8"), addr)
        except OSError as e:
            print(f"发送失败：{e}")
            continue

        # 等回复
        try:
            data, _ = sock.recvfrom(1024)
            print(f"[ESP32 回复] {data.decode('utf-8', errors='replace')}")
        except socket.timeout:
            print("[超时] 2 秒内没有收到回复（ESP32 可能没收到，或 ACK 丢了）")
        except OSError as e:
            print(f"接收失败：{e}")

    sock.close()


if __name__ == "__main__":
    main()