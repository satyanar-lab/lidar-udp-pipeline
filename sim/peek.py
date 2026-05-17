import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("127.0.0.1", 9000))
print("listening on 9000 ...")
data, addr = s.recvfrom(4096)
print(f"got {len(data)} bytes from {addr}: {data[:10].hex()}")
