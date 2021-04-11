import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('192.168.1.50', 1234))

msg = s.recv(1024)

print([x for x in msg])