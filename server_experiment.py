import socket

ip_address = "127.0.0.1"
port = 6767

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((ip_address, port))
    s.listen()

    while True:
        connection, address = s.accept()
        data = connection.recv(4096)

        if data:
            commands = data.decode()
            commands = commands.split("\n")
            commands.pop()
            commands = [s.split() for s in commands]
            for chain in commands:
                print(chain)
