#Reference: https://pymotw.com/2/socket/tcp.html

import socket
import sys
import signal
import json



packet_size = 500
port_num = 10000
hostname = 'localhost'

json_structure = {
    'Controllers': [0,0,0,0,0,0]
}


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
server_address = (hostname, port_num)

print >>sys.stderr, '#########\nstarting up on %s port %s\n#########' % server_address
sock.bind(server_address)
# Listen for incoming connections
sock.listen(1)
print >>sys.stderr, '#########\nwaiting for a connection\n#########'
connection, client_address = sock.accept()  #wait until it get a client

def write(conntection, data):
    print >>sys.stderr, 'sending data to the client', data
    connection.sendall(data)

def read(connection):
    data = []
    print >>sys.stderr, 'connection from', client_address
    # Receive the data in small chunks and retransmit it
    # while True:
    data.append(connection.recv(packet_size))         #reading part
    print >>sys.stderr, 'received "%s"' % data[-1]
        # if not(data[-1]):
        #     print >>sys.stderr, 'no more data from', client_address
        #     break
    return "".join(data)


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    connection.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)



while True:
    # Wait for a connection
    try:
        print("read-----------------------------")
        # read(connection)
        try:
            jsonObj = json.loads(read(connection))
            # print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
            # print(jsonObj['Controllers'][0])
            # print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
        except:
            print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ERROR in PARSING$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        # print(jsonObj['Controller'])
        # print("message: {:s}".format(read(connection)))
        print("write-----------------------------")
        write(connection,json.dumps(json_structure))
    except:
        print("BIG ERROR")
        break
connection.close()
