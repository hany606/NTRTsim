#Reference: https://pymotw.com/2/socket/tcp.html

#Coding Style: camelCase

#import the libraries
import socket
import sys
import signal
import json
from time import sleep
import os

#import openai
#import tensorflow as tf
#import numpy as np
#from baselines import ...

#--------------------------------------------Vars--------------------------------------------

#Settings for the TCP communication
packetSize = 500
portNum = 10010
hostName = 'localhost'

globalFlag = False

# JSON object structure
jsonObj = {
    'Controllers': [0,0,0,0,0,0],
    'Reset': 0
}
#--------------------------------------------------------------------------------------------


#--------------------------------------------Functions--------------------------------------------
# Ctrl+C Handle to close safely the TCP connection
def signalHandler(signal, frame):
    print('You pressed Ctrl+C!')
    globalFlag = True
    connection.close()
    sys.exit(0)

    # tmp = str(raw_input("You want reset or close: r/c: \n"))
    # print(tmp)
    # sleep(4)
    # if(tmp == 'r'):
    #     # TODO: Break the connection, then reset the connection
    #     print("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
    #     # print('#########\nwaiting for a connection\n#########')
    #     # connection, clientAddress = sock.accept()  #wait until it get a client
    #     # print('connection from', clientAddress)
    #     # main()
    # elif(tmp == 'c'):
    #     print("----------------------------------Exit-----------------------------------")
    #     sys.exit(0)
    # else:
    #     print("Please write 'r' or 'c'")
# function for writing data into TCP connection
def write(conntection, data):
    print('sending data to the client:"{}"'.format(data))
    connection.sendall(data)

# function for reading data from TCP connection
def read(connection):
    data = []
    # Receive the data in small chunks and retransmit it
    # while True:
    data.append(connection.recv(packetSize))         #reading part
    print('received "{}"'.format(data[-1]))
        # if not(data[-1]):
        #     print >>sys.stderr, 'no more data from', client_address
        #     break
    return "".join(data)


def main():
    # global globalFlag
    # globalFlag = False
    while True:
        # Wait for a connection
        try:
            r = read(connection)    # Recieve incoming data
        except:
            print("$$$$$$$$$$$$ ERROR in Reading $$$$$$$$$$$$")
            # break
        jsonObjTmp = json.loads(r)  # Parse the data from string to json
        # TODO: Use the incoming data after being converted to json

        # TODO:
        # Take the data from the simulator module
        # Formulate the data as observation
        # Generate Reward
        # Feed the RL Algorithm with Reward and observartion
        # Generate Action
        # Decide either end of episode (Reset the simulator) or specific Action
        # Modify the action in json
        try:
            write(connection,json.dumps(jsonObj))   # Write to the simulator module the json object with the required info
        except:
            print("$$$$$$$$$$$$ ERROR in Writing $$$$$$$$$$$$")
            # break
        # if(globalFlag):
        #     print("GLOBAL")
        #     break
#-------------------------------------------------------------------------------------------------


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # Create a TCP/IP socket
serverAddress = (hostName, portNum) # Bind the socket to the port


print('#########\nstarting up on {} port {}\n#########'.format(serverAddress, portNum))
sock.bind(serverAddress)
sock.listen(1)  # Listen for incoming connections



signal.signal(signal.SIGINT, signalHandler) # Activate the listen to the Ctrl+C

# This is top open the simulator
print("Opening the NTRT simulator")
#Note: TODO: Make in the simulator wait a second then send a message
os.system('/home/hany/repos/Tensegrity-Robot-IU-Internship19/src/dev/test/Communication_protocol_test/helper.sh')

print('#########\nwaiting for a connection\n#########')
connection, clientAddress = sock.accept()  #wait until it get a client
print('connection from', clientAddress)
main()
