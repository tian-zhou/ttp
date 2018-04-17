#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      tzhou
#
# Created:     11/10/2017
# Copyright:   (c) tzhou 2017
# Licence:     <your licence>
#-------------------------------------------------------------------------------

# server.py
import socket
import time

def main():
    # create a socket object
    serversocket = socket.socket(
    	        socket.AF_INET, socket.SOCK_STREAM)

    # get local machine name
    host = socket.gethostname()

    port = 59500

    # bind to the port
    serversocket.bind((host, port))

    # queue up to 5 requests
    serversocket.listen(5)

    while True:
        print "waiting for client to connect ..."

        # establish a connection
        clientsocket,addr = serversocket.accept()

        print("Got a connection from %s" % str(addr))
        msg_count = 0
        while True:
            currentTime = time.ctime(time.time()) + "\r\n"
            numBytes = clientsocket.send(currentTime.encode('ascii'))
            msg_count += 1
            print "message %i sent with %i bytes" % (msg_count, numBytes)
            if msg_count == 20:
                break
        clientsocket.close()
        print "socket with client is closed..."
    serversocket.close()
    print "server socket is closed..."

if __name__ == '__main__':
    main()
