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

# client.py
import socket
import time

def main():

    # create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # get local machine name
    # host = socket.gethostname()
    host = '128.46.125.47'

    port = 59500

    # connection to hostname on the port.
    re_try = 1
    while re_try:
        try:
            s.connect((host, port))
            re_try = 0
        except:
            print("Connection failed. Retry after 0.1 seconds")
            re_try = 1
            time.sleep(0.1)
            continue

    while True:
        # Receives 512 chars
        data = s.recv(512)

        print("received from server: %s" % data)

        # send acknowledgment back
        bytesSent = s.send('Got it!\0')
        print 'send ack back with len %i' % bytesSent

    s.close()
    print "client socket closed"

if __name__ == '__main__':
    main()
