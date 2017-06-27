#!/usr/bin/env python

import qi
import argparse
import sys
import time
import threading
import os


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--pose", type=float, nargs="+", default=[0, 0, 0],
                        help="Pose to set")
    args = parser.parse_args()
    pip = args.pip
    pport = args.pport

    #Starting application
    try:
        connection_url = "tcp://" + pip + ":" + str(pport)
        print "Connecting to ",	connection_url
        app = qi.Application(["Memory Write", "--qi-url=" + connection_url ])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    app.start()
    session = app.session

    #Starting services
    memory_service  = session.service("ALMemory")

    key = "NAOqiLocalizer/RobotPose"
    memory_service.insertData(key,args.pose)

    print "Write ",key," = ",args.pose

    


if __name__ == "__main__":
    main()

