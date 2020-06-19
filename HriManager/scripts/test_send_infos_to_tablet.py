#!/usr/bin/env python
from socketIO_client import SocketIO, LoggingNamespace
import time

class Test():

    def __init__(self):


        self.socketIO = SocketIO('http://127.0.0.1', 5000, LoggingNamespace)

        json_data = {
            "people_list": [
                {
                    "name": "Thomas",
                    "score": 10.3
                },
                {
                    "name": "Simon",
                    "score": 56.5
                }
            ]
        }

        self.socketIO.emit("sendPeopleListDebug",json_data,broadcast=True)

        time.sleep(5)

        json_data = {
            "people_list": []
        }
        self.socketIO.emit("sendPeopleListDebug",json_data,broadcast=True)


if __name__ == "__main__":
    a=Test()