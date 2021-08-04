import json
import os
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path

LOG_PATH = '/tmp/pynqp2p_logs'
os.mkdir(LOG_PATH)
message_counter = 0

class MyHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        cont_length = int(self.headers['Content-Length']) # <--- Gets the size of data
        body = self.rfile.read(cont_length).decode('UTF-8') # <--- Gets the data itself
        body_dict = json.loads(body)
   
        global message_counter
 
        log_file = Path(LOG_PATH, f'{message_counter}.log')
        message_counter += 1

        log = open(log_file, 'w')

        log.write(body_dict["data"])
        log.flush()
        log.close()

        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()


def run_webserver(server_class=HTTPServer, handler_class=MyHandler):
    server_address = ('', 9001)
    httpd = server_class(server_address, handler_class)
    httpd.serve_forever()


run_webserver()
