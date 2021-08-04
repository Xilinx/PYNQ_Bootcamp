from multiprocessing import Process
from pathlib import Path
import os
import shutil
import requests

TEMP_FILE = '/tmp/pynqp2p_running'
LOG_FOLDER = '/tmp/pynqp2p_logs'

def launch():
    # check for temporary file
    # if temp file exists, return error

    temp_file = Path(TEMP_FILE)
    if temp_file.is_file():
       raise RuntimeError('pynqp2p already running')

    # remove log folder
    try:
        shutil.rmtree(LOG_FOLDER)
    except FileNotFoundError:
        pass


    # launch background listener
    current_folder = Path(__file__).parent.resolve()

    server_script = Path(current_folder, "server.py")

    os.system(f'nohup python3 {server_script} &')

    temp_file.touch()

def reset():
    # kill background listener
    os.system(f'pkill -9 -f server.py')

    # delete temporary files
    try:
        os.remove(TEMP_FILE)
    except:
        pass

    try:
        shutil.rmtree(LOG_FOLDER)
    except FileNotFoundError:
        pass


def send(target_ip, message):
    # post message to target 
    r = requests.post(f'http://{target_ip}:9001', json = {'data': message})


def receive():
    # get sorted list of message files
    logs = sorted(os.listdir(LOG_FOLDER))
    # if no files, error
    if len(logs) == 0:
        raise RuntimeError('No messages are waiting to be recieved')
    
    # get oldest log file
    current_log_file = Path(LOG_FOLDER, logs[0])
    current_log = open(current_log_file)
    
    # get file contents
    message = current_log.read()
    
    # delet file
    os.remove(current_log_file)

    return message

def receive_all():
    # get sorted list of all mesagge log files 
    logs = sorted(os.listdir(LOG_FOLDER))
    if len(logs) == 0:
        raise RuntimeError('No messages are waiting to be recieved')

    messages = []
    
    # read all files and store in array
    for log in logs:
        current_log_file = Path(LOG_FOLDER, log)
        current_log = open(current_log_file)
        messages.append(current_log.read())
        os.remove(current_log_file)

    return messages

def clear():
    # delet all logfiles in folder
    for log in os.listdir(LOG_FOLDER):
        try:
            os.remove(log)
        except:
            pass
