import requests
import getmac
import pathlib


def register(ip, key):
    ip_filename = pathlib.PurePath(pathlib.Path.home(), 'pynqp2p-ip')
    key_filename = pathlib.PurePath(pathlib.Path.home(), 'pynqp2p-key')

    with open(ip_filename, 'w+') as ip_file:
        ip_file.write(ip)
    
    with open(key_filename, 'w+') as key_file:
        key_file.write(key)


def get_ip():
    ip_filename = pathlib.PurePath(pathlib.Path.home(), 'pynqp2p-ip')

    with open(ip_filename, 'r') as ip_file:
        return ip_file.read()


def get_key():
    key_filename = pathlib.PurePath(pathlib.Path.home(), 'pynqp2p-key')
    
    with open(key_filename, 'r') as key_file:
        return key_file.read()


def get_id():
    return getmac.get_mac_address(interface='eth0')


def ping():
    r = requests.post(f'http://{get_ip()}/ping', data = {'key': get_key(), 'id': get_id()})
    return r.text

def send(send_id, message):
    r = requests.post(f'http://{get_ip()}/send', data = {'key': get_key(), 'id': send_id,
        'message': message})
    return r.text

def receive():
    r = requests.get(f'http://{get_ip()}/receive', data = {'key': get_key(), 'id': get_id()})
    return r.text

def receive_all():
    r = requests.get(f'http://{get_ip()}/receive_all', data = {'key': get_key(), 'id': get_id()})
    return r.text.split('\n')[:-1]
