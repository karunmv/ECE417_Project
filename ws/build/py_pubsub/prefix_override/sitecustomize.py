import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/karunmv/ece417/ECE417_Project/ws/install/py_pubsub'
