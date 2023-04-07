import threading


class CustomSignal(object):
    def __init__(self):
        self.method = None

    def connect(self, method):
        self.method = method

    def emit(self, *args):
        threading.Thread(target=self.method, args=args, daemon=True).start()
