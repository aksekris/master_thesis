# Written by Aksel Kristoffersen

class SecondOrderLPFilter(object):
    def __init__(self, delta, omega):
        self.delta = delta
        self.omega = omega


class ReferenceModel:
    def __init__(self):
