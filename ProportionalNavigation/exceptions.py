class InvalidProportionalGainError(Exception):
    def __init__(self, N, message="Proportional Gain is not above 0"):
        self.N = N
        self.message = message
        super().__init__(self.message)
