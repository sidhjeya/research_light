# my_class.py

class DataProcessor:
    def __init__(self):
        self.data = None

    def process_data(self, data):
        self.data = data

    def get_processed_data(self):
        return self.data
