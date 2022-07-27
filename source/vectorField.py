class vectorField():
    vectorField = []
    def __init__(self):
        self.vectorField = []
    
    def get(self):
        return self.vectorField

    def add(self, vector):
        self.vectorField.append(vector)

    def print(self):
        print()