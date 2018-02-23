

class PDController():

    def __init__(self, K,B):
        self.K = K
        self.B = B

    def get_F(self,e,ed):
        F = self.K.dot(e) + self.B.dot(ed)
        return F

    def set_K(self,K):
        self.K = K


    def set_B(self,B):
        self.B = B
