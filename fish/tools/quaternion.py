class Quaternion:
    a = 0
    b = 0
    c = 0
    d = 0

    def get_norm(self):
        return sqrt((a*a) + (b*b) + (c*c) + (d*d))

    def get_versor(self):
        norm = get_norm()
        return Quaternion(self.a / norm, self.b / norm, self.c / norm, self.d / norm)

    def get_conjugate(self):
        return Quaternion(self.a, -self.b, -self.c, -self.d)

    def product(self, Quaternion q):
        new_a = (a*q.a) - (b*q.b) - (c*q.c) - (d*q.d)
        new_b = (a*q.b) + (b*q.a) + (c*q.d) - (d*q.c)
        new_c = (a*q.c) - (b*q.d) + (c*q.a) + (d*q.b) 
        new_d = (a*q.d) + (b*q.c) - (c*q.b) + (d*q.a)
        return Quaternion(new_a, new_b, new_c, new_d)

    def get_scalar(self):
        return a
        
    def get_vector(self):
        return [b, c, d]

    def add(self, val):
        return Quaternion(a + val, b + val, c + val, d + val)
