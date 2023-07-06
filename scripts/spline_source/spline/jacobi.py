import time

'''p = algarismos significativos'''
class Jacobi:

    # x(k+1) = bx(k) + g
    def __init__(self, m, y, p = 4, max_attemps = 100):
        self.b = m
        self.y = y
        self.g = y
        self.p = p
        self.max_attemps = max_attemps
        
        self.li_len  = len(self.b)
        self.col_len = len(self.b[0])
        
        self.x = [0]*self.li_len
        
        self.b0 = m
        self.g0 = y

        self.x_last = [0]*self.li_len
        self.attemp = 0
        

    def _calc_diag(self):
        diag = []

        for i in range(self.li_len):
            diag.append(self.b[i][i])
        
        self.diag = diag



    def _calc_b(self):
        
        b = self.b

        # divide pela diagonal, troca sinal, zera diagonal
        for li in range(self.li_len):
            for col in range(self.col_len):
                
                if col == li:
                    
                    b[li][col] = 0
                
                else:
 
                    if(self.diag[li] != 0):
                        b[li][col] = -self.b[li][col]/self.diag[li]
                time.sleep(0.05)
                
 
        self.b = b
    
    
    
    def _calc_g(self):
        
        g = [0]*self.col_len

        for i in range(self.col_len):
            
            if(self.diag[i] != 0):
                g[i] = self.g[i]/self.diag[i]
            

        self.g = g
    
    

    def _calc_line_abs(self, line):

        line_abs = abs(line[0])
        
        for i in range(self.col_len):

            if abs(line[i]) > line_abs:

                line_abs = abs(line[i])

        return line_abs



    def _calc_error(self):
        
        # err = ||error_a||/||x||
        error_a = [0]*self.li_len

        for i in range(self.li_len):

            error_a[i] = self.x[i] - self.x_last[i]

        line_abs_a = self._calc_line_abs(error_a)
        line_abs_x = self._calc_line_abs(self.x)

        self.error = (line_abs_a/line_abs_x)

    


    def _calc_x(self):

        bx = [0]*self.col_len

        for i in range(self.li_len):
            for j in range(self.col_len):

                bx[i] = bx[i] + self.b[i][j]*self.x[j]
                
        for i in range(self.li_len):

            bx[i] = bx[i] + self.g[i]
                
        self.x = bx



    def solve(self):
        # main

        # x(k+1) = bx(k) + g

        # find diag
        
        if self.attemp == 0:

            self._calc_diag()
            
            self._calc_g()
            
            self._calc_b()
            

        self._calc_x()

        
        
        self._calc_error()

        self.x_last = self.x


        if self.error > pow(10, -self.p) and self.attemp < self.max_attemps:
            
            self.attemp = self.attemp + 1
            self.solve()
        
        return self.x

if __name__ == "__main__":
    # li0 = [15, 5, -5]
    # li1 = [4, 10, 1]
    # li2 = [2, -2, 8]

    # m = [li0, li1, li2]

    # y = [30, 23, -10]


    # li0 = [2, 3]
    # li1 = [1, -5]
    # # li2 = [2, -2, 8]

    # m = [li0, li1]

    # y = [4, 2]


    m = [[1, 0, 0, 0, 0, 0, 0, 0],
    [1, 0.5, 0.25, 0.125, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0.5, 0.25, 0.125],
    [0, 0, 0, 0, 1, 1, 1, 1],
    [0, 1, 1.0, 0.75, 0, -1, -1.0, -0.75],
    [0, 0, 2, 3.0, 0, 0, -2, -3.0],
    [0, 0, 2, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 2, 6]]
    y = [3, 1.8616, 1.8616, -0.5571, 0, 0, 0, 0]

    j = Jacobi(m, y, 2, 10000)
    r = j.solve()
    print(r)