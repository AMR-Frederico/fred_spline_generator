# S = s0 + s1 + ... + sn
# as curvas s'ao do terceiro grau
# as curvas possuem derivadas iguais nas suas intersecoes adjacentes
# as curvas possuem derivadas segundas iguais nas suas intersecoes adjacentes
# para n pontos teremos n-1 polinomios
# as derivadas nas extremidades sao 0
# 4*(n-1) icognitas

# si = ai + bix + cix**2 + dix**3 

# si = s(i+1) 
# ai + bix + cix**2 + dix**3 = a(i+1) + b(i+1)x + c(i+1)x**2 + d(i+1)x**3 

# si' = s(i+1)'
# bi + 2*cix + 3*dix**2 = b(i+1) + 2*c(i+1)x + 3*d(i+1)x**2

  
# si'' = s(i+1)''
# 2*ci + 6*dix = 2*c(i+1) + 6*d(i+1)x 

# s0'' = sn'' 0
# 2c0 + 6d0x = 0
# 2cn + 6dnx = 0

# [coeficientes] * [variaveis] = [y]

# exemplo:

# 3 pontos 
# 4*(n-1) icognitas -> precisamos de 8 equacoes linearmente independentes

# para o ponto do meio possuimos as equacoes
# s0(bx) = by 
# s1(bx) = by
# s0'(bx) = s1'(bx) 
# s0''(bx) = s1''(bx) 

# para os pontos das extremidades temos
# s0'(ax) = 0
# s1'(cx) = 0
# s0(ax) = ay
# s1(cx) = cy


# a   b    c      d       a0   b0   c0       d0    
# 1   x0   x0^2   x0^3    0     0    0       0
# 0   0    0      0       1    x1   x1^2   x1^3
# 1   x0   x0^2   x0^3   -1   -x1   -x1^2   -x1^3
# 0   1     2x0   3x0^2   0    -1   -2x1    -3x1^2  

# y = [5, 4, 3, 2, 1]
# dy = derivator(y)

# print(y)
# print(dy)


from .gauss import Gauss

class Spline:

    def __init__(self, p, precision = 4):
        
        self.p = p
        self.p_amount = len(self.p)
        self.precision = precision    


        # create array (the python correct mode)
        self.m = []
        for i in range (4 * (self.p_amount -1)):
            self.m = self.m + [[0]*(4 * (self.p_amount -1))]
        
        self.y = [0]*(4 * (self.p_amount -1))
        self.x_line_index = 0
        self.y_line_index = 0


    def print_matrix(self, s):
        for i in range(len(s)):
            print(s[i])
        print('--------------------------------------')


    def _sx_equal_y_line_generator(self):

        for i in range(self.p_amount):

            x = self.p[i][0]
            y = self.p[i][1]

            if i != 0:

                self.m[self.x_line_index][0 + 4*(i-1)] = 1
                self.m[self.x_line_index][1 + 4*(i-1)] = x
                self.m[self.x_line_index][2 + 4*(i-1)] = pow(x, 2)
                self.m[self.x_line_index][3 + 4*(i-1)] = pow(x, 3)
                self.x_line_index = self.x_line_index + 1

                self.y[self.y_line_index] = y
                self.y_line_index = self.y_line_index + 1

            if i != (self.p_amount -1):

                self.m[self.x_line_index][0 + 4*i] = 1
                self.m[self.x_line_index][1 + 4*i] = x
                self.m[self.x_line_index][2 + 4*i] = pow(x, 2)
                self.m[self.x_line_index][3 + 4*i] = pow(x, 3)
                self.x_line_index = self.x_line_index +1

                self.y[self.y_line_index] = y
                self.y_line_index = self.y_line_index + 1





    def _equal_derivated_line_generator(self):

        for i in range(self.p_amount):


            # without extremities
            if (i != 0) and (i != (self.p_amount -1)):
                
                # first derivate
                x = self.p[i][0]

                dfx = [0, 1, 2*x, 3*pow(x, 2)]


                self.m[self.x_line_index][0 + 4*(i-1)] = dfx[0]
                self.m[self.x_line_index][1 + 4*(i-1)] = dfx[1]
                self.m[self.x_line_index][2 + 4*(i-1)] = dfx[2]
                self.m[self.x_line_index][3 + 4*(i-1)] = dfx[3]


                self.m[self.x_line_index][0 + 4*i] = -dfx[0]
                self.m[self.x_line_index][1 + 4*i] = -dfx[1]
                self.m[self.x_line_index][2 + 4*i] = -dfx[2]
                self.m[self.x_line_index][3 + 4*i] = -dfx[3]
                self.x_line_index = self.x_line_index +1

                self.y[self.y_line_index] = 0
                self.y_line_index = self.y_line_index + 1

                # second derivate
                ddfx = [0, 0, 2, 6*x]



                self.m[self.x_line_index][0 + 4*(i-1)] = ddfx[0]
                self.m[self.x_line_index][1 + 4*(i-1)] = ddfx[1]
                self.m[self.x_line_index][2 + 4*(i-1)] = ddfx[2]
                self.m[self.x_line_index][3 + 4*(i-1)] = ddfx[3]


                self.m[self.x_line_index][0 + 4*i] = -ddfx[0]
                self.m[self.x_line_index][1 + 4*i] = -ddfx[1]
                self.m[self.x_line_index][2 + 4*i] = -ddfx[2]
                self.m[self.x_line_index][3 + 4*i] = -ddfx[3]
                self.x_line_index = self.x_line_index +1

                self.y[self.y_line_index] = 0
                self.y_line_index = self.y_line_index + 1





    def _extremities_second_derivated_zero_line_generator(self):



     
        x = self.p[0][0]

        # second derivate
        ddfx = [0, 0, 2, 6*x]


        self.m[self.x_line_index][0] = ddfx[0]
        self.m[self.x_line_index][1] = ddfx[1]
        self.m[self.x_line_index][2] = ddfx[2]
        self.m[self.x_line_index][3] = ddfx[3]
        self.x_line_index = self.x_line_index +1

        self.y[self.y_line_index] = 0
        self.y_line_index = self.y_line_index + 1

        

        x = self.p[self.p_amount -1][0]

        # second derivate
        ddfx = [0, 0, 2, 6*x]

        i = self.p_amount-1

        self.m[self.x_line_index][0 + 4*(i-1)] = ddfx[0]
        self.m[self.x_line_index][1 + 4*(i-1)] = ddfx[1]
        self.m[self.x_line_index][2 + 4*(i-1)] = ddfx[2]
        self.m[self.x_line_index][3 + 4*(i-1)] = ddfx[3]
        self.x_line_index = self.x_line_index +1

        self.y[self.y_line_index] = 0
        self.y_line_index = self.y_line_index + 1






    def calculate(self):

        self._sx_equal_y_line_generator() # n - 4 equations 
        self._equal_derivated_line_generator() # 2 equations
        self._extremities_second_derivated_zero_line_generator() # 2 equations

        gauss = Gauss(self.m, self.y, self.precision)
        solution = gauss.solve()
        self.solution = []

        for i in range(self.p_amount -1):

            self.solution.append([solution[0 + i*4], solution[1 + i*4], solution[2 + i*4], solution[3 + i*4]])

        return self.solution

        
if __name__ == "__main__":
    # print matrix
    def print_matrix(s):
        for i in range(len(s.m)):
            print(s.m[i])
        print('--------------------------------------')


    p = [[0, 3], [0.5, 1.8616], [1, -0.5571]]
    # p = [[2, 3], [1, -5]]
    s = Spline(p)
    s.calculate()
    # print_matrix(s)
    print(s.y)
    print("-------------")
    print(s.solution)
    # def ordering_matrix(m):

    #     line_len = len(m)
    #     lineBuffer = []

    #     print(m)
    #     print(m[2][0])

    #     # run cols
    #     for col in range(line_len):
            
    #         # run lines
    #         for line in range(line_len):

    #             # check if line is already in order
    #             in_order_line = False
    #             for x in range(col):
    #                 if m[line][x] != 0:
    #                     in_order_line = True

                
    #             # if element is zero...

    #             if m[line][col] == 0 and not in_order_line:
    #                 print(f"m[{line}][{col}] ", m[line][col])
                    
    #                 # check if elements below are zero 
    #                 for check_line in range(line_len):
    #                     if check_line > line:
                            
    #                         if m[check_line][col] != 0:

    #                             # print("checkline: ", check_line)
    #                             # print("line:      ", line)
    #                             # print("change: ", m[line])
    #                             # print("By:     ", m[check_line])
    #                             lineBuffer =  m[line]
    #                             m[line] = m[check_line]
    #                             m[check_line] = lineBuffer
    #     return m


    # def print_matrix(s):
    #     for i in range(len(m)):
    #         print(m[i])
    #     print('--------------------------------------')



# li0 = [0, 0, 1, ]
# li1 = [0, 4, 1]
# li2 = [3, 0, 0]

# y = [15, 10, 11]

# m = [li0, li1, li2]

# s = ordering_matrix(m)
# print_matrix(s)


