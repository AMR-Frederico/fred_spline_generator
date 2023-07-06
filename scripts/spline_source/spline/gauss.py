import time

class Gauss:

    def __init__(self, m, y, precision = 4):

        self.m = m
        self.y = y
        self.line_amount = len(m)
        self.col_aum_amount = len(m) + 1
        self.precision = precision

    def print_matrix(self, s):
        for i in range(len(s)):
            print(s[i])
        print('--------------------------------------')


    def _calc_aum(self):

        self.m_aum = self.m

        for i in range(self.line_amount):

            self.m_aum[i] = self.m[i] + [self.y[i]]



    def _ordering_matrix(self):

        line_len = len(self.m_aum)
        lineBuffer = []

        # run cols
        for col in range(line_len):
            
            # run lines
            for line in range(line_len):

                # check if line is already in order
                in_order_line = False
                for x in range(col):
                    if self.m_aum[line][x] != 0:
                        in_order_line = True

                
                # if element is zero...
                if self.m_aum[line][col] == 0 and not in_order_line:
                    
                    # check if elements below are zero 
                    for check_line in range(line_len):
                        if check_line > line:
                            
                            if self.m_aum[check_line][col] != 0:

                                # print("change: ", self.m[line])
                                # print("By:     ", self.m[check_line])
                                lineBuffer =  self.m_aum[line]
                                self.m_aum[line] = self.m_aum[check_line]
                                self.m_aum[check_line] = lineBuffer



    def _gauss_iteration(self):

        pivot_index = 0
        last_pivot_index = 0
        times = 0
        # run diagonal
        while pivot_index < self.line_amount -1:

            self._ordering_matrix()

            if pivot_index == last_pivot_index:

                times = times +1

            
            
            # self.print_matrix(self.m_aum)
            pivot = self.m_aum[pivot_index][pivot_index]
            all_el_zero_below_pivot = True

            if pivot != 0:
                # run lines
                for line in range(self.line_amount):

                    # if line is below pivot
                    if line > pivot_index:
                        
                        # if item below pivot is not zero
                        if self.m_aum[line][pivot_index] != 0.0:

                            
                            all_el_zero_below_pivot = False
                            el_below_pivot = self.m_aum[line][pivot_index]
                            
                            factor = el_below_pivot/pivot

                            # run cols in line calculating
                            for col in range(self.col_aum_amount):
                                
                                self.m_aum[line][col] = round(self.m_aum[line][col] - factor * self.m_aum[pivot_index][col], (self.precision+1))
                                # self.m_aum[line][col] = self.m_aum[line][col] - factor * self.m_aum[pivot_index][col]
            

            # if times >= 10:
            #     print(f"pivot index: {pivot_index}/{self.line_amount -1}")
            #     self.print_matrix(self.m_aum)
            #     print("pivot:", self.m_aum[pivot_index][pivot_index])
            #     print("pivot line:", self.m_aum[pivot_index])
            #     print("all_el_zero_below_pivot:", all_el_zero_below_pivot)
            #     print("pivot lines:")
            #     for line in range(self.line_amount):

            #         if line == pivot_index:
            #             print(f"{self.m_aum[line][pivot_index]}<---")
            #         else:
            #             print(self.m_aum[line][pivot_index])
                    
            #         if self.m_aum[line][pivot_index] != 0:

            #             print("el != 0 -> ",self.m_aum[line][pivot_index])

            #     # break


            # if end lines and all el below pivot are zero...
            if all_el_zero_below_pivot:
                # print("plus pivot")
                pivot_index = pivot_index + 1

            last_pivot_index = pivot_index

            time.sleep(0.1)


    
    def _retroactive_resolution(self):

        solution = [0]*(self.col_aum_amount-1)
        solution_len = len(solution)

        # run lines from bottom to top
        for i in range(self.line_amount):

            line = self.line_amount -i -1
            
            # -sum independent terms
            sum_independent_terms = 0
            for j in range(i):
                
                col = self.col_aum_amount -2 -j 
                sum_independent_terms = sum_independent_terms - self.m_aum[line][col]*solution[solution_len-1-j]

            y = self.m_aum[line][self.col_aum_amount-1]
            y_plus_sum_ind_terms = (y + sum_independent_terms)
            dep_term = self.m_aum[line][self.col_aum_amount -2 -i]
            
            if dep_term != 0:
                
                # solution[line] = y_plus_sum_ind_terms/dep_term
                solution[line] = round(y_plus_sum_ind_terms/dep_term, self.precision + 1)

            else:
                
                solution[line] = round(y_plus_sum_ind_terms, self.precision + 1)
                # solution[line] = y_plus_sum_ind_terms
            
        
        self.solution = solution
    

    def solve(self):

        print("gauss: calculating aum matrix...")
        self._calc_aum()
        print("gauss: applying gauss method...")
        self._gauss_iteration()
        print("gauss: applying retroactive resolution...")
        self._retroactive_resolution()

        return self.solution   

            


            









if __name__ == "__main__":

    # li0 = [3, 2, 4]
    # li1 = [1, 1, 2]
    # li2 = [4, 3, -2]

    # y = [1, 2, 3]
    

    # li0 = [5, 5, 0]
    # li1 = [2, 4, 1]
    # li2 = [3, 4, 0]
    
    # y = [15, 10, 11]

    # m = [li0, li1, li2]

    # li0 = [7, 1, 3, 1]
    # li2 = [3, 4, 0]
    
    # y = [15, 10, 11]

    # m = [li0, li1, li2]


    li0 = [1,   0,   0,    0,     0,    0,    0,     0]
    li1 = [1.0, 0.5, 0.25, 0.125, 0.0,  0.0,  0.0,   0.0]
    li2 = [0.0, 0.0, 0.0,  0.0,   1.0,  0.5,  0.25,  0.125]
    li3 = [0,   0,   0,    0,     1,    1,    1,     1]
    li4 = [0.0, 1.0, 1.0,  0.75,  0.0,  -1,  -1,    -0.75]
    li5 = [0.0, 0.0, 2.0,  3.0,   0.0,  0.0, -2.0,  -3.0]
    li6 = [0.0, 0.0, 2.0,  0.0,   0.0,  0.0,  0.0,   0.0]
    li7 = [0.0, 0.0, 0.0,  0.0,   0.0,  0.0,  2.0,   6.0]

    m = [li0, li1, li2, li3, li4, li5, li6, li7]
    y = [3, 1.8616, 1.8616, -0.5571, 0, 0, 0, 0]

    # li0 = [1,   0,   0,    0,     0,    0,    0,     0]
    # li1 = [1.0, 0.5, 0.25, 0.125, 0.0,  0.0,  0.0,   0.0]
    # li4 = [0.0, 1.0, 1.0,  0.75,  0.0,  -1,  -1,    -0.75]
    # li5 = [0.0, 0.0, 2.0,  3.0,   0.0,  0.0, -2.0,  -3.0]
    # li6 = [0.0, 0.0, 2.0,  0.0,   0.0,  0.0,  0.0,   0.0]
    # li2 = [0.0, 0.0, 0.0,  0.0,   1.0,  0.5,  0.25,  0.125]
    # li3 = [0,   0,   0,    0,     1,    1,    1,     1]
    # li7 = [0.0, 0.0, 0.0,  0.0,   0.0,  0.0,  2.0,   6.0]

    # y0 = 3
    # y1 = 1.8616
    # y4 = 0
    # y5 = 0
    # y6 = 0
    # y2 = 1.8616
    # y3 = -0.5571
    # y7 = 0

    # m = [li0, li1, li4, li5, li6, li2, li3, li7]

    # y = [y0, y1, y4, y5, y6, y2, y3, y7]



    def print_matrix(s):
        for i in range(len(s)):
            print(s[i])
        print('--------------------------------------')



    g = Gauss(m, y)
    g._calc_aum()
    # print("g.m_aum")
    # print_matrix(g.m_aum)
    g._gauss_iteration()
    print_matrix(g.m_aum)
    g._retroactive_resolution()
    print("g.solution")
    print(g.solution)
