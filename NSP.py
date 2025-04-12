from scipy.integrate import solve_ivp
import numpy as np

class NSP():
    """
    This class models an N-pendulum system with springs instead of rods (minimum N is 2).
     
    Parameters
    ----------
    M : vector of floats
        the masses that make up the N-pendulum
    K : vector of floats
        the spring constants of the springs in between the masses
    L : vector of floats
        the equilibrium lengths if the springs in between the masses

    Methods
    -------
    dy_dt(y, t)
        Returns the right side of the differential equation in vector y, 
        given time t and the corresponding value of y.
    """
    def __init__(self, M, K, L):
        self.M = M
        self.K = K
        self.L = L
    
    def dy_dt(self, t, y):
        g = 20
        derivative = np.zeros(len(y))
        half_index = int(len(y)/2)
        y_index = int(half_index/2)

        derivative[:half_index] = y[half_index:]

        spr_len_1 = np.sqrt(y[0]**2 + y[y_index]**2)
        spr_len_2 = np.sqrt((y[1]-y[0])**2 + (y[y_index + 1]-y[y_index])**2)

        derivative[half_index] = (self.K[1]/self.M[0])*(y[1]-y[0])*(1-(self.L[1]/spr_len_2)) \
                                            - (self.K[0]/self.M[0])*y[0]*(1-self.L[0]/spr_len_1)
        
        derivative[half_index + y_index] = -g + (self.K[1]/self.M[0])*(y[y_index+1]-y[y_index]) \
                                            *(1-(self.L[1]/spr_len_2)) - (self.K[0]/self.M[0]) \
                                            *y[y_index]*(1-self.L[0]/spr_len_1)
        for i in range(1, y_index - 1):
            spr_len_1 = np.sqrt((y[i]-y[i-1])**2 + (y[y_index+i]-y[y_index+i-1])**2)
            spr_len_2 = np.sqrt((y[i+1]-y[i])**2 + (y[y_index+i+1]-y[y_index+i])**2)

            derivative[half_index + i] = (self.K[i+1]/self.M[i])*(y[i+1]-y[i])*(1-(self.L[i+1]/spr_len_2)) \
                                                - (self.K[i]/self.M[i])*(y[i]-y[i-1])*(1-self.L[i]/spr_len_1)
            
            derivative[half_index + y_index + i] = -g + (self.K[i+1]/self.M[i])*(y[y_index+i+1]-y[y_index+i]) \
                                                *(1-(self.L[i+1]/spr_len_2)) - (self.K[i]/self.M[i]) \
                                                *(y[y_index + i] - y[y_index+i-1])*(1-self.L[i]/spr_len_1)
        
        spr_len_1 = np.sqrt((y[y_index-1]-y[y_index-2])**2 + (y[half_index-1]-y[half_index-2])**2)

        derivative[half_index+y_index-1] = -(self.K[-1]/self.M[-1])*(y[y_index-1]-y[y_index-2])*(1-self.L[-1]/spr_len_1)
        
        derivative[-1] = -g - (self.K[-1]/self.M[-1])*(y[half_index-1]-y[half_index-2])*(1-self.L[-1]/spr_len_1)
                                                                        
        return derivative
    
    def solve_ode(self, x_0, y_0, x_dot_0, y_dot_0, t_pts, method='RK45', abserr=1.0e-10, relerr=1.0e-10):
        y = np.concatenate((x_0, y_0, x_dot_0, y_dot_0))
        solution = solve_ivp(self.dy_dt, (t_pts[0], t_pts[-1]), 
                             y, t_eval=t_pts, method=method, 
                             atol=abserr, rtol=relerr)
        return solution.y