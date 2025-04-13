from manim import *
import numpy as np
from NSP import NSP

class SpringPendulum(Scene):
    
    def construct(self):
        def update_spring(Point1, Point2, zags, width):
            center_1 = Point1.get_center()
            center_2 = Point2.get_center()

            vec = center_2 - center_1

            normal_to_vec = np.zeros_like(vec)
            normal_to_vec[0] = vec[1]
            normal_to_vec[1] = -vec[0]
            normal_to_vec = width*normal_to_vec / np.linalg.norm(normal_to_vec)

            corners = np.zeros((3*zags - 1, 3))
            corners[0] = center_1
            count = 1
            for i in range(1, 4*zags - 3, 2):
                if (count % 3 == 1):
                    corners[count] = vec * (i/(4*zags)) + normal_to_vec + corners[0]
                else:
                    corners[count] = vec * (i/(4*zags)) - normal_to_vec + corners[0]
                    corners[count + 1] = corners[count] + (normal_to_vec/1.25) - (vec/30)
                    count += 1
                count += 1

            corners[-1] = vec + corners[0]

            
            spring = VMobject(stroke_color=WHITE, stroke_width=2)
            spring.set_points_smoothly(corners)

            return spring

        N = 3
        M_vec = np.zeros(N) + 10
        K_vec = np.zeros(N) + 1000
        L_vec = np.zeros(N) + 1
        spring_pendulum = NSP(M_vec, K_vec, L_vec)

        x_0 = [0 for i in range(1, N+1)]
        y_0=[-i for i in range(1, N+1)] 
        x_dot_0 = np.zeros(N) + 2
        y_dot_0 = np.zeros(N)
        t_pts = np.arange(0, 5, 0.02)

        solution = spring_pendulum.solve_ode(x_0, y_0, x_dot_0, y_dot_0, t_pts)

        pivot = Dot().move_to(2*UP)
        Mass_Vector = [Dot() for i in range(N)]
        Spring_Vector = [VMobject() for i in range(N)]
        spr_width = 0.3
        zags = 10

        x1 = solution[0, :]
        y1 = solution[N, :]
        Mass_Vector[0] = Dot(radius=0.02*M_vec[0]).move_to(x1[0]*RIGHT+(y1[0]+2)*UP).set_color(BLUE)
        Spring_Vector[0] = update_spring(pivot, Mass_Vector[0], zags, spr_width)        
        Spring_Vector[0].add_updater(lambda x: x.become(update_spring(pivot, Mass_Vector[0], zags, spr_width)))

        for k in range(1,N):
            xk = solution[k, :]
            yk = solution[k+N, :]
            Mass_Vector[k] = Dot(radius=0.02*M_vec[k]).move_to(xk[0]*RIGHT+(yk[0]+2)*UP).set_color(BLUE)
            Spring_Vector[k] = update_spring(Mass_Vector[k-1], Mass_Vector[k], zags, spr_width)
            Spring_Vector[k].add_updater(lambda x, k=k: x.become(update_spring(Mass_Vector[k-1], Mass_Vector[k], zags, spr_width)))

        self.add(*Mass_Vector)
        self.add(*Spring_Vector)
        self.add(pivot)

        for i in range(len(x1) - 1):
            self.remove(*Mass_Vector)
            self.add(*Mass_Vector)
            animations = [ApplyMethod(Mass_Vector[m].move_to, solution[m, :][i+1]*RIGHT + (solution[m+N, :][i+1]+2)*UP) for m in range(N)]

            self.play(*animations, run_time=1/50, rate_func=linear)
