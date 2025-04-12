from manim import *
import numpy as np
from NSP import NSP

class SpringPendulum(Scene):
    
    def construct(self):
        def update_spring(Point1, Point2, zags):
            center_1 = Point1.get_center()
            center_2 = Point2.get_center()

            vec = center_2 - center_1

            normal_to_vec = np.zeros_like(vec)
            normal_to_vec[0] = vec[1]
            normal_to_vec[1] = -vec[0]
            normal_to_vec = 0.3*normal_to_vec / np.linalg.norm(normal_to_vec)

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

        M_vec = [10, 10, 10]
        K_vec = [500, 500, 500]
        L_vec = [2, 2, 2]
        spring_pendulum = NSP(M_vec, K_vec, L_vec)

        x_0 = [1, 2, 3]
        y_0=[1, 2, 3]
        x_dot_0 = [0, 0, 0]
        y_dot_0 = [0, 0, 0]
        t_pts = np.arange(0, 10, 0.02)

        solution = spring_pendulum.solve_ode(x_0, y_0, x_dot_0, y_dot_0, t_pts)

        x1 = solution[0, :]/2.25
        x2 = solution[1, :]/2.25
        x3 = solution[2, :]/2.25
        y1 = solution[3, :]/2.25
        y2 = solution[4, :]/2.25
        y3 = solution[5, :]/2.25

        pivot = Dot().move_to(1*UP)
        Mass_1= Dot(radius=0.03*M_vec[0]).move_to(x1[0]*RIGHT+(y1[0]+1)*UP).set_color(BLUE)
        Mass_2= Dot(radius=0.03*M_vec[1]).move_to(x2[0]*RIGHT+(y2[0]+1)*UP).set_color(BLUE)
        Mass_3= Dot(radius=0.03*M_vec[2]).move_to(x3[0]*RIGHT+(y3[0]+1)*UP).set_color(BLUE)

        Spring_1 = update_spring(pivot, Mass_1, 8)        
        Spring_1.add_updater(lambda x: x.become(update_spring(pivot, Mass_1, 8)))
        Spring_2 = update_spring(Mass_1, Mass_2, 8)        
        Spring_2.add_updater(lambda x: x.become(update_spring(Mass_1, Mass_2, 8)))
        Spring_3 = update_spring(Mass_2, Mass_3, 8)        
        Spring_3.add_updater(lambda x: x.become(update_spring(Mass_2, Mass_3, 8)))

        self.add(pivot, Mass_1, Spring_1, Mass_2, Spring_2, Mass_3, Spring_3)

        for i in range(len(x1) - 1):
            self.remove(Mass_1)
            self.remove(Mass_2)
            self.remove(Mass_3)
            self.add(Mass_1)
            self.add(Mass_2)
            self.add(Mass_3)
            
            self.play(
                    Mass_1.animate.move_to(x1[i+1]*RIGHT + (y1[i+1]+1)*UP),
                    Mass_2.animate.move_to(x2[i+1]*RIGHT + (y2[i+1]+1)*UP),
                    Mass_3.animate.move_to(x3[i+1]*RIGHT + (y3[i+1]+1)*UP),
                    run_time=1/50,rate_func=linear)