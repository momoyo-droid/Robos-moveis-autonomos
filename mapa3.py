

import math

import matplotlib.pyplot as plt

show_animation = True

cont = 0

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
            Inicializa o mapa para algoritmo A*

            ox: posição x dos obstaculos [m]
            oy: posição y dos obstaculos [m]
            resolution: grid resolução [m]
            rr: raio do robo
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index do grid
            self.y = y  # index do grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A* path finder 

        input:
            s_x: posição x inicio[m]
            s_y: posição y inicio[m]
            gx: posição x GPS [m]
            gy: posição y GPS [m]

        output:
            rx: x lista de posição do caminho final
            ry: y lista de posição do caminho final
        """


        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # mostra grafico
            if show_animation:  
                
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                #para parar a simulação com a tecla esc
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

        
            del open_set[c_id]

           
            closed_set[c_id] = current

            # expande a busca
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                #se o nó não for seguro, não faz nada
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # descobre um novo nó
                else:
                    if open_set[n_id].cost > node.cost:
                        # esse caminho é o melhor até agora, armazena
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # gera caminho final
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        cont = 0
        xy = []
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
            cont = cont + 1
            if cont == 2:
               #Armazena no vetor os pontos em escala do Vrep
               xy.insert(0,[(self.calc_grid_position(n.x, self.min_x))/10,self.calc_grid_position(n.y, self.min_y)/10])
               cont = 0
        #Printa os Waypoints no formato do Vrep
        print(xy)
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # checa colisão
        if self.obstacle_map[node.x][node.y]:
            return False

        return True
     
    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
       
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
     

        # gera obstaculos do mapa
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # coordenadas inicias do robo
    sy = -11.8 * 10  # [m]
    sx = -3.0 * 10 # [m]
    # coordenadas do objetivo
    gx = 0 * 10 # [m]
    gy = 11* 10# [m]
    #grid resolução
    grid_size = 10.0  # [m]
    #tamanho do robo
    robot_radius = 7.2  # [m]

    #gera obstaculos do mapa
    ox, oy = [], []
    for i in range(-125, 125):
        ox.append(i)
        oy.append(-125.0)
    for i in range(-125, 125):
        ox.append(125.0)
        oy.append(i)
    for i in range(-125, 125):
        ox.append(-125.0)
        oy.append(i)
    for i in range(-125, 125):
        ox.append(i)
        oy.append(125.0)
    #paredes de baixo
    for i in range(-125, -86): #OK
        ox.append(-125.0+118.0)
        oy.append(i)
    for i in range(-125, -86): #OK
        ox.append(29.0) 
        oy.append(i)
    for i in range(29, 47): #OK
        ox.append(i)
        oy.append(-86)
    for i in range(29, 47): #OK
        ox.append(i)
        oy.append(-136 + i)
    for i in range(-86, -31): #OK
        ox.append(47)
        oy.append(i)
    for i in range(25, 47): #OK
        ox.append(i)
        oy.append(-60)
    #diagonais
    for i in range(0, 30): #OK
        ox.append(-7-i)
        oy.append(-86+i)
    for i in range(0, 14): #OK
        ox.append(25-(i*1.2))
        oy.append(-60+i)
    #parede
    for i in range(-12, 9): #OK
        ox.append(i)
        oy.append(-46)  
    #diagonal #OK
    for i in range(0, 26):
        ox.append(-12-(i*1.1))
        oy.append(-46+i)  
    #diagonal #OK
    for i in range(0, 25):
        ox.append(47-(i*1.2))
        oy.append(-31+i)
    #diagonal da esquerda 
    for i in range(0, 43): #OK
        ox.append(-125+i)
        oy.append(-40+i) 
    #diagonal de baixo OK
    for i in range(0, 16):
        ox.append(-50+(i))
        oy.append(-125+(i*2))
    #parede de cima #OK
    for i in range(-8,43):
        ox.append(58)
        oy.append(i)
    #parede de cima
    for i in range(-125,-28):
        ox.append(i)
        oy.append(43)
    #mini diagonal esquerda
    for i in range(-28,-14):
        ox.append(i)
        oy.append(71+i)
    #mini diagonal direita
    for i in range(-14,-2):
        ox.append(i)
        oy.append(43-i)
    #parede de cima
    for i in range(-2,58):
        ox.append(i)
        oy.append(43)
    

    if show_animation: 
        plt.plot(ox, oy, ".k") #plota o mapa
        plt.plot(sx, sy, "og") #plota o robo
        plt.plot(gx, gy, "xb") #plota o objetivo
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  
        plt.plot(rx, ry, "-r") #plota o caminho
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
