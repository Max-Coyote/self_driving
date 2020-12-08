#Class for each cell
class node():
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position

        self.G = 0 # energy from start to current
        self.H = 0 # approx energy to end point
        self.F = 0 # F = H + G

    def count_F(self):
        self.F = self.H + self.G

    def __eq__(self, other):
        return self.position == other.position

#Check for checking possibility of point(zone and walls)
def check_possibility(zone, point, closed_list):
    for i in range(len(closed_list)):
        if point == closed_list[i].position:
            return False
    if point[0] < 0 or point[1] < 0 or point[0] >= len(zone) or point[1] >= len(zone[0]):
        return False
    elif zone[point[0]][point[1]] == 1:
        return False
    else:
        return True

def check_in_open(point, open_list):
    for i in range(len(open_list)):
        if open_list[i].position == point:
            return i #position of point in open_list
    return -1 #number in no point

# Manhatton_algorithm for count cost of current point to end
# def Manhatton_algorithm(cur_node, end_node):
#     cost = 10
#     return cost * (abs(cur_node.position[0] - end_node.position[0]) + abs(cur_node.position[1] - end_node.position[1]))

#A Star algorithm

def a_star(zone, start, end):
    #Init of open and closed list
    open_list = []
    closed_list = []

    #First entry
    start_node = node(None, start)
    end_node = node(None, end)


    #Adding start point to open list
    open_list.append(start_node)

    #List of bordered cells
    #eighbours = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    #Cost to dest
    #neighbours_cost = [10, 10, 10, 10, 14, 14, 14, 14]

    #List of bordered cells
    neighbours = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    # #Cost to dest
    neighbours_cost = [10, 10, 10, 10]
    


    while len(open_list) > 0:
      #find cell with the low F in open_list
      min_F =  open_list[0].F
      min_pose = 0
      for i in range(len(open_list)):
          if i == 0: continue

          if open_list[i].F < min_F:
              min_F = open_list[i].F
              min_pose = i
      #delete node from open_list and add to closed_list
      current_node =  open_list[min_pose]
      open_list.pop(min_pose)
      closed_list.append(current_node)
      #check neighbours
      neigh_counter = 0
      for position in neighbours:
          new_pose = node(None, (current_node.position[0] + position[0], current_node.position[1] + position[1]))
          #check possibility of cell and is the cell in closed_list
          if check_possibility(zone, new_pose.position, closed_list):
              #check is the cell in open_list
              Manhatton_algorithm = lambda new_pose, end_node: 10 * (abs(new_pose.position[0] - end_node.position[0]) + abs(new_pose.position[1] - end_node.position[1]))
              open_checker = check_in_open(new_pose.position, open_list)
              if open_checker == -1:
                  new_pose.parent = current_node.position
                  new_pose.G = neighbours_cost[neigh_counter] + current_node.G
                  new_pose.H = Manhatton_algorithm(new_pose, end_node)
                  new_pose.count_F()
                  open_list.append(new_pose)
              else:
                  #if in open
                  new_pose.parent = current_node.position
                  new_pose.G = neighbours_cost[neigh_counter] + current_node.G
                  new_pose.H = Manhatton_algorithm(new_pose, end_node)
                  new_pose.count_F()
                  if open_list[open_checker].G > new_pose.G:
                      open_list.pop(open_checker)
                      open_list.append(new_pose)
          neigh_counter += 1

      if check_in_open(end_node.position, open_list) > -1:
          el = open_list[check_in_open(end_node.position, open_list)]
          path = []
          while el.parent is not None:
              path.append(el.position)
              el = closed_list[check_in_open(el.parent, closed_list)]
          path.append(el.position)
          return path

    return []

      #if end find





def main():
    #Main part of input data
    zone = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
             [0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    #zone = [[0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (4, 5)
   #9,7
    path = a_star(zone, start, end)

    #Path drawing

    print(path)
    for i in range(len(path)):
         zone[path[i][0]][path[i][1]] = 3
    #zone[start[0]][start[1]] = 8
    #zone[end[0]][end[1]] = 8
    for line in zone:
        print(line)


if __name__ == '__main__':
    main()
