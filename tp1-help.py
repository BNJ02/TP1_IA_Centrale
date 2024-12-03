# Example of world building, display, and successor computation for the artificial 
# intelligence path-finding lab
#
# Author: Didier Lime
# Date: 2018-10-03

from copy import deepcopy
from random import random
from sys import stdout
import time

class world:
    # initialise the world
    # L is the number of columns
    # H is the number of lines
    # P is the probability of having a wall in a given tile
    def __init__(self, L, H, P):
        self.L = L 
        self.H = H

        # the world is represented by an array with one dimension
        self.w = [0 for i in range(L*H)] # initialise every tile to empty (0)

        # add walls in the first and last columns
        for i in range(H):
            self.w[i*L] = 1
            self.w[i*L+L-1] = 1
        
        # add walls in the first and last lines
        for j in range(L):
            self.w[j] = 1
            self.w[(H-1)*L + j] = 1

        for i in range(H):
            for j in range(L):
                # add a wall in this tile with probability P and provided that it is neither
                # the starting tile nor the goal tile 
                if random() < P and not (i == 1 and j == 1) and not (i == H-2 and j == L-2):
                    self.w[i*L+j] = 1

    # display the world
    def display(self):
        for i in range(self.H):
            for j in range(self.L):
                if self.w[i * self.L + j] == 0:
                    stdout.write('.')
                elif self.w[i * self.L + j] == 1:
                    stdout.write('W')

            print('')

    # compute the successors of tile number i in world w
    def successors(self, i):
        if i < 0 or i >= self.L * self.H or self.w[i] == 1:
            # i is an incorrect tile number (outside the array or on a wall)
            return [] 
        else:
            # look in the four adjacent tiles and keep only those with no wall
            return list(filter(lambda x: self.w[x] != 1, [i - 1, i + 1, i - self.L, i + self.L]))
    
    # Display the path found by the search algorithm
    def animate_path(self, path):
        """
        Anime le chemin trouvé sur la console.
        Les étapes du chemin sont affichées une par une.
        """
        grid = deepcopy(self.w)  # Crée une copie de la grille
        for step in path:
            # Convertir l'indice 1D en coordonnées 2D
            x = step // self.L
            y = step % self.L
            # Marquer le chemin avec un caractère spécial
            grid[x * self.L + y] = 2  # Chemin marqué
            self._display_animation(grid)
            time.sleep(0.2)  # Pause pour l'animation

    # Private method to display the grid
    def _display_animation(self, grid):
        """
        Affiche une version animée de la grille.
        """
        stdout.write("\033[H\033[J")  # Efface la console
        for i in range(self.H):
            for j in range(self.L):
                tile = grid[i * self.L + j]
                if tile == 0:
                    stdout.write('\033[37m.\033[0m')  # Case vide (gris)
                elif tile == 1:
                    stdout.write('\033[31mW\033[0m')  # Mur (rouge)
                elif tile == 2:
                    stdout.write('\033[32mo\033[0m')  # Chemin (vert)
            stdout.write('\n')
        stdout.flush()

    # Depth-first search
    # starting from tile number s0, find a path to tile number t
    # return (r, path) where r is true if such a path exists, false otherwise
    # and path contains the path if it exists  
    def dfs(self, s0, t):      
        # Stack for tiles to explore, initialized with the starting tile
        stack = [s0]
        # Set of visited tiles
        visited = set()
        # Dictionary to reconstruct the path
        parent = {}
        # Max size of the waiting list
        max_waiting_size = 0

        while stack:
            max_waiting_size = max(max_waiting_size, len(stack))  # Update the max size
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)

            # Check if we have reached the goal
            if current == t:
                path = []
                # Reconstruct the path
                while current is not None:
                    path.append(current)
                    current = parent.get(current)
                path.reverse()

                # Afficher les résultats
                print(f"Path found : {path}")
                print(f"Length of path : {len(path)}")
                print(f"Number of visited tiles : {len(visited)}")
                print(f"Max size of the waiting list : {max_waiting_size}")
                return (True, path)

            # Explore successors
            for neighbor in self.successors(current):
                if neighbor not in visited:
                    stack.append(neighbor)
                    # Record the parent to reconstruct the path later
                    parent[neighbor] = current

        # If we exit the loop, no path was found
        print("No path found.")
        print(f"Number of visited tiles : {len(visited)}")
        print(f"Max size of the waiting list : {max_waiting_size}")
        return (False, [])
    
    # Breadth-first search
    # starting from tile number s0, find a path to tile number t
    # return (r, path) where r is true if such a path exists, false otherwise
    # and path contains the path if it exists
    def bfs(self, s0, t):
        # Queue for tiles to explore, initialized with the starting tile
        queue = [s0]
        # Set of visited tiles
        visited = set()
        # Dictionary to reconstruct the path
        parent = {}
        # Max size of the waiting list
        max_waiting_size = 0

        while queue:
            max_waiting_size = max(max_waiting_size, len(queue))  # Update the max size
            current = queue.pop(0)  # Remove the first element
            if current in visited:
                continue
            visited.add(current)

            # Test if we have reached the goal
            if current == t:
                path = []
                # Reconstruct the path
                while current is not None:
                    path.append(current)
                    current = parent.get(current)
                path.reverse()

                # Display the results
                print(f"Path found : {path}")
                print(f"Length of path : {len(path)}")
                print(f"Number of visited tiles : {len(visited)}")
                print(f"Max size of the waiting list : {max_waiting_size}")
                return (True, path)

            # Explore successors
            for neighbor in self.successors(current):
                if neighbor not in visited and neighbor not in queue:
                    queue.append(neighbor)
                    parent[neighbor] = current

        # If we exit the loop, no path was found
        print("No path found.")
        print(f"Number of visited tiles : {len(visited)}")
        print(f"Max size of the waiting list : {max_waiting_size}")
        return (False, [])


# create a world
w = world(20, 10, 0.2)

# # display it 
# w.display()

# # print the tile numbers of the successors of the starting tile (1, 1)
# print(w.successors(w.L + 1))

start_tile = w.L + 1  # Tile (1, 1)
goal_tile = (w.H - 2) * w.L + (w.L - 2)  # Tile (H-2, L-2)
print("Initial tile:", start_tile)
print("Goal tile:", goal_tile)

# # Résoudre avec DFS
# result, path = w.dfs(start_tile, goal_tile)

# if result:
#     w.animate_path(path)   
#     w.dfs(start_tile, goal_tile)  # Afficher les statistiques

# Résoudre avec BFS
result, path = w.bfs(start_tile, goal_tile)

if result:
    w.animate_path(path)
    w.bfs(start_tile, goal_tile)  # Afficher les statistiques
