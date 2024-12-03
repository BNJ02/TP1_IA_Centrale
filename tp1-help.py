# Example of world building, display, and successor computation for the artificial 
# intelligence path-finding lab
#
# Author: Didier Lime
# Date: 2018-10-03

from copy import deepcopy
import heapq
from random import random
from sys import stdout
import time

class world:
    # initialise the world
    # L is the number of columns
    # H is the number of lines
    # P is the probability of having a wall in a given tile
    def __init__(self, L, H, P, c=10):
        self.L = L 
        self.H = H

        # the world is represented by an array with one dimension
        self.w = [0 for i in range(L*H)] # initialise every tile to empty (0)

        # the costs of the tiles are also stored in an array with one dimension
        self.costs = [1 for i in range(L*H)] # initialise every tile to cost 1

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
                idx = i * L + j
                # add a wall in this tile with probability P and provided that it is neither
                # the starting tile nor the goal tile 
                if random() < P and not (i == 1 and j == 1) and not (i == H-2 and j == L-2):
                    self.w[idx] = 1

                # add a high cost zone in the middle of the world
                if L // 3 <= j < 2 * L // 3 and 0 <= i < H:
                    self.costs[idx] = c  # zone with high cost

    # display the world
    def display(self):
        for i in range(self.H):
            for j in range(self.L):
                idx = i * self.L + j
                if self.w[idx] == 0:
                    if self.costs[idx] > 1:
                        stdout.write('*')
                    else:
                        stdout.write('.')
                elif self.w[idx] == 1:
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
        Anime le chemin trouvé sur la console, avec affichage des coûts.
        Les étapes du chemin sont affichées une par une.
        """
        grid = deepcopy(self.w)  # Copie de la grille des murs
        costs = deepcopy(self.costs)  # Copie de la grille des coûts

        for step in path:
            # Convertir l'indice 1D en coordonnées 2D
            x = step // self.L
            y = step % self.L
            # Marquer le chemin avec un caractère spécial
            grid[x * self.L + y] = 2  # Chemin marqué
            self._display_animation(grid, costs)
            time.sleep(0.2)  # Pause pour l'animation

    # Display the final path
    def display_final_path(self, path):
        """
        Affiche le monde avec le chemin final spécifié en paramètre.
        Le chemin est affiché avec des couleurs spéciales (vert pour le chemin).
        """
        grid = deepcopy(self.w)  # Copie de la grille des murs
        costs = deepcopy(self.costs)  # Copie de la grille des coûts

        # Marquer chaque tile du chemin avec une couleur spécifique (chemin = 'o')
        for step in path:
            x = step // self.L
            y = step % self.L
            grid[x * self.L + y] = 2  # Marquer le chemin

        self._display(grid, costs)

    # Private method to display the grid
    def _display(self, grid, costs):
        """
        Affiche la grille avec les coûts.
        - '.' pour une tuile vide à coût normal
        - '*' pour une tuile à coût élevé
        - 'W' pour un mur
        - 'o' pour le chemin suivi (en vert)
        """

        # Parcourir la grille et afficher chaque tuile
        for i in range(self.H):
            for j in range(self.L):
                idx = i * self.L + j
                tile = grid[idx]

                # Identifier le type de tuile
                if tile == 1:
                    stdout.write('\033[31mW\033[0m')  # Rouge pour les murs
                elif tile == 2:
                    stdout.write('\033[32mo\033[0m')  # Vert pour le chemin
                elif costs[idx] > 1:
                    stdout.write('\033[33m*\033[0m')  # Jaune pour les coûts élevés
                else:
                    stdout.write('\033[37m.\033[0m')  # Gris pour les cases normales
            stdout.write('\n')  # Nouvelle ligne après chaque ligne de la grille
    
    # Private method to display the grid with animation
    def _display_animation(self, grid, costs):
        """
        Affiche une version animée de la grille avec les coûts.
        - '.' pour une tuile vide à coût normal
        - '*' pour une tuile à coût élevé
        - 'W' pour un mur
        - 'o' pour le chemin suivi (en vert)
        """
        stdout.write("\033[H\033[J")  # Efface la console

        # Parcourir la grille et afficher chaque tuile
        for i in range(self.H):
            for j in range(self.L):
                idx = i * self.L + j
                tile = grid[idx]

                # Identifier le type de tuile
                if tile == 1:  # Mur
                    stdout.write('\033[31mW\033[0m')  # Rouge pour les murs
                elif tile == 2:  # Chemin suivi
                    stdout.write('\033[32mo\033[0m')  # Vert pour le chemin
                elif costs[idx] > 1:  # Tuile avec un coût élevé
                    stdout.write('\033[33m*\033[0m')  # Jaune pour les coûts élevés
                else:  # Tuile vide avec coût normal
                    stdout.write('\033[37m.\033[0m')  # Gris pour les cases normales

            stdout.write('\n')  # Nouvelle ligne après chaque ligne de la grille
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
                print(f"Length of path : {len(path)}")
                print(f"Total cost of the path : {sum(self.costs[step] for step in path)}")
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
                print(f"Length of path : {len(path)}")
                print(f"Total cost of the path : {sum(self.costs[step] for step in path)}")
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
        return (False, [])

    # Dijkstra's algorithm
    # starting from tile number s0, find a path to tile number t
    # return (r, path, cost) where r is true if such a path exists, false otherwise
    # path contains the path if it exists and cost contains the cost of the path
    def dijkstra(self, s0, t):
        """
        Implémente l'algorithme de Dijkstra basé sur l'image fournie.
        Trouve le chemin avec le coût minimal de s0 à t.
        
        s0: Indice de départ
        t: Indice d'arrivée
        """
        # Initialisation des structures
        pred = {s: None for s in range(self.L * self.H)}  # Pas de prédécesseur au départ
        cost = {s: float('inf') for s in range(self.L * self.H)}  # Coût initial infini
        cost[s0] = 0  # Coût de la case de départ
        W = [(0, s0)]  # File de priorité initiale avec s0
        r = False  # Pas encore trouvé de chemin
        max_waiting_size = 0 # Max size of the waiting list

        while W and not r:
            max_waiting_size = max(max_waiting_size, len(W))  # Update the max size
            # Extraire l'élément avec le coût minimal
            current_cost, s = heapq.heappop(W)

            # Vérifier si nous avons atteint la cible
            if s == t:
                r = True
            else:
                # Parcourir les successeurs
                for s_prime in self.successors(s):
                    new_cost = cost[s] + self.costs[s_prime]
                    if new_cost < cost[s_prime]:  # Si un meilleur chemin est trouvé
                        cost[s_prime] = new_cost
                        pred[s_prime] = s  # Mémoriser le prédécesseur
                        heapq.heappush(W, (new_cost, s_prime))  # Ajouter à la file

        # Reconstruction du chemin
        if r:
            path = []
            current = t
            while current is not None:
                path.append(current)
                current = pred[current]
            path.reverse()

            # Afficher les résultats
            print("Length of path : ", len(path))
            print(f"Total cost of the path : {sum(self.costs[step] for step in path)}")
            print(f"Number of visited tiles : {len(pred) - list(pred.values()).count(None)}")
            return True, path, cost[t]
        else:
            print("No path found.")
            return False, [], float('inf')

    # A* algorithm
    # starting from tile number s0, find a path to tile number t
    # return (r, path) where r is true if such a path exists, false otherwise
    # and path contains the path if it exists
    def a_star(self, s0, t):
        open_list = []
        closed_list = set()
        g = {s0: 0}  # coût pour atteindre chaque nœud
        h = {s0: self.manhattan_heuristic(s0, t)}  # heuristique
        f = {s0: g[s0] + h[s0]}  # coût total
        came_from = {}  # pour reconstruire le chemin

        heapq.heappush(open_list, (f[s0], s0))

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == t:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()

                # Ajouter le nœud de départ à l'élément 0
                path.insert(0, s0)

                # Display the results
                print(f"Length of path : {len(path)}")
                print(f"Total cost of the path : {sum(self.costs[step] for step in path)}")
                print(f"Number of visited tiles : {len(closed_list)}")

                return True, path

            closed_list.add(current)

            for neighbor in self.successors(current):
                if neighbor in closed_list:
                    continue
                tentative_g = g[current] + self.costs[neighbor]
                if neighbor not in g or tentative_g < g[neighbor]:
                    came_from[neighbor] = current
                    g[neighbor] = tentative_g
                    h[neighbor] = self.manhattan_heuristic(neighbor, t)
                    f[neighbor] = g[neighbor] + h[neighbor]
                    if not any(neighbor == item[1] for item in open_list):
                        heapq.heappush(open_list, (f[neighbor], neighbor))

        return False, []

    def manhattan_heuristic(self, current, target):
        cx, cy = divmod(current, self.L)
        tx, ty = divmod(target, self.L)
        return abs(cx - tx) + abs(cy - ty)

# Entry point of the program
if __name__ == "__main__":
    # create a world
    w = world(20, 10, 0.2)

    ##### Display the initial world ##### 
    print("\nInitial world:")
    w.display_final_path([])  # Display the world without a path

    ##### Define the start and goal tiles #####
    start_tile = w.L + 1  # Tile (1, 1)
    goal_tile = (w.H - 2) * w.L + (w.L - 2)  # Tile (H-2, L-2)
    print("Initial tile:", start_tile)
    print("Goal tile:", goal_tile)


    ##### Résoudre avec DFS #####
    print("\n---Depth-first search---")
    result, path = w.dfs(start_tile, goal_tile)

    if result:
        w.display_final_path(path)   

    ##### Résoudre avec BFS #####
    print("\n---Breadth-first search---")
    result, path = w.bfs(start_tile, goal_tile)

    if result:
        w.display_final_path(path)

    ##### Résoudre avec Dijkstra #####
    print("\n---Dijkstra's algorithm---")
    result, path, cost = w.dijkstra(start_tile, goal_tile)

    if result:
        w.display_final_path(path)

    ##### Résoudre avec A* #####
    print("\n---A* algorithm---")
    result, path = w.a_star(start_tile, goal_tile)

    if result:
        w.display_final_path(path)
    