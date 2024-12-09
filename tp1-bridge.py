import heapq
from itertools import count

class BridgeState:
    """
    Classe pour modéliser un état du problème de traversée du pont.
    """
    def __init__(self, left_side, right_side, flashlight_side, total_time=0):
        """
        Initialise un état de la traversée du pont.
        
        :param left_side: Liste des personnes sur le côté gauche du pont.
        :param right_side: Liste des personnes sur le côté droit du pont.
        :param flashlight_side: Côté où se trouve la lampe ("left" ou "right").
        :param total_time: Temps total écoulé jusqu'à cet état.
        """
        self.left_side = left_side
        self.right_side = right_side
        self.flashlight_side = flashlight_side
        self.total_time = total_time

    def is_goal_state(self):
        """
        Vérifie si l'état actuel est un état final.
        """
        return len(self.left_side) == 0 and self.flashlight_side == "right"

    def move(self, persons):
        """
        Effectue un mouvement pour traverser le pont avec les personnes spécifiées.
        
        :param persons: Liste des personnes qui traversent (1 ou 2 personnes).
        :return: Nouvel état après la traversée.
        """
        if self.flashlight_side == "left":
            new_left = [p for p in self.left_side if p not in persons]
            new_right = self.right_side + persons
            new_flashlight_side = "right"
        else:
            new_left = self.left_side + persons
            new_right = [p for p in self.right_side if p not in persons]
            new_flashlight_side = "left"

        # Le temps ajouté est déterminé par la personne la plus lente
        added_time = max(persons)
        new_total_time = self.total_time + added_time

        return BridgeState(new_left, new_right, new_flashlight_side, new_total_time)

    def successors(self):
        """
        Génère les successeurs de l'état actuel.
        :return: Une liste de tuples (BridgeState, coût)
        """
        successors = []

        if self.flashlight_side == "left":
            # Deux personnes traversent de la gauche vers la droite
            for i in range(len(self.left_side)):
                for j in range(i + 1, len(self.left_side)):
                    new_left = self.left_side[:]
                    new_right = self.right_side[:]
                    person1 = new_left.pop(i)
                    person2 = new_left.pop(j - 1)  # j - 1 car i a été retiré
                    new_right.extend([person1, person2])

                    # Coût est basé sur le plus lent des deux
                    cost = max(person1, person2)
                    new_state = BridgeState(new_left, new_right, "right", self.total_time + cost)
                    successors.append((new_state, cost))

        else:
            # Une personne retourne de la droite vers la gauche
            for i in range(len(self.right_side)):
                new_left = self.left_side[:]
                new_right = self.right_side[:]
                person = new_right.pop(i)
                new_left.append(person)

                # Coût est la vitesse de la personne qui retourne
                cost = person
                new_state = BridgeState(new_left, new_right, "left", self.total_time + cost)
                successors.append((new_state, cost))

        return successors

    def heuristic(self):
        """
        Heuristique admissible pour estimer le coût restant.
        
        :return: Estimation du temps minimum nécessaire pour finir la traversée.
        """
        if self.is_goal_state():
            return 0  # Si l'état est final, aucun coût supplémentaire

        if self.flashlight_side == "left":
            return max(self.left_side)
        else:
            return min(self.right_side) + max(self.left_side)

    def __hash__(self):
        """
        Permet à l'état d'être utilisé dans des ensembles et comme clé de dictionnaire.
        """
        return hash((tuple(self.left_side), tuple(self.right_side), self.flashlight_side))

    def __eq__(self, other):
        """
        Vérifie si deux états sont équivalents.
        """
        return (self.left_side == other.left_side and
                self.right_side == other.right_side and
                self.flashlight_side == other.flashlight_side)

    def __repr__(self):
        """
        Représentation lisible d'un état.
        """
        return (f"Left: {self.left_side}, Right: {self.right_side}, "
                f"Flashlight: {self.flashlight_side}, Total time: {self.total_time} min")

class AStarSolver:
    """
    Implémente l'algorithme A* pour résoudre le problème de traversée du pont.
    L'algorithme trouve le chemin optimal (en termes de coût total) de l'état initial
    à un état final.

    Attributs :
    - initial_state : l'état de départ, une instance de la classe BridgeState.
    - counter : un compteur unique pour gérer les priorités dans la file, en cas d'égalité.
    """

    def __init__(self, initial_state):
        """
        Initialise le solveur A* avec l'état initial.
        
        :param initial_state: Instance de BridgeState représentant l'état initial.
        """
        self.initial_state = initial_state  # L'état de départ.
        self.counter = count()  # Un compteur unique pour briser les égalités dans la file de priorité.

    def solve(self):
        """
        Résout le problème en utilisant l'algorithme A*.

        Fonctionnement :
        - Utilise une file de priorité (`heapq`) pour explorer les états.
        - À chaque étape, l'état avec le plus petit coût estimé (f = g + h) est exploré.
        - Les états visités sont marqués pour éviter les doublons.

        :return: Un tuple (coût_total, chemin) où :
                 - coût_total est le coût total de la solution.
                 - chemin est une liste d'états (BridgeState) menant à la solution.
                 Si aucune solution n'est trouvée, retourne (None, None).
        """
        # File de priorité contenant les états à explorer.
        # Chaque élément est un tuple (f_cost, compteur, g_cost, état, chemin).
        frontier = []
        
        # Ajouter l'état initial dans la file avec :
        # - f_cost = heuristique initiale
        # - g_cost = 0 (aucun coût initial)
        # - chemin = liste vide
        heapq.heappush(frontier, (self.initial_state.heuristic(), next(self.counter), 0, self.initial_state, []))

        # Ensemble des états explorés pour éviter les répétitions.
        explored = set()

        # Boucle principale : tant qu'il y a des états à explorer.
        while frontier:
            # Extraire l'état avec le plus petit coût f(n).
            _, _, g_cost, current_state, path = heapq.heappop(frontier)

            # Vérifier si cet état a déjà été exploré.
            if current_state in explored:
                continue  # Passer à l'itération suivante.

            # Ajouter cet état à l'ensemble des états explorés.
            explored.add(current_state)

            # Vérifier si l'état actuel est un état final.
            if current_state.is_goal_state():
                # Si c'est le cas, retourner le coût total et le chemin.
                return g_cost, path + [current_state]

            # Générer les successeurs de l'état actuel.
            for next_state, cost in current_state.successors():
                # Si le successeur n'a pas été exploré, calculer ses coûts.
                if next_state not in explored:
                    # g_cost : coût réel pour atteindre cet état.
                    new_g_cost = g_cost + cost
                    # f_cost : coût total estimé (g + h).
                    f_cost = new_g_cost + next_state.heuristic()
                    # Ajouter le successeur dans la file avec ses coûts et le chemin mis à jour.
                    heapq.heappush(frontier, (f_cost, next(self.counter), new_g_cost, next_state, path + [current_state]))

        # Si la file est vide et qu'aucune solution n'a été trouvée, retourner None.
        return None, None


# Exemple du cours
# initial_state = BridgeState([1, 2, 5, 10], [], "left")
if __name__ == "__main__":
    initial_state = BridgeState([1, 2, 5, 10], [], "left")
    solver = AStarSolver(initial_state)
    total_cost, solution_path = solver.solve()

    if solution_path:
        print(f"Solution trouvée avec un coût total de {total_cost} minutes.")
        for state in solution_path:
            print(state)
    else:
        print("Aucune solution trouvée.")
