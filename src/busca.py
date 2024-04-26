'''
function BEST-FIRST-SEARCH(problem, f ) returns a solution node or failure 
    node←NODE(STATE=problem.INITIAL)
    frontier ← a priority queue ordered by f , with node as an element
    reached←a lookup table, with one entry with key problem.INITIAL and value node 
    while not IS-EMPTY(frontier) do
        node←POP(frontier)
        if problem.IS-GOAL(node.STATE) then return node 
        for each child in EXPAND(problem, node) do
            s←child.STATE
            if s is not in reached or child.PATH-COST < reached[s].PATH-COST then
                reached[s] ← child
                add child to frontier 
    return failure

function EXPAND(problem,node) yields nodes 
    s←node.STATE
    for each action in problem.ACTIONS(s) do
        s′ ←problem.RESULT(s,action)
        cost←node.PATH-COST + problem.ACTION-COST(s,action,s′)
        yield NODE(STATE=s′, PARENT=node, ACTION=action, PATH-COST=cost)
'''

class Node:
    
    def __init__(self, state, parent = None, action = None, path_cost = 0):
        self.state = state          # o estado ao qual o nó corresponde; (str)
        self.parent = parent        # o nó da árvore que gerou este nó; (Node)
        self.action = action        # a ação executada para gerar este nó; (str)
        self.path_cost = path_cost  # o custo do caminho do nó inicial até este nó. (int)

class Frontier:
    def __init__(self):
        self.elements = []          # lista de nós
    def is_empty(self):
        # IS-EMPTY(frontier) returns true only if there are no nodes in the frontier.
        return len(self.elements) == 0
    def pop(self):
        # POP(frontier) removes the top node from the frontier and returns it.
        return self.elements.pop(0)
    def top(self):
        # TOP(frontier) returns (but does not remove) the top node of the frontier.
        return self.elements[0]
    def add(self, node):
        #ADD(node, frontier) inserts node into its proper place in the queue.
        pass

class Problem:

    def __init__(self, states, initial, goal, actions, transition_model, cost):
        self.states = states                #estados possíveis
        if initial not in states:           #verifica se o estado inicial é um estado possível
            self.states.append(initial)     #caso não seja, adiciona o estado inicial aos estados possíveis
        self.initial = initial              #estado inicial do problema
        if goal not in states:              #verifica se o estado objetivo é um estado possível
            self.states.append(goal)        #caso não seja, adiciona o estado objetivo aos estados possíveis
        self.goal = goal                    #estado(s) objetivo do problema
        self.actions = actions              #ações possíveis
        self.transition_model = transition_model
        self.cost = cost
    def get_actions(self, state):
        return self.actions[state]
    def result(self, state, action):
        return self.transition_model[state][action]
    def goal_test(self, state):
        return state == self.goal
    def action_cost(self, state1, action, state2):
        if action in self.actions[state1] and state2 == self.result(state1, action):
            return self.cost[state1][state2]
        else:
            return -1

class PriorityQueue(Frontier):
        
    def __init__(self, f):
        super().__init__()            # lista de nós
        self.f = f                  # função de avaliação f
    
    def add(self, node):
        # ADD(node, frontier) inserts node into its proper place in the queue.
        self.elements.append(node)
        self.elements = sorted(self.elements, key = self.f)

class FIFOQueue(Frontier):
        
    def add(self, node):
        '''
        ADD(node, frontier) inserts node at the end of the queue.
        '''
        self.elements.append(node)

class BestFirstSearch:
    def __init__(self, problem, f):
        self.problem = problem
        self.f = f
    def search(self):
        node = Node(self.problem.initial) # node←NODE(STATE=problem.INITIAL)
        frontier = PriorityQueue(self.f) # frontier ← a priority queue ordered by f , with node as an element
        frontier.add(node) # add node to frontier
        reached = {self.problem.initial: node} # reached←a lookup table, with one entry with key problem.INITIAL and value node
        while not frontier.is_empty(): # while not IS-EMPTY(frontier) do
            node = frontier.pop() # node←POP(frontier)
            if self.problem.goal_test(node.state):
                return node
            else:
                print(node.state, node.path_cost)
            for child in self.expand(node):
                s = child.state
                if s not in reached or child.path_cost < reached[s].path_cost:
                    reached[s] = child
                    frontier.add(child)
        return None
    
    def expand(self, node):
        s = node.state
        for action in self.problem.get_actions(s):
            s_prime = self.problem.result(s, action)
            cost = node.path_cost + self.problem.action_cost(s, action, s_prime)
            yield Node(s_prime, node, action, cost)
    def path(self, node):
        path_back = []
        while node:
            path_back.append(node)
            node = node.parent
        return path_back[::-1]
    

'''
function BREADTH-FIRST-SEARCH(problem) returns a solution node or failure 
    node←NODE(problem.INITIAL)
    if problem.IS-GOAL(node.STATE) then return node
    frontier ← a FIFO queue, with node as an element 
    reached←{problem.INITIAL}
    while not IS-EMPTY(frontier) do 
        node←POP(frontier)
        for each child in EXPAND(problem, node) do
            s←child.STATE
            if problem.IS-GOAL(s) then return child 
            if s is not in reached then
                add s to reached
                add child to frontier 
    return failure

function UNIFORM-COST-SEARCH(problem) returns a solution node, or failure 
    return BEST-FIRST-SEARCH(problem, PATH-COST)
'''

class BreadthFirstSearch:
    def __init__(self, problem):
        self.problem = problem
    def search(self):
        node = Node(self.problem.initial)
        if self.problem.goal_test(node.state):
            return node
        else:
            print(node.state, node.path_cost)
        frontier = FIFOQueue()
        frontier.add(node)
        reached = [self.problem.initial]  # reached←{problem.INITIAL} (como uma lista)
        while not frontier.is_empty():
            node = frontier.pop()
            for child in self.expand(node):
                s = child.state
                if self.problem.goal_test(s):
                    return child
                else:
                    print(child.state, child.path_cost)
                if s not in reached:
                    reached.append(s)
                    frontier.add(child)
        return None
    
    def expand(self, node):
        s = node.state
        for action in self.problem.get_actions(s):
            s_prime = self.problem.result(s, action)
            cost = node.path_cost + self.problem.action_cost(s, action, s_prime)
            yield Node(s_prime, node, action, cost)
    def path(self, node):
        path_back = []
        while node:
            path_back.append(node)
            node = node.parent
        return path_back[::-1]

def main():
    states = ['Arad', 'Zerind', 'Oradea', 'Sibiu', 'Timisoara', 
          'Lugoj', 'Mehadia', 'Drobeta', 'Craiova', 'Rimnicu Vilcea', 
          'Fagaras', 'Pitesti', 'Bucharest', 'Giurgiu', 'Urziceni', 
          'Hirsova', 'Eforie', 'Vaslui', 'Iasi', 'Neamt']
    initial = 'Arad'
    goal = 'Bucharest'
    actions = {'Arad': ['toZerind', 'toSibiu', 'toTimisoara'],
            'Zerind': ['toArad', 'toOradea'],
            'Oradea': ['toZerind', 'toSibiu'],
            'Sibiu': ['toArad', 'toOradea', 'toFagaras', 'toRimnicu Vilcea'],
            'Timisoara': ['toArad', 'toLugoj'],
            'Lugoj': ['toTimisoara', 'toMehadia'],
            'Mehadia': ['toLugoj', 'toDrobeta'],
            'Drobeta': ['toMehadia', 'toCraiova'],
            'Craiova': ['toDrobeta', 'toRimnicu Vilcea', 'toPitesti'],
            'Rimnicu Vilcea': ['toSibiu', 'toCraiova', 'toPitesti'],
            'Fagaras': ['toSibiu', 'toBucharest'],
            'Pitesti': ['toRimnicu Vilcea', 'toCraiova', 'toBucharest'],
            'Bucharest': ['toFagaras', 'toPitesti', 'toGiurgiu', 'toUrziceni'],
            'Giurgiu': ['toBucharest'],
            'Urziceni': ['toBucharest', 'toHirsova', 'toVaslui'],
            'Hirsova': ['toUrziceni', 'toEforie'],
            'Eforie': ['toHirsova'],
            'Vaslui': ['toUrziceni', 'toIasi'],
            'Iasi': ['toVaslui', 'toNeamt'],
            'Neamt': ['toIasi']}
    transition_model = {
        'Arad': {'toZerind': 'Zerind', 'toSibiu': 'Sibiu', 'toTimisoara': 'Timisoara'},
        'Zerind': {'toArad': 'Arad', 'toOradea': 'Oradea'},
        'Oradea': {'toZerind': 'Zerind', 'toSibiu': 'Sibiu'},
        'Sibiu': {'toArad': 'Arad', 'toOradea': 'Oradea', 'toFagaras': 'Fagaras', 'toRimnicu Vilcea': 'Rimnicu Vilcea'},
        'Timisoara': {'toArad': 'Arad', 'toLugoj': 'Lugoj'},
        'Lugoj': {'toTimisoara': 'Timisoara', 'toMehadia': 'Mehadia'},
        'Mehadia': {'toLugoj': 'Lugoj', 'toDrobeta': 'Drobeta'},
        'Drobeta': {'toMehadia': 'Mehadia', 'toCraiova': 'Craiova'},
        'Craiova': {'toDrobeta': 'Drobeta', 'toRimnicu Vilcea': 'Rimnicu Vilcea', 'toPitesti': 'Pitesti'},
        'Rimnicu Vilcea': {'toSibiu': 'Sibiu', 'toCraiova': 'Craiova', 'toPitesti': 'Pitesti'},
        'Fagaras': {'toSibiu': 'Sibiu', 'toBucharest': 'Bucharest'},
        'Pitesti': {'toRimnicu Vilcea': 'Rimnicu Vilcea', 'toCraiova': 'Craiova', 'toBucharest': 'Bucharest'},
        'Bucharest': {'toFagaras': 'Fagaras', 'toPitesti': 'Pitesti', 'toGiurgiu': 'Giurgiu', 'toUrziceni': 'Urziceni'},
        'Giurgiu': {'toBucharest': 'Bucharest'},
        'Urziceni':{'toBucharest': 'Bucharest', 'toHirsova': 'Hirsova', 'toVaslui': 'Vaslui'},
        'Hirsova': {'toUrziceni': 'Urziceni', 'toEforie': 'Eforie'},
        'Eforie': {'toHirsova': 'Hirsova'},
        'Vaslui': {'toUrziceni': 'Urziceni', 'toIasi': 'Iasi'},
        'Iasi': {'toVaslui': 'Vaslui', 'toNeamt': 'Neamt'},
        'Neamt': {'toIasi': 'Iasi'}}
    cost = {'Arad': {'Zerind': 75, 'Sibiu': 140, 'Timisoara': 118},
            'Zerind': {'Arad': 75, 'Oradea': 71},
            'Oradea': {'Zerind': 71, 'Sibiu': 151},
            'Sibiu': {'Arad': 140, 'Oradea': 151, 'Fagaras': 99, 'Rimnicu Vilcea': 80},
            'Timisoara': {'Arad': 118, 'Lugoj': 111},
            'Lugoj': {'Timisoara': 111, 'Mehadia': 70},
            'Mehadia': {'Lugoj': 70, 'Drobeta': 75},
            'Drobeta': {'Mehadia': 75, 'Craiova': 120},
            'Craiova': {'Drobeta': 120, 'Rimnicu Vilcea': 146, 'Pitesti': 138},
            'Rimnicu Vilcea': {'Sibiu': 80, 'Craiova': 146, 'Pitesti': 97},
            'Fagaras': {'Sibiu': 99, 'Bucharest': 211},
            'Pitesti': {'Rimnicu Vilcea': 97, 'Craiova': 138, 'Bucharest': 101},
            'Bucharest': {'Fagaras': 211, 'Pitesti': 101, 'Giurgiu': 90, 'Urziceni': 85},
            'Giurgiu': {'Bucharest': 90},
            'Urziceni': {'Bucharest': 85, 'Hirsova': 98, 'Vaslui': 142},
            'Hirsova': {'Urziceni': 98, 'Eforie': 86},
            'Eforie': {'Hirsova': 86},
            'Vaslui': {'Urziceni': 142, 'Iasi': 92},
            'Iasi': {'Vaslui': 92, 'Neamt': 87},
            'Neamt': {'Iasi': 87}}
    
    Arad2Bucarest = Problem(states, initial, goal, actions, transition_model, cost)
    busca = BestFirstSearch(Arad2Bucarest, lambda node: node.path_cost)
    print("Best First Search (Arad -> Bucharest):")
    node = busca.search()
    print(node.state, node.path_cost)
    print("Solution:")
    for step in busca.path(node):
        print(step.state, step.path_cost)
    

    print("__________________________")
    print("Breadth First Search (Arad -> Bucharest):")
    busca = BreadthFirstSearch(Arad2Bucarest)
    node = busca.search()
    print(node.state, node.path_cost)
    print("Solution:")
    for step in busca.path(node):
        print(step.state, step.path_cost)

if __name__ == '__main__':
    main()