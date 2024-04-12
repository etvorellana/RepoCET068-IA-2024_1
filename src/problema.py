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