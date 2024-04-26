"""
function TABLE-DRIVEN-AGENT(percept) returns an action 
    persistent: percepts, a sequence, initially empty
                table, a table of actions, indexed by percept sequences, initially fully specified
    append percept to the end of percepts 
    action←LOOKUP(percepts,table) 
    return action
"""
percepts = []
table = {
        ('A', 'Limpo'): 'Right',
        ('B', 'Limpo'): 'Left',
        ('A', 'Sujo'): 'Aspirar',
        ('B', 'Sujo'): 'Aspirar'
    }
def table_driven_agent(percept):
    global percepts
    global table
    def lookup(percepts, table):
        if percepts[-1] in table:
            return table[percepts[-1]]
        else:
            return 'None'
    percepts.append(percept)
    action = lookup(percepts, table)
    return action

def main():
    # Testando o agente
    # Ambiente: A e B sujos
    ambiente = [('A', 'Sujo'), ('B', 'Sujo')]
    pos = 'A' # posição do aspirador
    # Sensor do agente 
    def sensor(ambiente):
        nonlocal pos
        if pos == 'A':
            return ambiente[0]
        else:
            return ambiente[1]

    def atuador(ambiente, action):
        nonlocal pos
        if action == 'Aspirar':
            if pos == 'A':
                ambiente[0] = ('A', 'Limpo')
            else:
                ambiente[1] = ('B', 'Limpo')
        elif action == 'Right':
            pos = 'B'
        elif action == 'Left':
            pos = 'A'
        return ambiente

    for step in range(5):
        print('Step: ', step, end=' ')
        percept = sensor(ambiente)
        action = table_driven_agent(percept)
        print('Percept: ', percept)
        print('Action: ', action)
        ambiente = atuador(ambiente, action)
        print('Ambiente: ', ambiente)
        print('---')

if __name__ == '__main__':
    main()