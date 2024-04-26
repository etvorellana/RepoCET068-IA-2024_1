"""
function SIMPLE-REFLEX-AGENT(percept) returns an action 
    persistent: rules, a set of condition–action rules
    state←INTERPRET-INPUT(percept) 
    rule←RULE-MATCH(state,rules) 
    action←rule.ACTION
    return action
"""
def simple_reflex_agent(percept):
    rules = {
        ('A'): 'Right',
        ('B'): 'Left',
        ('Sujo'): 'Aspirar',
    }
    if percept[1] == 'sujo':  
        state = percept[1]  
    else:
        state = percept[0]
    action = rules[state]
    return action