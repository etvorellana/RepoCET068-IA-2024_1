"""
function REFLEX-VACUUM-AGENT([location,status]) returns an action
    if status = Dirty then return Suck 
    else if location = A then return Right 
    else if location = B then return Left
"""

def reflex_vacuum_agent(percept):
    location, status = percept
    if status == 'Sujo':
        return 'Aspirar'
    elif location == 'A':
        return 'Right'
    elif location == 'B':
        return 'Left'