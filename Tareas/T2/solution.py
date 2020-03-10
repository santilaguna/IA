#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the homework.

#   You may add only standard python imports---i.e.
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files
#   Runs this file requiere numpy library and python 3.6 or higher

import os #for time functions
import math
from search import * #for search engines
from sliders import *
from problems import *


#SLIDERS HEURISTICS
def sliders_h_zero(state):
    return 0
    
def sliders_h_basic(state):
    sum_in_width = 0
    sum_in_height = 0
    # print(state.tiles.shape)  # filas, columnas
    for width in range(state.tiles.shape[0]):  # state.tiles[width] -> filas
        correlative = np.arange(np.min(state.tiles[width]), np.min(state.tiles[width])+state.tiles.shape[1], 1)
        #print(correlative, state.tiles[width])
        if np.array_equal(state.tiles[width], correlative) is False:
            sum_in_width += 1
    #print()
    for height in range(state.tiles.shape[1]):
        correlative = np.arange(np.min(state.tiles[:,height]), state.tiles.shape[1]+np.min(state.tiles[:,height])+1, state.tiles.shape[1] )
        #print(correlative, state.tiles[:,height])
        if np.array_equal(state.tiles[:,height], correlative) is False :
            sum_in_height += 1
    #print('...')
    #print(state.tiles)
    return min(sum_in_width,sum_in_height)  


def sliders_h_alternate(state):
    """
    a better heuristic
    INPUT: a sliders state
    OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.
    """
    #IMPLEMENT
    # primera idea ->  #casillas en fila incorrecta + #casillas en columna incorrecta/ 3
    # segunda idea min distancias manhattan / min(state.tiles.shape)
    sum_of_manhattan_distances = 0
    for row in range(state.tiles.shape[0]):
        for column in range(state.tiles.shape[1]):

            number = state.tiles[row][column]
            should_be_row = number // state.tiles.shape[1]
            should_be_column = number % state.tiles.shape[1]
            #if should_be_row != row:
            #    sum_of_manhattan_distances += 1
            #if should_be_column != column:
            #    sum_of_manhattan_distances += 1
            if should_be_row > row:
                sum_of_manhattan_distances += min(should_be_row - row, state.tiles.shape[0] - should_be_row + row)
            elif should_be_row < row:
                sum_of_manhattan_distances += min(row - should_be_row, state.tiles.shape[0] - row + should_be_row)

            if should_be_column > column:
                sum_of_manhattan_distances += min(should_be_column - column, state.tiles.shape[1] - should_be_column + column)
            elif should_be_column < column:
                sum_of_manhattan_distances += min(column - should_be_column, state.tiles.shape[1] - column + should_be_column)

    return sum_of_manhattan_distances / max(state.tiles.shape)


def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SlidersState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    #print(sN.gval, sN.hval, sN.gval + weight*sN.hval )
    return sN.gval + weight*sN.hval

def weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
    '''Provides an implementation of weighted a-star'''
    '''INPUT: a sliders state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    wa_se = SearchEngine('custom', 'full')
    wa_se.trace_on(0)
    wa_se.init_search(initial_state, sliders_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
    result = wa_se.search(timebound=timebound)

    if result:
        return result
    else :
        return False


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
    #IMPLEMENT
    #----------------------------------------------
    '''Provides an implementation of anytime weighted a-star (AWA*), as described in the Homework'''
    '''INPUT: a sliders state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''
    wa_se = SearchEngine('custom', 'full')
    wa_se.init_search(initial_state, sliders_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
    costbound_ = None
    current_best = False
    start_time = os.times()[0]
    total_time = 0
    while total_time < timebound:
        result = wa_se.search(timebound=timebound - total_time, costbound=costbound_)
        if result: 
            current_best = result
            costbound_ = (result.gval, math.inf, result.gval + weight * heur_fn(result))
        else:
            break
        total_time = os.times()[0] - start_time
    return current_best

   

def restarting_weighted_astar(initial_state, heur_fn, weight=1., phi=0.8, timebound = 10):
    #IMPLEMENT
    #----------------------------------------------
    '''Provides an implementation of RWA*, as described in the homework'''
    '''INPUT: a sliders state that represents the start state, an heuristic function, an initial weight, a phi parameter and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''
    wa_se = SearchEngine('custom', 'full')
    
    costbound_ = None
    current_best = False
    start_time = os.times()[0]
    total_time = 0
    while total_time < timebound:
        wa_se.init_search(initial_state, sliders_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
        result = wa_se.search(timebound=timebound - total_time, costbound=costbound_)
        if result:
            weight = max(1, weight * phi)
            current_best = result
            costbound_ = (result.gval, math.inf, result.gval + weight * heur_fn(result))
        else:
            break
        if weight == 1:
            break
        total_time = os.times()[0] - start_time

    return current_best



if __name__ == "__main__":

    #sample runs 
    #se = SearchEngine('astar', 'full')

    #If you want to trace the search, set trace_on.  Using Level 1 for illustration. Level 2 prints more detailed results.    
    #se.trace_on(0)
    #se.trace_on(1)
    #se.trace_on(2)

    s0 = PROBLEMS[1]
    s1 = PROBLEMS[5]
    s2 = PROBLEMS[8]
    s3 = PROBLEMS[10]

    #print("=========Demo 1. Astar with h_zero heuristic========")
    #se.init_search(s0, sliders_goal_state, sliders_h_zero)
    #final = se.search(timebound=20)
    #if final: final.print_path()
    #print("===================================================")

    #print("=========Demo 2. Astar with h_basic heuristic========")
    #se.init_search(s0, sliders_goal_state, sliders_h_basic)
    #final = se.search(timebound=20)
    #if final: final.print_path()
    #print("===================================================")

    #print("=========Demo 3. Weighted Astar with h_basic heuristic========")
    #weight = 10
    #final = weighted_astar(s0, heur_fn=sliders_h_basic, weight=weight, timebound=20)
    #if final: final.print_path()
    #print("===================================================")

    #print("=========Demo 4. Weighted Astar with h_alternate heuristic========")
    #weight = 5
    #final = weighted_astar(s0, heur_fn=sliders_h_alternate, weight=weight, timebound=20)
    #if final: final.print_path()
    #print("===================================================")
 
    print("=========Demo 2x2. Anytime Weighted Astar with h_alternate heuristic========")
    start_time_2 = os.times()[0]
    weight = 5
    final = anytime_weighted_astar(s0, heur_fn=sliders_h_alternate, weight=weight, timebound=0)
    if final: final.print_path()
    print(start_time_2 - os.times()[0])
    print("===================================================")

    print("=========Demo 2x2. Restarting Weighted Astar with h_alternate heuristic========")
    start_time_2 = os.times()[0]
    weight = 5
    final = restarting_weighted_astar(s0, heur_fn=sliders_h_alternate, weight=weight, timebound=20)
    if final: final.print_path()
    print(start_time_2 - os.times()[0])
    print("===================================================")

    print("=========Demo 3x3. Anytime Weighted Astar with h_alternate heuristic========")
    start_time_2 = os.times()[0]
    weight = 5
    final = anytime_weighted_astar(s1, heur_fn=sliders_h_alternate, weight=weight, timebound=20)
    if final: final.print_path()
    print(start_time_2 - os.times()[0])
    print("===================================================")

    print("=========Demo 3x3. Restarting Weighted Astar with h_alternate heuristic========")
    start_time_2 = os.times()[0]
    weight = 5
    final = restarting_weighted_astar(s1, heur_fn=sliders_h_alternate, weight=weight, timebound=20)
    if final: final.print_path()
    print(start_time_2 - os.times()[0])
    print("===================================================")

    print("=========Demo 3x4. Anytime Weighted Astar with h_alternate heuristic========")
    start_time_2 = os.times()[0]
    weight = 5
    final = anytime_weighted_astar(s2, heur_fn=sliders_h_alternate, weight=weight, timebound=20)
    if final: final.print_path()
    print(start_time_2 - os.times()[0])
    print("===================================================")

    print("=========Demo 3x4. Restarting Weighted Astar with h_alternate heuristic========")
    start_time_2 = os.times()[0]
    weight = 5
    final = restarting_weighted_astar(s2, heur_fn=sliders_h_alternate, weight=weight, timebound=20)
    if final: final.print_path()
    print(start_time_2 - os.times()[0])
    print("===================================================")

    print("=========Demo 5x5. Anytime Weighted Astar with h_alternate heuristic========")
    start_time_2 = os.times()[0]
    weight = 5
    final = anytime_weighted_astar(s3, heur_fn=sliders_h_alternate, weight=weight, timebound=20)
    if final: final.print_path()
    print(start_time_2 - os.times()[0])
    print("===================================================")

    print("=========Demo 5x5. Restarting Weighted Astar with h_alternate heuristic========")
    start_time_2 = os.times()[0]
    weight = 5
    final = restarting_weighted_astar(s3, heur_fn=sliders_h_alternate, weight=weight, timebound=20)
    if final: final.print_path()
    print(start_time_2 - os.times()[0])
    print("===================================================")
