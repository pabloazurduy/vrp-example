import mip
import itertools
import random
from scipy.spatial import distance_matrix
from copy import deepcopy



def create_instance(n_nodes = 25, n_trucks=2, starting_nodes='default'):
    # set seed 
    random.seed(0)             
    nodes = list(range(n_nodes))  
    nodes_pos = {}
    nodes_pos_list  = []
    for node_id in nodes:
        x = random.uniform(0, 150)
        y = random.uniform(0, 150)
        nodes_pos[node_id] = (x,y) 
        nodes_pos_list.append([x,y])
        
    cost_matrix = distance_matrix(nodes_pos_list, nodes_pos_list)
    #cost_matrix  = np.random.rand(45,45) # matriz tiempo o distancia
    #cost_matrix =  cost_matrix + cost_matrix.T  - 2*np.diag(cost_matrix.diagonal())
    
    nodes = list(range(cost_matrix.shape[0]))
    trucks = list(range(n_trucks))
    if starting_nodes == 'default':
        origin = {}
        for k in trucks:
            origin[k]=k
    elif type(starting_nodes) == dict:
        origin = deepcopy(starting_nodes)
        
    
    return nodes, trucks, cost_matrix, nodes_pos, origin

nodes, trucks, cost_matrix, nodes_pos, origin = create_instance(starting_nodes={0:0, 1:1, 2:2} )        
# model
model = mip.Model(name =  'vrp')

# ========================== #
# ==== var declaration ===== #
# ========================== #

x = {}
for i,j,k in itertools.product(nodes,nodes,trucks):
    # declarate path variables 
    if i != j : 
        x[(i,j,k)] = model.add_var(var_type = mip.BINARY , 
                                   name = f'path_{i}_{j}_{k}')

y = {}    
for i,k in itertools.product(nodes,trucks):
    # declarate path variables 
    y[(i,k)] = model.add_var(var_type = mip.CONTINUOUS , 
                             name = f'asignation_{i}_{k}')

u = {}
for i,k in itertools.product(nodes,trucks):
    u[(i,k)] = model.add_var(var_type = mip.INTEGER , 
                             name = f'order_{i}_{k}')

z = {} # last node 
for i,k in itertools.product(nodes,trucks):
    # declarate path variables 
    if i != origin[k]:
        z[(i,k)] = model.add_var(var_type = mip.BINARY , 
                                 name = f'end_node_{i}_{k}')


# ======================== #
# ===== constraints ====== #
# ======================== #

# 0. end node codification  
for k in trucks:
    model.add_sos([(z[i,k],1) for i in nodes if i!= origin[k]], sos_type=1) 

# 1. flow conservation
for i,k in itertools.product(nodes,trucks):
    if i != origin[k]:
        model.add_constr(mip.xsum([x[(j,i,k)] for j in nodes if j!=i ]) == # lo que entra
                         mip.xsum([x[(i,j,k)] for j in nodes if j!=i ]) - z[i,k] , # lo que sale
                         name = f'flow_conservation_{i}_{k}' ) 

# 2. y codification 
for i,k in itertools.product(nodes,trucks):
    model.add_constr(y[(i,k)] == mip.xsum([x[(j,i,k)] for j in nodes if j!=i]) , name=f'y[{i}{k}]_cod') 

# 3. demand fulfillment
for i in nodes:
    model.add_constr(mip.xsum([ y[(i,k)] for k in trucks]) == 1 , name=f'y[{i}{k}]_cod') 

 
# 4. subtour elimination 
graph_len = len(nodes)
for k in trucks:
    for i,j in itertools.product(nodes,nodes):
        if i != j and (i != origin[k] and j!= origin[k]): # remove origin 
            model.add_constr(u[(i,k)] - u[(j,k)] + 1  <= graph_len*(1- x[(i,j,k)]) , name=f'subtour_constraint_{i}_{j}_{k}')
    
    model.add_constr(u[(origin[k],k)] == 1 , name=f'subtour_constraint_origin_{k}')
    
    for i in nodes:
        if i != origin[k] :
            model.add_constr(u[(i,k)] >=2  , name=f'subtour_constraint_lowerbound_{i}')
            model.add_constr(u[(i,k)] <= graph_len , name=f'subtour_constraint_upperbound_{i}')
            

# ============================ #
# ==== model declaration ===== #
# ============================ #

# objective function
model.objective = mip.xsum([x[key]*cost_matrix[key[0]][key[1]] for key in x.keys()])
model.sens = mip.MINIMIZE

# model tunning
# cuts
# -1  automatic, 0 disables completely, 
# 1 (default) generates cutting planes in a moderate way,
# 2 generates cutting planes aggressively  
# 3 generates even more cutting planes
model.cuts = 2 
model.max_mip_gap = 0.005 # 5%
model.max_seconds = 15*60 
model.optimize()

"""
for key,var in x.items():
    print('x',key, var.x)

for key,var in y.items():
    print('y',key, var.x)
"""
for k in trucks:
    print('total nodes served by {0} = {1}'.format(k, sum([y[(i,k)].x for i in nodes ])))

for k in trucks:
    print('node_list = {}'.format([ i for i in nodes if y[(i,k)].x == 1]))

# ====================== #
# ==== viz results ===== #
# ====================== #

import networkx as nx
G = nx.Graph()

G.add_nodes_from(nodes)
G.add_edges_from([(key[0], key[1]) for key in x.keys() if x[key].x == 1]) #{'weight': 3.1415}
#node_positions = nx.spring_layout(G, k=0.15, scale=100)
for node_id,pos in nodes_pos.items():
    G.nodes[node_id]['pos'] = (pos[0],pos[1])

nx.draw(G, pos=nodes_pos, with_labels = True)


