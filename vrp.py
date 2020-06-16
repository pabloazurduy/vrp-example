import mip
import numpy as np
import itertools

cost_matrix  = np.random.rand(10,10) # matriz tiempo o distancia
cost_matrix =  cost_matrix + cost_matrix.T  - np.diag(cost_matrix.diagonal())

nodes = list(range(cost_matrix.shape[0]))
trucks = list(range(2))


model = mip.Model(name =  'vrp')
x = {}
for i,j,k in itertools.product(nodes,nodes,trucks):
    # declarate path variables 
    x[(i,j,k)] = model.add_var(var_type = mip.BINARY , 
                               name = f'path_{i}_{j}_{k}')

y = {}    
for i,k in itertools.product(nodes,trucks):
    # declarate path variables 
    y[(i,k)] = model.add_var(var_type = mip.BINARY , 
                             name = f'asignation_{i}_{k}')

model.objective = mip.xsum([x[(i,j,k)]*cost_matrix[i][j] for i,j,k in itertools.product(nodes,nodes,trucks) ])
model.sens = mip.MINIMIZE
model.optimize()

for key,var in x.items():
    print(key, var.x)