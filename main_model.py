import numpy as np
import gurobipy as gp
from gurobipy import GRB

class two_stage_model:
    def __init__(self, model_params):
        self.max_mit = 25
        self.coarse = model_params["mit_coarse"]
        self.input1 = model_params["input1"]
        self.input2 = model_params["input2"]
        self.budget = model_params["budget"]
        self.fc = model_params["fixed_cost"]
        self.vc = model_params["variable_cost"]
        self.flexible_generation = model_params["flexible_generation"]
        self.robust_model = model_params["robust_flag"]
        self.reference_bus = model_params["reference_bus"]
        self.filter_col = [col for col in self.input1 if col.startswith('max')]
        self.n_buses = self.input1.shape[0]
        self.n_branches = self.input2.shape[0]
        self.n_scenarios = len(self.filter_col)        
        self.unique_substations = self.input1["SubNum"].unique()
        self.optimization_type = model_params["set_objective"]
        self.substation_and_bus_info()
        self.node_matrix()
        self.create_model()
        self.budget_constraint()
        self.box_constraints()
        self.linking_and_capacity_constraints()
        self.edge_constraints()
        self.flow_balance_constraints()
        self.phase_angle()
        self.robust_constraints()
        self.set_objective()

    def create_model(self):    
        self.model = gp.Model()
        # Cost Variables
        self.y = self.model.addVars(self.unique_substations, vtype=GRB.BINARY, name = "y")
        self.x = self.model.addVars(self.unique_substations, lb=0, ub=GRB.INFINITY, vtype=GRB.INTEGER, name = "x")
        # Buses Variables
        self.z = self.model.addVars(self.n_buses, self.n_scenarios, vtype=GRB.BINARY, name = "z")
        self.g = self.model.addVars(self.n_buses, self.n_scenarios, lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name = "g")
        self.s = self.model.addVars(self.n_buses, self.n_scenarios, lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name = "s")
        self.theta = self.model.addVars(self.n_buses, self.n_scenarios, lb=-3.14, ub=3.14, vtype=GRB.CONTINUOUS, name = "theta")
        # Branch Variables
        self.edge = self.model.addVars(self.n_branches, self.n_scenarios, lb=-GRB.INFINITY, ub=GRB.INFINITY, 
                                        vtype=GRB.CONTINUOUS, name = "edge")
        # Other variants of the model (with flexible generation or robust model)
        if self.flexible_generation:
            self.alpha = self.model.addVars(self.n_buses, self.n_scenarios, vtype=GRB.BINARY, name = "alpha")
        if self.robust_model:
            self.tau_scenario = self.model.addVars(self.n_scenarios, lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, 
                                                    name = "tau_scenario")
            self.tau = self.model.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name = "tau")

    def box_constraints(self):
        #self.model.addConstrs((self.x[i] <= self.y[i]*self.substation_info[i] for i in self.substation_info))
        self.model.addConstrs((self.coarse*self.x[i] <= self.y[i]*self.max_mit for i in self.substation_info))
        
    def budget_constraint(self):
        self.budget_ref = self.model.addConstr(self.fc*self.y.sum() + self.coarse*self.vc*self.x.sum() <= self.budget, 
                                                                                        name="budget_constraint")    

    def linking_and_capacity_constraints(self):
        flood_frame = self.input1[self.filter_col].values
        input1 = self.input1.values
        # Supply should be less than the maximum demand
        self.supply_demand_constraint_reference = self.model.addConstrs((self.s[i,j] <= self.z[i,j]*input1[i,8] 
                                                                        for i in range(self.n_buses) 
                                                                        for j in range(self.n_scenarios)), 
                                                                        name="supply_constraint")
        # Capacity Constraints
        if self.flexible_generation:
            self.generation_lower_bound_reference = self.model.addConstrs((self.alpha[i,j]*input1[i,6] <= self.g[i,j] 
                                                                        for i in range(self.n_buses) 
                                                                        for j in range(self.n_scenarios)), 
                                                                        name = "generation_lower_bound")

            self.model.addConstrs(self.alpha[i,j]*input1[i,7] >= self.g[i,j] 
                                                                        for i in range(self.n_buses) 
                                                                        for j in range(self.n_scenarios))

            self.model.addConstrs(self.alpha[i,j] <= self.z[i,j] for i in range(self.n_buses) 
                                                                        for j in range(self.n_scenarios))
        else:
            self.generation_lower_bound_reference = self.model.addConstrs((self.z[i,j]*input1[i,6] <= self.g[i,j] 
                                                                        for i in range(self.n_buses) 
                                                                        for j in range(self.n_scenarios)), 
                                                                        name = "generation_lower_bound")

            self.model.addConstrs(self.z[i,j]*input1[i,7] >= self.g[i,j] for i in range(self.n_buses) 
                                                                        for j in range(self.n_scenarios))

        # Linking Constraints
        for i in range(self.n_buses):
            for j in range(self.n_scenarios):
                substation_id = input1[i,2]
                if (self.substation_info[substation_id]> 0):
                    self.model.addConstr(1 - self.z[i,j] >= (flood_frame[i, j] - self.coarse*self.x[substation_id])/35)
                    self.model.addConstr(self.z[i,j] >= (-flood_frame[i, j] + self.coarse*self.x[substation_id] + 0.5)/35)
                else:
                    self.model.addConstr(self.z[i,j] == 1)

    def edge_constraints(self):
        temp_index = np.array(self.input1.index)
        input1 = self.input1.values
        input2 = self.input2.values
        for j in range(self.n_scenarios):
            for i in range(self.n_branches):
                head = input1[np.where(temp_index == input2[i,1])[0][0], 0]
                tail = input1[np.where(temp_index == input2[i,2])[0][0], 0]
                b_r = input2[i,3]
                c_max = input2[i,4]
                m = c_max + 6.28*b_r
                #m = 50000
                self.model.addConstr(-self.z[head, j]*c_max <= self.edge[i,j])
                self.model.addConstr(self.edge[i,j] <=  self.z[head, j]*c_max)
                self.model.addConstr(-self.z[tail, j]*c_max <= self.edge[i,j])
                self.model.addConstr(self.edge[i,j] <= self.z[tail, j]*c_max)

                self.model.addConstr(m*(self.z[head, j] + self.z[tail, j]) -2*m + self.edge[i,j] 
                                                            <= b_r*(self.theta[head,j] - self.theta[tail,j]))
                self.model.addConstr(-m*(self.z[head, j] + self.z[tail, j]) + 2*m + self.edge[i,j] 
                                                            >= b_r*(self.theta[head,j] - self.theta[tail,j]))

    def flow_balance_constraints(self):
        for scenario in range(self.n_scenarios):
            for i in range(self.n_buses):
                temp = 0
                for j in self.node_edge_dictionary[i]:
                    temp =  temp + self.node_arc_incidence_matrix[i,j]*self.edge[j, scenario]
                self.model.addConstr(temp == self.g[i,scenario] - self.s[i,scenario])

    def phase_angle(self):
        self.model.addConstrs((self.theta[self.reference_bus,k] == 0 for k in range(self.n_scenarios)))

    def robust_constraints(self):
        if self.robust_model:
            max_load = self.input1.iloc[:,8].sum()
            self.model.addConstrs((max_load - self.s.sum('*', j) == self.tau_scenario[j] for j in range(self.n_scenarios)))
            self.model.addConstrs((self.tau >= self.tau_scenario[j] for j in range(self.n_scenarios)))
    
    def set_objective(self):
        if self.robust_model:
            self.model.setObjective(self.tau, GRB.MINIMIZE)
        else:
            if self.optimization_type == 'min':
                self.model.setObjective(self.input1["load"].sum() - (self.s.sum('*','*')/self.n_scenarios), GRB.MINIMIZE)
            else:
                self.model.setObjective(self.s.sum('*','*')/self.n_scenarios, GRB.MAXIMIZE)
                

    def substation_and_bus_info(self):
        # substation_info: key --> substation_id, value --> max flooding at the substation
        # bus_info: key --> bus_id, value --> corresponding substation
        self.bus_info = {}
        self.substation_info = {}
        for i in self.unique_substations:
            self.substation_info[i] = self.input1[self.input1["SubNum"] == i][self.filter_col].max(axis=1).iloc[0]
        for i in range(len(self.input1["bus_index"].unique())):
            self.bus_info[i] = self.input1[self.input1["bus_index"] == i].iloc[0,2] 

    def node_matrix(self):
        # Creating of node arc incidence matrix
        self.node_arc_incidence_matrix = np.zeros((len(self.input1), len(self.input2)))
        for i in range(len(self.input2)): 
            head = self.input1.loc[self.input2.iloc[i,1], "bus_index"]
            tail = self.input1.loc[self.input2.iloc[i,2], "bus_index"] 
            self.node_arc_incidence_matrix[head,i] = 1           # Indicates outgoing flux
            self.node_arc_incidence_matrix[tail,i] = -1          # Indicates incoming flux
        self.node_edge_dictionary = {}
        for i in range(len(self.input2)):
            head = self.input1.loc[self.input2.iloc[i,1], "bus_index"]
            tail = self.input1.loc[self.input2.iloc[i,2], "bus_index"] 
            if head in self.node_edge_dictionary.keys():
                self.node_edge_dictionary[head].append(i)
            else:
                self.node_edge_dictionary[head] = []
                self.node_edge_dictionary[head].append(i)
            if tail in self.node_edge_dictionary.keys():
                self.node_edge_dictionary[tail].append(i)
            else:
                self.node_edge_dictionary[tail] = []
                self.node_edge_dictionary[tail].append(i)