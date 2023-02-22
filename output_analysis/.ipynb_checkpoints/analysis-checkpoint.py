import sys
import json
import pandas as pd

sys.path.append("/Users/ashutoshshukla/Desktop/TwoStageModel/")
from utils import prepare_input
from main_model import two_stage_model

class analysis:
    def __init__(self, n_scenario, model_type ="sm_", mit_coarse = 1, budget_vector = [0,10,20,30,40,50,60, 70, 80], 
                 path="/Users/ashutoshshukla/Desktop/TwoStageModel/output/"):
        with open(path + model_type + str(n_scenario) + "/" + "model_params.json") as file:
            self.model_params = json.load(file)
        
        self.model_params["mit_coarse"] = mit_coarse
        self.model_params["input1"], self.model_params["input2"] = prepare_input(self.model_params["path_to_input"])
        
        self.n_flooded_substations = self.model_params["input1"][self.model_params["input1"].iloc[:,9:].sum(axis=1) > 0].shape[0]
        self.base_model = two_stage_model(self.model_params)        
        self.base_model.model.update()
        self.budget_vector = budget_vector
        
        hardening_decisions = {}
        for budget in self.budget_vector:
            hardening = []
            sol_path = path + model_type + str(n_scenario)+ "/"+str(budget)+"M_solution.sol"
            self.base_model.model.read(sol_path)
            self.base_model.model.update()
            for sub_id in self.base_model.unique_substations:
                temp = self.base_model.model.getVarByName('x[' +  str(sub_id) + ']').Start
                hardening.append(temp)
            hardening_decisions[budget] = hardening

        df = pd.DataFrame(hardening_decisions)
        
        self.main_df = df.copy()
        self.main_df.index = self.base_model.unique_substations
        self.main_df = self.main_df.loc[~(self.main_df==0).all(axis=1)].round(0)
        self.hardening_df = df.loc[~(df==0).all(axis=1)].round(0)
        
    def substation_hardened_with_budget(self):
        for i in range(self.budget_vector[0], 
                        self.budget_vector[-1] + int(self.budget_vector[1] - self.budget_vector[0]), 
                        int(self.budget_vector[1] - self.budget_vector[0])):
            print(i, self.hardening_df[self.hardening_df[i] > 0].shape)
        
    def non_nested_substation_count(self):        
        temp1 = 0
        for i in range(self.hardening_df.shape[0]):
            temp = 0
            for j in range(self.hardening_df.shape[1]):
                temp2 = self.hardening_df.iloc[i,j]
                if temp2 >= temp:
                    temp = temp2
                else:
                    temp1 = temp1 + 1
                    print("Substation not having nested hardening:\t",self.hardening_df.index[i])
                    break    