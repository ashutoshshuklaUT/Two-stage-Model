import numpy as np
import pandas as pd
from pyomo.environ import *

def prepare_input(path_str):
    # Read, format and do per unit correction
    input1 = pd.read_csv(path_str + "Final_Input1.csv")
    input2 = pd.read_csv(path_str + "Final_Input2.csv")
    input1 = input1.reset_index()
    input1.rename(columns={'index':'bus_index'}, inplace=True)
    input1 = input1.set_index("BusNum")
    input2 = input2.reset_index()
    input2.rename(columns={'index':'edge_index'}, inplace=True)
    # Per unit system correction
    input1["generation_capacity_max"] = input1["generation_capacity_max"]/100
    input1["generation_capacity_min"] = input1["generation_capacity_min"]/100
    input1["load"] = input1["load"]/100
    input2["RATE_A"] = input2["RATE_A"]/100
    return input1, input2