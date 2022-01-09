import csv

path = "/home/anisha/flipkart_ws/src/FlipkartGrid/grid_control/grid_phase2_controller/data/Sample Data - Sheet1.csv"
induct_list = [[], []]
with open(path) as file:
    reader = csv.reader(file)
    for i, row in enumerate(reader):
        if i == 0:
            continue
        induct_list[int(row[1]) - 1].append([row[0],row[-1]])
# print(induct_list)
print(induct_list[1][0])
