import csv
#import matplotlib.pyplot as plt
#import easygui
#import pygame
def get_choice(title, choices):
    print(title)
    for i,c in enumerate(choices):
        print(f'{i}:{c}, ', end='')
    try:
        index = int(input("Choice #: "))
        return choices[index]

    except (ValueError, IndexError):
        return get_choice(title, choices)
def get_expr(curve):
    
    return "42"
store_path = r"C:\Users\Luke\Documents\FRC\NordicStorm2022\shootingcurves.txt"
file_path = r"C:\Users\Luke\Documents\FRC\NordicStorm2022\src\main\java\frc\robot\subsystems\Barrel.javab"
curves = {}
current_labels = []
with open(store_path,'r') as f:
    reader = csv.reader(f, delimiter=',')
    for row in reader:
        if row[0] == 'x':
            current_labels = list(row)
            curves = {}
            for label in current_labels:
                curves[label]=[]
        else:
            x=0
            for i,val in enumerate(row):
                val = float(val)
                if i == 0:x=val
                curves[current_labels[i]].append([x, val])

#print(curves)
lines = []
with open(file_path, 'r') as f:
    for l in f.readlines():
        l=l.rstrip()

        if "//CURVE" in l:
            indent = len(l)-len(l.lstrip())
            name = l.split("CURVE:")[1]
            name = name.split(",")[0]
            meta = f"//CURVE:{name},now"
            expr = get_expr(curve)
            l = " "*indent + f"double result = {expr}; {meta}"
        l+="\n"
        lines.append(l)

with open(file_path, 'w') as f:
    f.writelines(lines)
    
