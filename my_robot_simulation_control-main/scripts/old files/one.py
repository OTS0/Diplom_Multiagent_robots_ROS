#!/usr/bin/env python3  

mass = 30
width = 1.1
height = 1.9
length = 2.5

# Расчет моментов инерции
ixx = (mass / 12) * (height**2 + length**2)
iyy = (mass / 12) * (width**2 + length**2)
izz = (mass / 12) * (width**2 + height**2)

print("ixx:")
print(ixx)
print("iyy:")
print(iyy)
print("izz:")
print(izz)
