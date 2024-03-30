import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from sympy import S, symbols, printing

# Read data from csv file
df = pd.read_csv('motor-noload-data.csv', header=None, names=['Motor', 'Duty Cycle', 'Speed'])

# Adjust speed values for each motor
df.loc[df['Motor'] == 1, 'Speed'] = df.loc[df['Motor'] == 1, 'Speed'] * 0.898
df.loc[df['Motor'] == 2, 'Speed'] = df.loc[df['Motor'] == 2, 'Speed'] * 0.945
df.loc[df['Motor'] == 3, 'Speed'] = df.loc[df['Motor'] == 3, 'Speed'] * 0.879

# Remove outliers
df.drop(df[(df['Motor'] == 3) & (df['Duty Cycle'] == 47.5)].index, inplace=True)

# Pivot table to get speed values for each motor
df_wide = df.pivot_table('Speed', 'Duty Cycle', 'Motor')
df_wide["avg"] = df_wide.mean(axis=1)

# Remove first 10 rows with 0 speed
df_wide_for_fit = df_wide[df_wide["avg"] != 0]

# Fit a polynomial curve to the averaged speed values
x = np.array(df_wide_for_fit.index)
y = np.array(df_wide_for_fit["avg"])
coefficients = np.polyfit(x, y, 5)
p = np.poly1d(coefficients)

# Create plot
plt.figure(figsize=(10, 6))

# Plot averaged speed over all motors
plt.plot(df_wide.index, df_wide["avg"], label=f"Averaged", color='black', linestyle='dashed')

# Plot speed values for each motor
for i in range(1, 4):
    plt.plot(df_wide.index, df_wide[i], label=f"Motor {i}", alpha=0.5)

# Plot fitted curve
plt.plot(x, p(x), label='Fitted curve', color='red')

# Add text with polynomial coefficients
plt.text(0.3, 0.08, coefficients, fontsize=10, transform=plt.gca().transAxes)

# Add legend, labels, title and grid
plt.xlabel('Duty Cycle [%]')
plt.ylabel('Speed [m/s]')
plt.title('Duty Cycle [%] vs Speed [m/s] for motor 1 to 4')
plt.legend()
plt.grid(True)
plt.show()
