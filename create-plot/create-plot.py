import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('motor-data.csv', header=None, names=['Motor', 'Duty Cycle', 'Speed'])
grouped = df.groupby('Motor')

plt.figure(figsize=(10, 6))

for name, group in grouped:
    plt.plot(group['Duty Cycle'], group['Speed'], label=f"Motor {name}")

plt.xlabel('Duty Cycle [%]')
plt.ylabel('Speed [m/s]')
plt.title('Duty Cycle [%] vs Speed [m/s] for motor 1 to 4')
plt.legend()
plt.grid(True)
plt.show()
