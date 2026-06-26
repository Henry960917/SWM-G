import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('HelicalCSV.csv')

fig = plt.figure(figsize=(10, 8))
ax = plt.subplot(projection='3d')

img = ax.scatter(df['CommandPos-0'], 
                 df['CommandPos-1'], 
                 df['CommandPos-2'], 
                 )

ax.set_title('HelicalInterpolation', fontsize=20)
ax.set_xlabel('X: CommandPos-0', fontsize=12)
ax.set_ylabel('Y: CommandPos-1', fontsize=12)
ax.set_zlabel('Z: CommandPos-2', fontsize=12)

plt.show()
