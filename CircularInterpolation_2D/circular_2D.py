import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('123456.csv')

plt.figure(figsize=(8, 8))
plt.scatter(df["CommandPos-0"], df["CommandPos-1"], color="blue", s=10)
plt.title("CircularInterpolation_2D", fontsize = 30)
plt.xlabel("CommandPos-0", fontsize = 20)
plt.ylabel("CommandPos-1", fontsize = 20)
plt.show()
