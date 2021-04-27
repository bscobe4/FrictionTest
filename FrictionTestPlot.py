import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

datalog = pd.read_csv('datalog.csv')

datalog.plot(1, 0)
plt.show()
