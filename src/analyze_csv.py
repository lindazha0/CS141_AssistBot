import pandas as pd
import matplotlib.pyplot as plt
import os
# import numpy as np

CSV_DIR = '../results/'
BOX_PLOT = '../results/box_plot.png'

def explain_csv():
    """Explain the csv file."""
    print(f"Explaining csv files under {CSV_DIR}...")
    names = ['base', 'predict']

    times = []
    times.append(pd.read_csv(os.path.join(CSV_DIR, names[0] + '_time.csv')))
    for i in range(5):
        times.append(pd.read_csv(os.path.join(CSV_DIR, f"{names[1]}_time_{i}.csv")))
    df = pd.concat(times)
    # df = pd.read_csv(file_name, header=None)
    print(df.head())
    print(df.describe())
    box = df.plot(kind='box', title='Time Steps to Reach Terminating Condition', notch=True, patch_artist=True)
    plt.show()
    # print(df.shape)
    # print(df.info())

    # save box plot
    box.get_figure().savefig(BOX_PLOT)

if __name__ == '__main__':
    explain_csv()