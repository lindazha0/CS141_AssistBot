import pandas as pd

CSV_FILE = '../results/base_time.csv'

def explain_csv(file_name):
    print(f"Explaining {file_name}...")
    df = pd.read_csv(file_name, header=None)
    print(df.head())
    print(df.describe())
    # print(df.shape)
    # print(df.info())

if __name__ == '__main__':
    explain_csv(CSV_FILE)