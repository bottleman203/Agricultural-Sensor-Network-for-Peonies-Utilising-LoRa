import pandas as pd
import matplotlib.pyplot as plt

# Data
data_str = '''<your CSV data here>'''

# Load the data into a pandas DataFrame
from io import StringIO
data = pd.read_csv(StringIO(data_str))

# Convert the Timestamp to datetime format
data['Timestamp'] = pd.to_datetime(data['Timestamp'])

# Plotting RSSI against time
plt.figure(figsize=(10,6))
plt.plot(data['Timestamp'], data['RSSI'], label='RSSI', marker='o')
plt.xlabel('Timestamp')
plt.ylabel('RSSI')
plt.title('RSSI Over Time')
plt.grid(True)
plt.xticks(rotation=45)
plt.tight_layout()

# Show the plot
plt.show()
