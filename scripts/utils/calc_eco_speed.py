import os
import pandas as pd
import numpy as np
import pathlib

def calculate_smoothed_speed(df, window_size=5):
    # Calculate deltas for x, y, z, and time
    df['delta_x'] = df['x'].diff().rolling(window=window_size, min_periods=1).mean()
    df['delta_y'] = df['y'].diff().rolling(window=window_size, min_periods=1).mean()
    df['delta_z'] = df['z'].diff().rolling(window=window_size, min_periods=1).mean()
    df['delta_time'] = df['current_time'].diff().rolling(window=window_size, min_periods=1).mean()

    # Calculate speed using the formula: sqrt((delta_x)^2 + (delta_y)^2 + (delta_z)^2) / delta_time
    # This will compute speed based on averaged changes in position and time.
    df['calculated_speed'] = df.apply(
        lambda row: (row['delta_x']**2 + row['delta_y']**2 + row['delta_z']**2)**0.5 / row['delta_time'] *3.6 # convert to kph
        if row['delta_time'] > 0 else 0,
        axis=1
    )

    # Drop the intermediate delta columns
    df.drop(columns=['delta_x', 'delta_y', 'delta_z', 'delta_time'], inplace=True)

    return df

# def calculate_speed(df):
#     # Calculate deltas for x, y, z, and time
#     df['delta_x'] = df['x'].diff()
#     df['delta_y'] = df['y'].diff()
#     df['delta_z'] = df['z'].diff()
#     df['delta_time'] = df['current_time'].diff()

#     # Calculate speed
#     df['calculated_speed'] = np.sqrt(df['delta_x']**2 + df['delta_y']**2 + df['delta_z']**2) / df['delta_time'] / 1000 * 3600

#     # Handle the first row where delta calculations result in NaN
#     df.at[0, 'calculated_speed'] = 0

#     # if we get a value over 100 m/s, it must be some sort of teleport/respawn/despawn situation and set the speed to 0
#     df.loc[df['calculated_speed'] > 100, 'calculated_speed'] = 0

#     # Drop the intermediate delta columns
#     df.drop(columns=['delta_x', 'delta_y', 'delta_z', 'delta_time'], inplace=True)
    
#     return df

def smooth_speeds(df):
    # skip to the first non-zero calculated speed, no need to smooth before the vehicle starts moving
    first_non_zero_index_df = df[df['calculated_speed'] != 0]

    # Find the index of the last non-zero value from the end in the "calculated_speed" column
    last_non_zero_index = df[df['calculated_speed'] != 0].index[-1]

    if len(first_non_zero_index_df.index) > 0:
        first_non_zero_index = first_non_zero_index_df.index[0]
    else:
        print("\tWARNING: All calculated speeds for this data are zero")
        return df
    
    indices_of_zeros_after_first_non_zero = df.index[(df['calculated_speed'] == 0) & 
                                                     (df.index > first_non_zero_index) &
                                                     (df.index < last_non_zero_index)].tolist()

    # zero_speed_indices = df.index[df['calculated_speed'] == 0].tolist()
    
    for index in indices_of_zeros_after_first_non_zero:
        # print(f"{index}")
        if index == 0:
            continue  # Skip the first row
        
        # Find nearest non-zero speeds before and after the current index
        prev_non_zero_index = df['calculated_speed'].iloc[:index][df['calculated_speed'][:index] != 0].last_valid_index()
        next_non_zero_index = df['calculated_speed'].iloc[index+1:][df['calculated_speed'][index+1:] != 0].first_valid_index()
        
        # Calculate average speed if possible
        if pd.notnull(prev_non_zero_index) and pd.notnull(next_non_zero_index):
            avg_speed = (df.at[prev_non_zero_index, 'calculated_speed'] + df.at[next_non_zero_index, 'calculated_speed']) / 2
            df.at[index, 'calculated_speed'] = avg_speed
            # print(f"  Smoothing with average of next and prev speed - {avg_speed}")
        elif pd.notnull(prev_non_zero_index):
            df.at[index, 'calculated_speed'] = df.at[prev_non_zero_index, 'calculated_speed']
            # print(f"  Smoothing using prev speed - {df.at[prev_non_zero_index, 'calculated_speed']}")
        elif pd.notnull(next_non_zero_index):
            df.at[index, 'calculated_speed'] = df.at[next_non_zero_index, 'calculated_speed']
            # print(f"  Smoothing using next speed - {df.at[next_non_zero_index, 'calculated_speed']}")
    
    return df

def apply_low_pass_filter(df, alpha=0.1):
    """
    Apply Exponential Moving Average (EMA) as a low-pass filter to the 'smoothed_speed' column.

    Args:
    df (pandas.DataFrame): DataFrame with the 'calculated_speed' column.
    alpha (float): The smoothing factor of EMA, between 0 and 1.
                   Small alpha results in more smoothing.

    Returns:
    pandas.DataFrame: DataFrame with an additional 'smoothed_speed' column.
    """
    df['smoothed_speed'] = df['calculated_speed'].ewm(alpha=alpha).mean()
    return df

def process_csv_files(directory):
    # Convert the directory path to a pathlib.Path object
    root_dir = pathlib.Path(directory)
    
    # Walk through the directory and its subdirectories
    for filename in root_dir.rglob('*.csv'):  # rglob method is used for recursive globbing
        filepath = str(filename)
        print(f'Checking: {filepath}')

    # for filename in os.listdir(directory):
        if filename.name.endswith(".csv"):
            # filepath = os.path.join(directory, filename)
            
            df = pd.read_csv(filepath)
            
            # Ensure the required fields are present
            if all(field in df.columns for field in ['current_time', 'x', 'y', 'z']):
                df = calculate_smoothed_speed(df)
                # df = smooth_speeds(df)  # Apply smoothing function
                df = apply_low_pass_filter(df, alpha=0.075)
                new_filename = filepath.replace('/ORIG/','/CALC/')
                df.to_csv(new_filename, index=False)
                print(f"Processed and saved: {new_filename}")
            else:
                print(f"Skipping {filename.name}, required fields not found.")

# Replace '/path/to/directory' with your actual directory path
directory_path = '/home/vug/voices-poc/logs/Official_Eco_Data/ORIG/'
process_csv_files(directory_path)