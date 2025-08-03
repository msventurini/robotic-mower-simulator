"""
tachytools.py - Utility functions for processing and visualizing tachymeter data

This module provides a collection of functions for:
- Loading and processing data from XML files
- Rotating, transforming, and filtering point data
- Creating and analyzing convex hulls
- Detecting standing positions and outside-boundary events
- Visualizing trajectories, hulls, and statistical analyses
- Calculating straightness metrics and generating reports

Primarily used for robotic lawnmower trajectory analysis and evaluation.
"""

# Standard library imports
import xml.etree.ElementTree as ET

# Third-party imports
# - Data analysis and manipulation
import numpy as np
import pandas as pd

# - Scientific and math libraries
from scipy import interpolate
from scipy.interpolate import interp1d
from scipy.interpolate import griddata
from scipy.spatial import ConvexHull
from sklearn.metrics import r2_score

# - Geometry and spatial analysis
from shapely.geometry import (
    Point, Polygon, MultiPolygon, 
    LinearRing, LineString, MultiLineString, 
    GeometryCollection
)
from shapely.ops import unary_union
from shapely.prepared import prep
from shapely.validation import make_valid

# - Plotting and visualization
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# - Date/time handling
import pytz
import datetime

def rotate_points(x_data, y_data, angle_degrees):
    """
    Rotate points in 2D space (original function for backward compatibility).
    
    Parameters:
    x_data, y_data: Arrays of x, y coordinates
    angle_degrees: Rotation angle in degrees
    
    Returns:
    Rotated x, y coordinates
    """
    # Convert angle from degrees to radians
    angle_radians = np.radians(angle_degrees)
    
    # Create rotation matrix
    rotation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians)],
        [np.sin(angle_radians), np.cos(angle_radians)]
    ])
    
    # Stack x_data and y_data into a 2D array
    points = np.vstack((x_data, y_data))
    
    # Apply rotation
    rotated_points = rotation_matrix @ points
    
    return rotated_points[0], rotated_points[1]

def load_xml(xml_file, rotate = 0, mirror = False, remove_tachy_point = True, drop_points = False, include_z = False):
    """
    Load coordinates from XML file with option to include Z data.
    
    Parameters:
    xml_file: Path to the XML file
    rotate: Rotation angle in degrees (default: 0)
    mirror: Whether to mirror points over y-axis (default: False)
    remove_tachy_point: Whether to remove the last point (default: True)
    drop_points: Number of points to drop from the beginning (default: False)
    include_z: Whether to include Z coordinates in the return values (default: False)
    
    Returns:
    If include_z is False: x_data, y_data, timestamp (original behavior)
    If include_z is True: x_data, y_data, z_data, timestamp
    """
    # XML-Datei parsen
    tree = ET.parse(xml_file)
    root = tree.getroot()
    x_data = []
    y_data = []
    z_data = []
    timestamp = []

    # Namespace-Definition für das XML-Parsing
    ns = {'landxml': 'http://www.landxml.org/schema/LandXML-1.2'}

    for cgpoint in root.findall('.//landxml:CgPoint', ns):
        coords = cgpoint.text.split()
        x_data.append(float(coords[0]))  # x-Koordinate
        y_data.append(float(coords[1]))  # y-Koordinate
        z_data.append(float(coords[2]))  # z-Koordinate
        timestamp.append(cgpoint.get('timeStamp'))
    
    if rotate != 0:
        if include_z:
            x_data, y_data, z_data = rotate_points_3d(x_data, y_data, z_data, rotate, 'z')
        else:
            x_data, y_data = rotate_points(x_data, y_data, rotate)
    
    if mirror == True:
        # Mirror the points (reflect over the y-axis)
        x_data = [-x for x in x_data]

    if remove_tachy_point == True:
        x_data  = x_data[:-1]
        y_data  = y_data[:-1]
        z_data  = z_data[:-1]
        timestamp = timestamp[:-1]
    
    if drop_points != False:
        x_data  = x_data[drop_points:]
        y_data  = y_data[drop_points:]
        z_data  = z_data[drop_points:]
        timestamp = timestamp[drop_points:]

    # Return original values if include_z is False (for backward compatibility)
    if include_z:
        return x_data, y_data, z_data, timestamp
    else:
        return x_data, y_data, timestamp
    
def rotate_points_3d(x_data, y_data, z_data, angle_degrees=0, axis='z'):
    """
    Rotate points in 3D space around a specified axis.
    
    Parameters:
    x_data, y_data, z_data: Arrays of x, y, z coordinates
    angle_degrees: Rotation angle in degrees
    axis: Axis of rotation ('x', 'y', or 'z')
    
    Returns:
    Rotated x, y, z coordinates
    """
    # Convert angle from degrees to radians
    angle_radians = np.radians(angle_degrees)
    
    # Stack coordinates into a 3D array
    points = np.vstack((x_data, y_data, z_data))
    
    # Create rotation matrix based on the specified axis
    if axis.lower() == 'x':
        # Rotation around x-axis
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(angle_radians), -np.sin(angle_radians)],
            [0, np.sin(angle_radians), np.cos(angle_radians)]
        ])
    elif axis.lower() == 'y':
        # Rotation around y-axis
        rotation_matrix = np.array([
            [np.cos(angle_radians), 0, np.sin(angle_radians)],
            [0, 1, 0],
            [-np.sin(angle_radians), 0, np.cos(angle_radians)]
        ])
    else:  # Default is z-axis rotation
        # Rotation around z-axis (same as 2D rotation)
        rotation_matrix = np.array([
            [np.cos(angle_radians), -np.sin(angle_radians), 0],
            [np.sin(angle_radians), np.cos(angle_radians), 0],
            [0, 0, 1]
        ])
    
    # Apply rotation
    rotated_points = rotation_matrix @ points
    
    return rotated_points[0], rotated_points[1], rotated_points[2]

def create_hull(x_data, y_data, closed=True, min_area=0.1):
    points = [(x, y) for x, y in zip(x_data, y_data)]
    if closed:
        hull = Polygon(points)
        if isinstance(hull, Polygon):
            return hull if hull.area > min_area else None
        elif isinstance(hull, MultiPolygon):
            largest_poly = max(hull.geoms, key=lambda poly: poly.area)
            return largest_poly if largest_poly.area > min_area else None
    else:
        hull = LineString(points)
        print("Full not closed.")
        return hull
    
def moving_average(values, window_size):
    averages = []
    for i in range(len(values)):
        if i < window_size - 1:
            # For the initial values, use the average of the available elements
            window = values[:i + 1]
        else:
            window = values[i - window_size + 1:i + 1]
        averages.append(sum(window) / len(window))
    return averages

def create_velocity_df(x_data, y_data, timestamp, velocity_filter = -1):

    timestamp_local = pd.to_datetime(timestamp)
    unix_timestamp = timestamp_local.astype(int) / 10**9
    unix_timestamp_norm = unix_timestamp - np.min(unix_timestamp) +1

    dx = np.diff(x_data)
    dy = np.diff(y_data)
    dt = abs(np.diff(unix_timestamp_norm))

    velocity = abs(np.sqrt(dx**2 + dy**2) / dt)
    filtered_velocity = moving_average(velocity,10)
    filtered_velocity = np.array(filtered_velocity)

    if velocity_filter < 0:
        # Calculate mean and standard deviation
        mean_velocity = np.mean(filtered_velocity)
        std_velocity = np.std(filtered_velocity)

        # Replace values greater than the standard deviation with the mean
        filtered_velocity[filtered_velocity > (mean_velocity +  std_velocity)] = mean_velocity
    else:
        filtered_velocity[filtered_velocity > velocity_filter] = velocity_filter

    filtered_velocity = np.insert(filtered_velocity,0,filtered_velocity[0])
    velocity = np.insert(velocity,0,velocity[0])

    df = pd.DataFrame({
    "x_data": x_data,
    "y_data": y_data,
    "velocity": velocity,
    "moving_avg_velocity": filtered_velocity,
    "date_time": timestamp,
    "unix_timestamp_norm": unix_timestamp_norm
    })

    return df, dt

def create_velocity_clusters(df: pd.DataFrame, velocity_filter: bool = True, 
                           enter_standing_threshold: float = 0.005, 
                           exit_standing_threshold: float = 0.015, 
                           minimum_standing_time: float = 10):
    """
    Create clusters of standing positions based on velocity data with improved accuracy.
    
    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing robot position and velocity data
    velocity_filter : bool, default=True
        Whether to use the moving average velocity or raw velocity
    enter_standing_threshold : float, default=0.005
        Threshold below which the robot enters standing state
    exit_standing_threshold : float, default=0.015
        Threshold above which the robot exits standing state
    minimum_standing_time : float, default=10
        Minimum duration (seconds) for a valid standing event
        
    Returns:
    --------
    pd.DataFrame
        DataFrame containing standing clusters with positions and timestamps
    """
    # Create a copy to avoid modifying the original DataFrame
    df_copy = df.copy()
    
    # Pre-filter velocity data to reduce noise with a median filter
    velocity_col = 'moving_avg_velocity' if velocity_filter else 'velocity'
    window_size = 3
    if len(df_copy) >= window_size:
        df_copy[f'{velocity_col}_filtered'] = df_copy[velocity_col].rolling(
            window=window_size, center=True, min_periods=1).median()
    else:
        df_copy[f'{velocity_col}_filtered'] = df_copy[velocity_col]
    
    # Determine standing states with hysteresis thresholds
    df_copy['standing'] = 0  # Initialize all to not standing
    standing_state = 0
    
    for i in range(len(df_copy)):
        current_velocity = abs(df_copy.iloc[i][f'{velocity_col}_filtered'])
        
        # Apply hysteresis logic
        if standing_state == 0 and current_velocity < enter_standing_threshold:
            standing_state = 1  # Enter standing state
        elif standing_state == 1 and current_velocity > exit_standing_threshold:
            standing_state = 0  # Exit standing state
            
        df_copy.iloc[i, df_copy.columns.get_loc('standing')] = standing_state
    
    # Identify points where standing state changes
    df_copy['standing_change'] = df_copy['standing'] != df_copy['standing'].shift(1)
    df_copy.iloc[0, df_copy.columns.get_loc('standing_change')] = True  # First point is always a change
    
    # Extract change points
    df_changed = df_copy[df_copy['standing_change'] == True].copy()
    
    # Create clusters dataframe
    clusters = pd.DataFrame(columns=["x_data", "y_data", 'standing_time', 'start_standing', 'end_standing'])
    
    # Process standing intervals only if we have change points
    if len(df_changed) >= 2:
        # Convert change points to start/end pairs
        standing_starts = []
        standing_ends = []
        
        # Handle first point based on its state
        is_first_standing = df_changed.iloc[0]['standing'] == 1
        
        # Process change points to extract start/end pairs
        for i in range(len(df_changed)):
            is_standing = df_changed.iloc[i]['standing'] == 1
            
            if i == 0 and not is_first_standing:
                # Skip first point if it's not standing
                continue
                
            if is_standing:
                standing_starts.append(i)
            else:
                standing_ends.append(i)
        
        # Handle case where we end in standing state
        if len(standing_starts) > len(standing_ends):
            # Add a synthetic end point using the last row of original dataframe
            last_row = df_copy.iloc[-1:].copy()
            last_row['standing'] = 0
            last_row['standing_change'] = True
            df_changed = pd.concat([df_changed, last_row], ignore_index=True)
            standing_ends.append(len(df_changed) - 1)
        
        # Make sure we have equal number of starts and ends
        if len(standing_starts) == len(standing_ends):
            # Extract positions at start and end of standing intervals
            standing_positions = []
            for start_idx, end_idx in zip(standing_starts, standing_ends):
                start_row = df_changed.iloc[start_idx]
                end_row = df_changed.iloc[end_idx]
                
                # Calculate average position during standing
                x_position = df_copy.loc[
                    (df_copy['unix_timestamp_norm'] >= start_row['unix_timestamp_norm']) & 
                    (df_copy['unix_timestamp_norm'] <= end_row['unix_timestamp_norm']), 
                    'x_data'
                ].mean()
                
                y_position = df_copy.loc[
                    (df_copy['unix_timestamp_norm'] >= start_row['unix_timestamp_norm']) & 
                    (df_copy['unix_timestamp_norm'] <= end_row['unix_timestamp_norm']), 
                    'y_data'
                ].mean()
                
                # Calculate standing duration
                standing_time = end_row['unix_timestamp_norm'] - start_row['unix_timestamp_norm']
                
                # Get start and end times - ensure they are datetime objects
                start_time = pd.to_datetime(start_row['date_time']) if not isinstance(start_row['date_time'], pd.Timestamp) else start_row['date_time']
                end_time = pd.to_datetime(end_row['date_time']) if not isinstance(end_row['date_time'], pd.Timestamp) else end_row['date_time']
                
                # Store unix timestamps - ensure they're numeric
                start_unix = float(start_row['unix_timestamp_norm'])
                end_unix = float(end_row['unix_timestamp_norm'])
                
                standing_positions.append({
                    'x_data': x_position,
                    'y_data': y_position,
                    'standing_time': standing_time,
                    'start_standing': start_time,
                    'end_standing': end_time,
                    'start_standing_unix': start_unix,
                    'end_standing_unix': end_unix
                })
            
            # Convert to DataFrame and filter by minimum standing time
            if standing_positions:
                clusters = pd.DataFrame(standing_positions)
                clusters = clusters[clusters['standing_time'] > minimum_standing_time]
                clusters = clusters.reset_index(drop=True)
    
    return clusters

def interpolate_missing_points(df: pd.DataFrame, max_time_gap: float = 0.5, method: str = 'linear'):
    """
    Detects gaps in timestamp data and returns only the interpolated points using vectorized operations.
    
    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing robot tracking data with columns 'unix_timestamp_norm', 
        'x_data', 'y_data', and optionally velocity data
    max_time_gap : float, default=2.0
        Maximum allowable time gap in seconds; gaps larger than this will be interpolated
    method : str, default='linear'
        Interpolation method to use ('linear', 'cubic', 'spline', etc.)
        
    Returns:
    --------
    pd.DataFrame
        DataFrame containing only the interpolated points
    """
    import pandas as pd
    import numpy as np
    from scipy import interpolate
    
    # Sort data by timestamp
    df_sorted = df.copy().sort_values(by='unix_timestamp_norm').reset_index(drop=True)
    
    # Calculate time differences between consecutive points
    time_diffs = df_sorted['unix_timestamp_norm'].diff()
    
    # Find gaps exceeding the threshold
    gap_indices = np.where(time_diffs > max_time_gap)[0]
    
    if len(gap_indices) == 0:
        # No gaps found, return empty DataFrame with same structure
        return pd.DataFrame(columns=df_sorted.columns)
    
    # Create lists to store all interpolated points
    all_interp_points = []
    
    # Define columns to interpolate
    columns_to_interpolate = ['x_data', 'y_data']
    if 'velocity' in df_sorted.columns:
        columns_to_interpolate.append('velocity')
    if 'moving_avg_velocity' in df_sorted.columns:
        columns_to_interpolate.append('moving_avg_velocity')
    
    # Ensure date_time is a proper datetime
    if 'date_time' in df_sorted.columns:
        if not pd.api.types.is_datetime64_any_dtype(df_sorted['date_time']):
            df_sorted['date_time'] = pd.to_datetime(df_sorted['date_time'])
    
    # Process each gap
    for gap_idx in gap_indices:
        # Get start and end points of the gap
        start_idx = gap_idx - 1
        end_idx = gap_idx
        
        # Get timestamps at gap boundaries
        start_time = df_sorted.iloc[start_idx]['unix_timestamp_norm']
        end_time = df_sorted.iloc[end_idx]['unix_timestamp_norm']
        gap_duration = end_time - start_time
        
        # Calculate number of points to insert (at least 1)
        num_points = max(1, int(gap_duration))
        
        # Generate new timestamps (exclude endpoints)
        new_timestamps = np.linspace(start_time, end_time, num_points + 2)[1:-1]
        
        if len(new_timestamps) == 0:
            continue
        
        # Create template for interpolated points
        interp_df = pd.DataFrame({'unix_timestamp_norm': new_timestamps})
        
        # Prepare reference points for interpolation (start and end points of gap)
        ref_points = df_sorted.iloc[[start_idx, end_idx]]
        
        # Interpolate each required column
        for col in columns_to_interpolate:
            # Get values at gap boundaries
            start_val = ref_points.iloc[0][col]
            end_val = ref_points.iloc[1][col]
            # For other methods, use scipy's interp1d
            f = interpolate.interp1d(
                [start_time, end_time], 
                [start_val, end_val],
                kind=method, 
                bounds_error=False,
                fill_value="extrapolate"
            )
            interp_df[col] = f(new_timestamps)
        
        # Handle date_time column if present
        if 'date_time' in df_sorted.columns:
            # First ensure both datetime values are proper datetime objects
            start_dt = df_sorted.iloc[start_idx]['date_time']
            end_dt = df_sorted.iloc[end_idx]['date_time']
            
            # Convert to timestamp (seconds since epoch) for proper interpolation
            start_ts = start_dt.timestamp()
            end_ts = end_dt.timestamp()
            
            # Create interpolation function for timestamps
            f_dt = interpolate.interp1d(
                [start_time, end_time],
                [start_ts, end_ts],
                kind=method,
                bounds_error=False,
                fill_value="extrapolate"
            )
            
            # Calculate interpolated timestamps
            interp_ts = f_dt(new_timestamps)
            
            # Convert interpolated timestamps back to datetime objects
            # Preserve timezone if present
            if hasattr(start_dt, 'tzinfo') and start_dt.tzinfo is not None:
                interp_df['date_time'] = pd.to_datetime(interp_ts, unit='s', utc=True)
                # Convert to match original timezone
                if str(start_dt.tzinfo) != 'UTC':
                    interp_df['date_time'] = interp_df['date_time'].dt.tz_convert(start_dt.tzinfo)
            else:
                interp_df['date_time'] = pd.to_datetime(interp_ts, unit='s')
        
        # Add any other columns from original DataFrame with NaN values
        for col in df_sorted.columns:
            if col not in interp_df.columns:
                interp_df[col] = np.nan
        
        all_interp_points.append(interp_df)
    
    # Combine all interpolated points
    if all_interp_points:
        result = pd.concat(all_interp_points).sort_values(by='unix_timestamp_norm').reset_index(drop=True)
        return result
    else:
        return pd.DataFrame(columns=df_sorted.columns)

def plot_point_and_hull(df,hull):

    fig, ax = plt.subplots(figsize = (10,10), dpi=1200) #BIG PICTURE
    #ConvexHullHULL
    if isinstance(hull, Polygon):
        x_hull, y_hull = hull.exterior.xy
        plt.plot(x_hull, y_hull, 'k-')
        plt.fill(x_hull, y_hull, 'lightgrey', alpha=0.5)
    elif isinstance(hull, MultiPolygon):
        for poly in hull.geoms:
            x_hull, y_hull = poly.exterior.xy
            plt.plot(x_hull, y_hull, 'k-')
            plt.fill(x_hull, y_hull, 'lightgrey', alpha=0.5)
    ax.set_aspect('equal', 'box')
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.scatter(df["x_data"], df["y_data"], s=1, alpha=0.5)

def plot_standing(hull, velocity_df, clusters, save: bool = False, filename = "Standing_points.png"):
    """
    Plot the standing points of the robot on a map.
    
    Parameters:
    -----------
    hull : shapely.geometry.Polygon or shapely.geometry.MultiPolygon
        The hull area to plot
    velocity_df : pd.DataFrame
        DataFrame containing robot tracking data
    clusters : pd.DataFrame
        DataFrame containing the standing clusters
    save : bool, default=False
        Whether to save the plot to a file
    filename : str, default="Standing_points.png"
        Filename to save the plot if save=True
    """
    fig, ax = plt.subplots(figsize = (10,10), dpi=1200) #BIG PICTURE
    
    # Plot the hull
    if isinstance(hull, Polygon):
        x_hull, y_hull = hull.exterior.xy
        plt.plot(x_hull, y_hull, 'k-')
        plt.fill(x_hull, y_hull, 'lightgrey', alpha=0.5)
    elif isinstance(hull, MultiPolygon):
        for poly in hull.geoms:
            x_hull, y_hull = poly.exterior.xy
            plt.plot(x_hull, y_hull, 'k-')
            plt.fill(x_hull, y_hull, 'lightgrey', alpha=0.5)
    
    # Create a colormap based on velocity
    cmap = plt.get_cmap('viridis')
    colors = cmap(velocity_df["velocity"].copy(deep=True))
    
    # Create colorbar
    norm = Normalize(vmin=velocity_df["velocity"].min(), vmax=0.5)
    sm = ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label('Velocity [m/s]')

    # Plot the velocity points
    ax.scatter(velocity_df["x_data"], velocity_df["y_data"], s=1, c=colors, alpha=0.5)

    # Plot the clusters
    for i in range(len(clusters)):
        if clusters['standing_time'][i] > 1:
            center = (clusters['x_data'][i], clusters['y_data'][i])
            size = clusters['standing_time'][i]
            
            # Handle timestamp formatting with better error handling
            timestamp = clusters['start_standing'][i]
            
            # Try to convert to datetime if it's a string
            if isinstance(timestamp, str):
                try:
                    timestamp = pd.to_datetime(timestamp)
                except:
                    # If conversion fails, just use the string as is
                    timestamp_str = f"{timestamp} (format unknown)"
                    
            # Format the timestamp if it's a datetime object
            if isinstance(timestamp, (pd.Timestamp, datetime.datetime)):
                try:
                    # Check if it has timezone info
                    if hasattr(timestamp, 'tzinfo') and timestamp.tzinfo is not None:
                        try:
                            # Try to convert to CET if pytz is available
                            import pytz
                            cet = pytz.timezone('CET')
                            timestamp_cet = timestamp.astimezone(cet)
                            timestamp_str = timestamp_cet.strftime('%d-%m %H:%M:%S')
                        except ImportError:
                            # Fallback without pytz
                            timestamp_str = timestamp.strftime('%d-%m %H:%M:%S') + " UTC"
                    else:
                        # For timezone-naive timestamps
                        timestamp_str = timestamp.strftime('%d-%m %H:%M:%S') + " (local)"
                except Exception as e:
                    # Last resort fallback if all formatting fails
                    timestamp_str = f"Time data error: {str(e)[:20]}"
            else:
                # If it's neither string nor datetime
                timestamp_str = f"Start: {timestamp}"
            
            # Plot cluster center and label
            plt.scatter(center[0], center[1], marker='+', c='r')
            plt.text(center[0], center[1], f"{size:.1f} s \n Starting at: {timestamp_str}", 
                     fontsize=8, fontweight='bold', ha='center', va='center', c='k')
    
    # Set the aspect of the plot to be equal
    plt.title('Velocity distribution and standing points')
    ax.set_aspect('equal', 'box')
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
    
    # Save or show the plot
    if save:
        plt.savefig(filename, bbox_inches='tight')
        plt.close(fig)  # Close the figure to free memory
    else:
        plt.tight_layout()
        plt.show()

def create_outside_df(df, hull, min_outside_dist=0.1):
    """
    Identify points outside the hull boundary and cluster them.
    
    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing robot tracking data
    hull : shapely.geometry.Polygon or shapely.geometry.MultiPolygon
        The boundary hull
    min_outside_dist : float, default=0.1
        Minimum distance from hull boundary to consider a point as outside
        
    Returns:
    --------
    tuple
        (df_outside, outside_cluster) - DataFrame of outside points and their clusters
    """
    # Calculate distance to hull boundary for each point
    df['distance_to_border'] = df.apply(
        lambda point: abs(Point(point['x_data'], point['y_data']).distance(hull)), 
        axis=1
    )

    # Identify points outside the hull with minimum distance
    df['outside'] = df.apply(
        lambda point: 1 if (not hull.contains(Point(point['x_data'], point['y_data'])) 
                            and point['distance_to_border'] > min_outside_dist) else 0,
        axis=1
    )

    # Detect changes in outside status
    df['outside_change'] = df['outside'] != df['outside'].shift(1)
    df.iloc[0, df.columns.get_loc('outside_change')] = False

    # Get points that are outside
    df_outside = df[df['outside'] == 1].copy()
    
    # Sort by timestamp instead of date_time to avoid type issues
    if 'unix_timestamp_norm' in df_outside.columns:
        df_outside = df_outside.sort_values(by='unix_timestamp_norm', ascending=True)
    
    # Get points where outside status changes
    df_outside_change = df[df['outside_change'] == True].copy()
    
    # Ensure even number of change points (entry/exit pairs)
    if len(df_outside_change) % 2 != 0:
        df_outside_change = df_outside_change.iloc[:-1]
    
    # Sort by timestamp instead of date_time
    if 'unix_timestamp_norm' in df_outside_change.columns:
        df_outside_change = df_outside_change.sort_values(by='unix_timestamp_norm', ascending=True)
    
    df_outside_change = df_outside_change.reset_index()

    # Create clusters DataFrame
    outside_cluster = pd.DataFrame(columns=["x_data", "y_data", 'outside_time', 'distance_to_border'])
    
    # Handle case when there are no outside points
    if len(df_outside_change) < 2:
        return df_outside, outside_cluster
    
    # Extract indices for outside event pairs
    indices = df_outside_change['index'].values[:len(df_outside_change) - len(df_outside_change) % 2]
    outside_range = indices.reshape(-1, 2).tolist()
    
    # Process each outside event
    for i in range(len(outside_range)):
        # Get data slice for this outside event
        start_idx, end_idx = outside_range[i]
        
        # Make sure the indices are in the right order
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
        
        df_slice = df.loc[start_idx:end_idx]
        
        if not df_slice.empty:
            # Find point with maximum distance from border
            idx = df_slice['distance_to_border'].idxmax()
            
            # Record cluster data
            outside_cluster.loc[i, 'x_data'] = df.loc[idx, 'x_data']
            outside_cluster.loc[i, 'y_data'] = df.loc[idx, 'y_data']
            
            # Calculate time spent outside
            if 'unix_timestamp_norm' in df.columns:
                outside_cluster.loc[i, 'outside_time'] = df.loc[end_idx, 'unix_timestamp_norm'] - df.loc[start_idx, 'unix_timestamp_norm']
            else:
                outside_cluster.loc[i, 'outside_time'] = np.nan
                
            outside_cluster.loc[i, 'distance_to_border'] = df.loc[idx, 'distance_to_border']
    
    return df_outside, outside_cluster

def plot_outside(hull, df_outside, outside_cluster, save: bool = False, filename = "Outside_plot.png"):
    fig, ax = plt.subplots(figsize = (10,10), dpi=1200) #BIG PICTURE
    #fig, ax = plt.subplots() # SMALL PICTURE

    if isinstance(hull, Polygon):
        x_hull, y_hull = hull.exterior.xy
        plt.plot(x_hull, y_hull, 'k-')
        plt.fill(x_hull, y_hull, 'lightgrey', alpha=0.5)
    elif isinstance(hull, MultiPolygon):
        for poly in hull.geoms:
            x_hull, y_hull = poly.exterior.xy
            plt.plot(x_hull, y_hull, 'k-')
            plt.fill(x_hull, y_hull, 'lightgrey', alpha=0.5)

    ax.scatter(df_outside['x_data'], df_outside['y_data'], s=1)
    ax.scatter(outside_cluster['x_data'], outside_cluster['y_data'], marker='+', c='r')
    for i in range(len(outside_cluster)):
        ax.annotate(f"{(outside_cluster['distance_to_border'][i]):.1f}m", (outside_cluster['x_data'][i], outside_cluster['y_data'][i]))
    plt.title(f'Maximum outside distance and location')
    ax.set_aspect('equal', 'box')
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
    # Save or show the plot
    if save:
        plt.savefig(filename, bbox_inches='tight')
        plt.close(fig)  # Close the figure to free memory
    else:
        plt.tight_layout()
        plt.show()

def process_interpoled_points(hull, df, radius=0.15):
    """
    Process only points inside the hull by creating buffers around them.
    
    Parameters:
    hull (Shapely geometry): The convex hull
    df (DataFrame): DataFrame with x_data and y_data columns
    radius (float): Buffer radius for each point
    
    Returns:
    Shapely geometry: Combined buffered points that are inside the hull
    """
    if len(df['x_data']) < 1:
        return 
    # Filter points inside hull
    inside_hull = [hull.contains(Point(x, y)) for x, y in zip(df['x_data'], df['y_data'])]
    filtered_df = df[inside_hull].copy()
    
    # Create circles around each point inside the hull
    circles = [Point(x, y).buffer(radius) for x, y in zip(filtered_df['x_data'], filtered_df['y_data'])]
    
    # Ensure hull is valid
    hull = make_valid(hull)
    
    # Combine all circles using unary_union
    try:
        combined_circles = unary_union(circles)
        combined_circles = make_valid(combined_circles)
    except Exception as e:
        print(f"Error in unary_union: {e}")
        # Fallback: try to buffer slightly to resolve topology issues
        buffered_circles = [circle.buffer(0.0000001) for circle in circles]
        combined_circles = unary_union(buffered_circles)
        combined_circles = make_valid(combined_circles)
    
    return combined_circles

def process_holes(hull, df, radius=0.15):
    # Create circles around each point
    circles = [Point(x, y).buffer(radius) for x, y in zip(df['x_data'], df['y_data'])]
    
    # Ensure hull is valid
    hull = make_valid(hull)
    
    # Combine all circles using unary_union
    try:
        combined_circles = unary_union(circles)
        combined_circles = make_valid(combined_circles)
    except Exception as e:
        print(f"Error in unary_union: {e}")
        # Fallback: try to buffer slightly to resolve topology issues
        buffered_circles = [circle.buffer(0.0000001) for circle in circles]
        combined_circles = unary_union(buffered_circles)
        combined_circles = make_valid(combined_circles)
    
    # Apply a small buffer to fix potential topology issues
    hull_buffered = hull.buffer(0.0000001)
    combined_circles_buffered = combined_circles.buffer(0.0000001)
    
    # Calculate the mowed area directly as the intersection
    try:
        mowed_area = hull_buffered.intersection(combined_circles_buffered)
        mowed_area = make_valid(mowed_area)
    except Exception as e:
        print(f"Error in intersection: {e}")
        # Fallback: try with larger buffer
        hull_buffered = hull.buffer(0.000001)
        combined_circles_buffered = combined_circles.buffer(0.000001)
        mowed_area = hull_buffered.intersection(combined_circles_buffered)
        mowed_area = make_valid(mowed_area)
    
    # Calculate the difference to get the holes using a safe approach
    try:
        # Use buffer(0) trick to fix potential topology issues
        hull_fixed = hull_buffered.buffer(0)
        mowed_area_fixed = mowed_area.buffer(0)
        holes = hull_fixed.difference(mowed_area_fixed)
        holes = make_valid(holes)
    except Exception as e:
        print(f"Error in difference operation: {e}")
        # Fallback if difference still fails
        try:
            # Try with prepared geometries

            prepared_mowed = prep(mowed_area_fixed)
            # Get the hull boundary and create small polygons where it doesn't intersect mowed area
            # This is a simplistic approach - might not give perfect results
            hull_boundary = hull_fixed.boundary
            holes = hull_fixed
            for i in range(10):  # Try a few iterations with increasingly larger buffers
                try:
                    buffer_val = 0.0000001 * (10 ** i)
                    mowed_buffered = mowed_area_fixed.buffer(buffer_val)
                    holes = hull_fixed.difference(mowed_buffered)
                    if not holes.is_empty:
                        break
                except:
                    continue
        except Exception as nested_e:
            print(f"Fallback also failed: {nested_e}")
            # Last resort: Return an empty geometry
            holes = GeometryCollection([])
    
    # Ensure holes is a MultiPolygon or handle empty case
    if holes.is_empty:
        # Create an empty MultiPolygon
        holes = MultiPolygon([])
        hole_areas = np.array([])
    elif isinstance(holes, Polygon):
        holes = MultiPolygon([holes])
        hole_areas = np.array([holes.area])
    else:
        # Handle GeometryCollection or other geometry types
        if isinstance(holes, GeometryCollection):
            # Extract only polygons from the collection
            polygons = [geom for geom in holes.geoms if isinstance(geom, Polygon)]
            if polygons:
                holes = MultiPolygon(polygons)
                hole_areas = np.array([poly.area for poly in polygons])
            else:
                holes = MultiPolygon([])
                hole_areas = np.array([])
        elif hasattr(holes, 'geoms'):
            # It's already a MultiPolygon or similar with geoms
            hole_areas = np.array([poly.area for poly in holes.geoms])
        else:
            # Try to get area directly
            try:
                hole_areas = np.array([holes.area])
            except:
                hole_areas = np.array([])
    
    # Create a DataFrame to store the results
    results_df = pd.DataFrame({
        'Hole Area': hole_areas
    })
    
    return mowed_area, holes, results_df

def plot_holes(hull, mowed_area, holes, interpolated_points=None, save: bool = False, filename="mowed_area.png"):
    """
    Plot the mowed area, holes, and optionally interpolated points.
    
    Parameters:
    -----------
    hull : shapely.geometry.Polygon or shapely.geometry.MultiPolygon
        The boundary hull
    mowed_area : shapely.geometry.Polygon or shapely.geometry.MultiPolygon
        The area that was mowed
    holes : shapely.geometry.Polygon or shapely.geometry.MultiPolygon
        The unmowed areas (holes)
    interpolated_points : pd.DataFrame or bool, default=None
        DataFrame containing interpolated points to plot in yellow
    save : bool, default=False
        Whether to save the plot to a file
    filename : str, default="mowed_area.png"
        Filename to save the plot if save=True
    """
    fig, ax = plt.subplots(figsize=(10, 10), dpi=1200)
    
    # Plot the hull
    if isinstance(hull, Polygon):
        x_hull, y_hull = hull.exterior.xy
        plt.plot(x_hull, y_hull, 'k-')
        plt.fill(x_hull,y_hull,'blue', alpha=1)
    elif isinstance(hull, MultiPolygon):
        for poly in hull.geoms:
            x_hull, y_hull = poly.exterior.xy
            plt.plot(x_hull, y_hull, 'k-')
            plt.fill(x_hull,y_hull,'blue', alpha=1)
    
    # Plot the holes (unmowed areas) in red
    if isinstance(holes, Polygon):
        x, y = holes.exterior.xy
        #plt.plot(x, y, 'k-')
        plt.fill(x, y, 'red', alpha=1)
    elif isinstance(holes, MultiPolygon):
        for poly in holes.geoms:
            x, y = poly.exterior.xy
            #plt.plot(x, y, 'k-')
            plt.fill(x, y, 'red', alpha=1)
    
    # Plot interpolated points in yellow if provided

    if isinstance(interpolated_points, Polygon):
        x, y = interpolated_points.exterior.xy
        #plt.plot(x, y, 'k-')
        plt.fill(x, y, 'yellow', alpha=1)
    elif isinstance(interpolated_points, MultiPolygon):
        for poly in interpolated_points.geoms:
            x, y = poly.exterior.xy
            #plt.plot(x, y, 'k-')
            plt.fill(x, y, 'yellow', alpha=1)

    plt.title('Mowed area (blue) × Not mowed area (red)')
    ax.set_aspect('equal', 'box')
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
    
    if save:
        plt.savefig(filename, bbox_inches='tight')
        plt.close(fig)  # Close the figure to free memory
    else:
        plt.tight_layout()
        plt.show()

def plot_points(tachy_df, DAQ_df = None, big = 0, save: bool = False, filename = "tachy_DAQ_Plot.png"):
    if big == 0: #Plot big image only when requested
        fig, ax = plt.subplots()
    else:
        fig, ax = plt.subplots(figsize = (10,10), dpi=1200)

    ax.scatter(tachy_df["x_data"], tachy_df["y_data"], c='b', s=1, label='Tachy') # Plot tachy DATA

    if isinstance(DAQ_df, pd.DataFrame):  # Check if DAQ_df is a DataFrame
        ax.scatter(DAQ_df["x_data"], DAQ_df["y_data"], c='r', s=1, label='DAQ')  # Plot DAQ data

    plt.legend(loc="upper left")
    ax.set_aspect('equal', 'box')
    # if requested, save the file
    if save == True:
        plt.savefig(filename, bbox_inches='tight')

def plot_points_3d(tachy_df, DAQ_df=None, big=0, save=False, filename="tachy_DAQ_3D_Plot.png", elev=30, azim=45, interactive=True, z_scale=1.0):
    """
    Create a 3D scatter plot of tachy and DAQ data.
    
    Parameters:
    tachy_df: DataFrame containing tachy data (must include x_data, y_data, z_data)
    DAQ_df: Optional DataFrame containing DAQ data (must include x_data, y_data, z_data if provided)
    big: If non-zero, creates a larger high-resolution plot
    save: Whether to save the plot to a file
    filename: Filename to save the plot (if save is True)
    elev, azim: Elevation and azimuth viewing angles for the 3D plot
    interactive: If True, enables interactive rotation with mouse (default: True)
    z_scale: Scaling factor for Z-axis to accentuate inclinations (default: 1.0, smaller values exaggerate height)
    """
    
    
    # Check if z_data exists in the dataframes
    if "z_data" not in tachy_df.columns:
        raise ValueError("tachy_df must contain a z_data column for 3D plotting")
    
    if isinstance(DAQ_df, pd.DataFrame) and "z_data" not in DAQ_df.columns:
        raise ValueError("DAQ_df must contain a z_data column for 3D plotting")
    
    # Create figure with appropriate size
    if big == 0:
        fig = plt.figure()
    else:
        fig = plt.figure(figsize=(10, 10), dpi=1200)
    
    # Create 3D axes
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot tachy data with scaled Z values
    ax.scatter(
        tachy_df["x_data"], 
        tachy_df["y_data"], 
        tachy_df["z_data"] * z_scale,  # Apply scaling to Z values
        c='b', 
        s=1, 
        label='Tachy'
    )
    
    # Plot DAQ data if provided
    if isinstance(DAQ_df, pd.DataFrame):
        ax.scatter(
            DAQ_df["x_data"], 
            DAQ_df["y_data"], 
            DAQ_df["z_data"] * z_scale,  # Apply scaling to Z values
            c='r', 
            s=1, 
            label='DAQ'
        )
    
    # Set the viewing angle
    ax.view_init(elev=elev, azim=azim)
    
    # Labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend(loc="upper left")
    
    # Set equal aspect ratio
    # Note: equal aspect ratio in 3D is more complex than in 2D
    # This is a reasonable approximation
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    
    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    
    max_range = max(x_range, y_range, z_range)
    
    x_middle = (x_limits[1] + x_limits[0]) / 2
    y_middle = (y_limits[1] + y_limits[0]) / 2
    z_middle = (z_limits[1] + z_limits[0]) / 2
    
    ax.set_xlim3d([x_middle - max_range/2, x_middle + max_range/2])
    ax.set_ylim3d([y_middle - max_range/2, y_middle + max_range/2])
    ax.set_zlim3d([z_middle - max_range/2, z_middle + max_range/2])
    
    # Enable interactive mode if requested
    if interactive:
        plt.tight_layout()
        
        # Add a title with instructions
        ax.set_title("Click and drag to rotate the view")
        
        # Enable tight layout for better spacing
        plt.tight_layout()
        
    # Save figure if requested
    if save:
        plt.savefig(filename, bbox_inches='tight')
    
    return fig, ax

def enable_interactive_3d_plots():
    """
    Enable interactive 3D plots for Jupyter notebooks.
    Call this function before creating 3D plots to enable mouse rotation.
    """
    try:
        # Try to get IPython instance - will succeed in Jupyter
        ipython = get_ipython()
        
        # Switch to interactive Qt backend
        ipython.run_line_magic('matplotlib', 'qt')
        print("Interactive 3D plotting enabled. Plots will open in separate windows.")
        print("Click and drag to rotate the view.")
        
    except (ImportError, NameError):
        # If not in Jupyter, use a different approach
        
        matplotlib.use('TkAgg')  # Or another interactive backend
        print("Using TkAgg backend for interactive plotting.")

def disable_interactive_3d_plots():
    """
    Switch back to inline plotting for Jupyter notebooks.
    Call this when you're done with interactive 3D plots.
    """
    try:
        # Try to get IPython instance
        ipython = get_ipython()
        
        # Switch back to inline plotting
        ipython.run_line_magic('matplotlib', 'inline')
        print("Switched back to inline plotting.")
        
    except (ImportError, NameError):
        # If not in Jupyter, don't do anything
        pass

def create_tachy_df(filename, remove_tachy_point=True, drop=False, rotate=0, include_z=False):
    """
    Create a pandas DataFrame from the XML file data with option to include Z-axis data.
    
    Parameters:
    filename: Path to the XML file
    remove_tachy_point: Whether to remove the last point (default: True)
    drop: Number of points to drop from the beginning (default: False)
    rotate: Rotation angle in degrees (default: 0)
    include_z: Whether to include Z coordinates in the DataFrame (default: False)
    
    Returns:
    pandas DataFrame with x, y, (z), timestamp data
    """
    if include_z:
        x_data, y_data, z_data, timestamp = load_xml(filename, rotate=rotate, mirror=False, 
                                                   remove_tachy_point=remove_tachy_point, 
                                                   drop_points=drop, include_z=True)
    else:
        x_data, y_data, timestamp = load_xml(filename, rotate=rotate, mirror=False, 
                                           remove_tachy_point=remove_tachy_point, 
                                           drop_points=drop, include_z=False)

    timestamp_local = pd.to_datetime(timestamp)
    unix_timestamp = (timestamp_local.astype(int) / 10**9).astype(int)
    unix_timestamp_norm = unix_timestamp - np.min(unix_timestamp) + 1

    # Create DataFrame with or without z_data based on include_z parameter
    if include_z:
        df = pd.DataFrame({
            "x_data": x_data,
            "y_data": y_data,
            "z_data": z_data,
            "date_time": timestamp,
            "unix_timestamp": unix_timestamp,
            "unix_timestamp_norm": unix_timestamp_norm
        })
    else:
        df = pd.DataFrame({
            "x_data": x_data,
            "y_data": y_data,
            "date_time": timestamp,
            "unix_timestamp": unix_timestamp,
            "unix_timestamp_norm": unix_timestamp_norm
        })

    df = df.sort_values(by='date_time', ascending=True)
    df = df.reset_index()
    df = df.drop(columns=['index'])

    return df

def mirror_df(df):
    x_mean = df['x_data'].mean()
    y_mean = df['y_data'].mean()
    
    # Mirror the data around the mean values
    df['x_data'] = 2 * x_mean - df['x_data']
    df['y_data'] = 2 * y_mean - df['y_data']
    
    return df

def convert_enue_enun(df, theta=0):
    theta_rad = np.radians(theta)
    df['x_data'] = df['ENUE'] * np.cos(theta_rad) - df['ENUN'] * np.sin(theta_rad)
    df['y_data'] = df['ENUE'] * np.sin(theta_rad) + df['ENUN'] * np.cos(theta_rad)
    return df

def rotate_df(df, angle_degrees):
    # Convert angle from degrees to radians
    angle_radians = np.radians(angle_degrees)

    x_data = df['x_data'].values
    y_data = df['y_data'].values

    # Create rotation matrix
    rotation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians)],
        [np.sin(angle_radians), np.cos(angle_radians)]
    ])
    
    # Stack x_data and y_data into a 2D array
    data = np.vstack((x_data, y_data))
    
    # Apply rotation
    rotated_data = rotation_matrix.dot(data)
    
    df['x_data'] = rotated_data[0, :]
    df['y_data'] = rotated_data[1, :]

    return df

def calculate_inclination(df):
    # Get the x and y values from the DataFrame
    x = df['x_data']
    y = df['y_data']
    
    # Calculate the slope (m) of the line using linear regression
    m, _ = np.polyfit(x, y, 1)
    
    # Calculate the inclination angle in degrees
    inclination = np.degrees(np.arctan(m))
    
    return inclination

def manual_procrustes(A, B):
    # Center the data
    A_mean = A.mean(axis=0)
    B_mean = B.mean(axis=0)
    A_centered = A - A_mean
    B_centered = B - B_mean

    # Compute the optimal rotation matrix using Singular Value Decomposition (SVD)
    U, S, Vt = np.linalg.svd(np.dot(B_centered.T, A_centered))
    R = np.dot(U, Vt)

    # Apply the rotation to B
    B_rotated = np.dot(B_centered, R)

    # Compute the scaling factor
    scale = np.sum(S) / np.sum(B_centered ** 2)

    # Apply the scaling
    B_transformed = scale * B_rotated

    # Translate back to the original mean
    B_transformed += A_mean

    # Calculate the disparity
    disparity = np.sum((A - B_transformed) ** 2)

    return B_transformed, R, scale, disparity

def teach_in_fix(meas_df, border_df):
    A = meas_df[['x_data', 'y_data']].values
    B = border_df[['x_data', 'y_data']].values
    
    # Check if A and B are not empty
    if A.size == 0 or B.size == 0:
        raise ValueError("Input arrays must not be empty")
    
    # Pad the smaller array with zeroes
    if A.shape[0] > B.shape[0]:
        padding = np.zeros((A.shape[0] - B.shape[0], B.shape[1]))
        B = np.vstack([B, padding])
    elif B.shape[0] > A.shape[0]:
        padding = np.zeros((B.shape[0] - A.shape[0], A.shape[1]))
        A = np.vstack([A, padding])
    
    # Perform Procrustes analysis
        
    
    # Create aligned dataframes
    x_data = A[:, 0]
    y_data = A[:, 1]
    x_hull = B_transformed[:, 0]
    y_hull = B_transformed[:, 1]

    return x_data, y_data, x_hull, y_hull

def fit_measurements(tachy_df, meas_df):
    # Merge the dataframes on 'unix_timestamp'
    merged_df = pd.merge(tachy_df, meas_df, on='unix_timestamp', suffixes=('_tachy', '_meas'))
    
    # Filter out rows where any of the coordinates are zero
    merged_df = merged_df[(merged_df['x_data_tachy'] != 0) & (merged_df['y_data_tachy'] != 0) & 
                          (merged_df['x_data_meas'] != 0) & (merged_df['y_data_meas'] != 0)]
    merged_df = merged_df.reset_index(drop=True)

    # Create dataframes for matching timestamps
    meas_matching_timestamp = pd.DataFrame({
        "x_data": merged_df['x_data_meas'],
        "y_data": merged_df['y_data_meas'],
        "unix_timestamp": merged_df['unix_timestamp'],
    })
    tachy_matching_timestamp = pd.DataFrame({
        "x_data": merged_df['x_data_tachy'],
        "y_data": merged_df['y_data_tachy'],
        "unix_timestamp": merged_df['unix_timestamp'],
    })

    # Extract coordinates as numpy arrays
    A = tachy_matching_timestamp[['x_data', 'y_data']].values
    B = meas_matching_timestamp[['x_data', 'y_data']].values

    # Check if A and B are not empty
    if A.size == 0 or B.size == 0:
        raise ValueError("Input arrays must not be empty")

    # Perform Procrustes analysis
    B_transformed, R, scale, disparity = manual_procrustes(A, B)

    # Create aligned dataframes
    tachy_aligned = pd.DataFrame(A, columns=['x_data', 'y_data'])
    tachy_aligned['unix_timestamp'] = tachy_matching_timestamp['unix_timestamp']
    meas_aligned = pd.DataFrame(B_transformed, columns=['x_data', 'y_data'])
    meas_aligned['unix_timestamp'] = meas_matching_timestamp['unix_timestamp']

    return tachy_aligned, meas_aligned, disparity

def create_daq_df(filename):

    daq_csv_df = pd.read_csv(filename)
    try:
        daq_df = pd.DataFrame({
        "ENUE": daq_csv_df['BAL2RAL.PosAplIn.posRtk.posEast[m]'],
        "ENUN": daq_csv_df['BAL2RAL.PosAplIn.posRtk.posNorth[m]'],
        "unix_timestamp": daq_csv_df['Meas.timSystem[s]']+946692000, #946692000 to convert iMOW timestamp to unix timestamp with CET
        })
    except:
        daq_df = pd.DataFrame({
        "ENUE": daq_csv_df['BAL2RAL.PosAplIn.posRtk.posEast'],
        "ENUN": daq_csv_df['BAL2RAL.PosAplIn.posRtk.posNorth'],
        "unix_timestamp": daq_csv_df['Meas.timSystem']+946692000, #946692000 to convert iMOW timestamp to unix timestamp with CET
        })
    daq_df["unix_timestamp_norm"] = daq_df['unix_timestamp'] - daq_df['unix_timestamp'].min() +1
    daq_df = convert_enue_enun(daq_df,0)

    return daq_df

def crop_end(df, df_outside, standing_clusters, timestamp):
    # Ensure timestamp is timezone-aware
    # timestamp = pd.to_datetime(timestamp).tz_localize('GMT')

    # Create conditions
    condition_df = pd.to_datetime(df['date_time']).dt.tz_localize('GMT') >= timestamp
    condition_outside = pd.to_datetime(df_outside['date_time']).dt.tz_localize('GMT') >= timestamp
    condition_standing = standing_clusters['start_standing'] >= timestamp

    # Drop rows based on conditions
    df = df.drop(df[condition_df].index)
    df_outside = df_outside.drop(df_outside[condition_outside].index)
    standing_clusters = standing_clusters.drop(standing_clusters[condition_standing].index)

    return df, df_outside, standing_clusters

def crop_beginning(df, df_outside, standing_clusters, timestamp):
    # Ensure timestamp is timezone-aware
    # timestamp = pd.to_datetime(timestamp).tz_localize('GMT')

    # Create conditions
    condition_df = pd.to_datetime(df['date_time']).dt.tz_localize('GMT') < timestamp
    condition_outside = pd.to_datetime(df_outside['date_time']).dt.tz_localize('GMT') < timestamp
    condition_standing = standing_clusters['start_standing'] < timestamp

    # Drop rows based on conditions
    df = df.drop(df[condition_df].index).reset_index(drop=True)
    df_outside = df_outside.drop(df_outside[condition_outside].index).reset_index(drop=True)
    standing_clusters = standing_clusters.drop(standing_clusters[condition_standing].index).reset_index(drop=True)

    return df, df_outside, standing_clusters

def create_hull_bad_data(x_data, y_data, closed=True, min_area=0.1):
    points = [(x, y) for x, y in zip(x_data, y_data)]
    hull_indices = ConvexHull(points).vertices
    hull_points = [points[i] for i in hull_indices]
    
    if closed:
        hull = Polygon(hull_points)
        if isinstance(hull, Polygon):
            return hull if hull.area > min_area else None
        elif isinstance(hull, MultiPolygon):
            largest_poly = max(hull.geoms, key=lambda poly: poly.area)
            return largest_poly if largest_poly.area > min_area else None
    else:
        hull = LineString(hull_points)
        print("Hull not closed.")
        return hull
    
def create_smaller_hull(x_data, y_data, shrink_factor=0.1, closed=True, min_area=0.0001):
    points = [(x, y) for x, y in zip(x_data, y_data)]
    
    if closed:
        hull = Polygon(points)
        if isinstance(hull, Polygon):
            if hull.area > min_area:
                # Calculate buffer distance based on square root of area
                # This helps maintain shape proportions better
                buffer_distance = -1 * shrink_factor * (hull.area ** 0.5)
                
                # Use more segments and mitered joins to preserve shape
                smaller_hull = hull.buffer(buffer_distance, 
                                        join_style=1,  # Mitered join
                                        cap_style=1,   # Round cap
                                        resolution=16)  # More segments for smoother result
                
                if isinstance(smaller_hull, MultiPolygon):
                    smaller_hull = unary_union(smaller_hull)
                    if isinstance(smaller_hull, MultiPolygon):
                        smaller_hull = max(smaller_hull.geoms, key=lambda p: p.area)
                
                return smaller_hull
                
        elif isinstance(hull, MultiPolygon):
            largest_poly = max(hull.geoms, key=lambda poly: poly.area)
            if largest_poly.area > min_area:
                buffer_distance = -1 * shrink_factor * (largest_poly.area ** 0.5)
                
                smaller_hull = largest_poly.buffer(buffer_distance,
                                                 join_style=1,
                                                 cap_style=1,
                                                 resolution=16)
                
                if isinstance(smaller_hull, MultiPolygon):
                    smaller_hull = max(smaller_hull.geoms, key=lambda p: p.area)
                    
                return smaller_hull
        return None
    else:
        hull = LineString(points)
        print("Hull not closed.")
        return hull
    
def plot_hulls(hull,small_hull, plot_points = False, df_points=0):
    fig, ax = plt.subplots(figsize = (10,10), dpi=1200) #BIG PICTURE
    #fig, ax = plt.subplots() # SMALL PICTURE

    if isinstance(hull, Polygon):
        x_hull, y_hull = hull.exterior.xy
        plt.plot(x_hull, y_hull, 'k-')
        plt.fill(x_hull, y_hull, 'lightgrey', alpha=0.5)
    elif isinstance(hull, MultiPolygon):
        for poly in hull.geoms:
            x_hull, y_hull = poly.exterior.xy
            plt.plot(x_hull, y_hull, 'k-')
            plt.fill(x_hull, y_hull, 'lightgrey', alpha=0.5)

    if isinstance(small_hull, Polygon):
        x_hull, y_hull = small_hull.exterior.xy
        plt.plot(x_hull, y_hull, 'r-')
        plt.fill(x_hull, y_hull, 'blue', alpha=0.5)
    elif isinstance(small_hull, MultiPolygon):
        for poly in small_hull.geoms:
            x_hull, y_hull = poly.exterior.xy
            plt.plot(x_hull, y_hull, 'r-')
            plt.fill(x_hull, y_hull, 'blue', alpha=0.5)

    if plot_points==True:
        ax.scatter(df_points['x_data'], df_points['y_data'], s=1)
    
    ax.set_aspect('equal', 'box')
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
    plt.show()

def remove_points_outside_hull_df(df, hull):
    """
    Filter DataFrame to keep only rows where points are inside the hull.
    Uses vectorized operations for better performance with large datasets.
    
    Parameters:
    -----------
    df: pd.DataFrame
        DataFrame with columns 'x_data' and 'y_data'
    hull: shapely.geometry.Polygon or MultiPolygon
        The boundary hull
        
    Returns:
    --------
    pd.DataFrame
        Filtered DataFrame with only points inside the hull, with reset index
    """
    # Use prepared geometry for faster contains operations
    prepared_hull = prep(hull)
    
    # Create a vectorized test function
    def points_inside(chunk):
        points = [Point(x, y) for x, y in zip(chunk['x_data'], chunk['y_data'])]
        return [prepared_hull.contains(p) for p in points]
    
    # Process in chunks for large dataframes
    if len(df) > 10000:
        chunk_size = 10000
        mask = []
        for i in range(0, len(df), chunk_size):
            chunk = df.iloc[i:i+chunk_size]
            mask.extend(points_inside(chunk))
    else:
        mask = points_inside(df)
    
    # Filter DataFrame and reset index
    filtered_df = df[mask].reset_index(drop=True)
    
    return filtered_df

def split_continuous_segments(df, max_time_gap=1.0, min_distance=1.0, radians=False):
    """
    Split DataFrame into continuous time segments and filter by distance traveled
    
    Args:
        df: DataFrame with coordinates and timestamps
        max_time_gap: maximum allowed time gap between points
        min_distance: minimum distance between first and last point
    """
    df = df.sort_values('date_time').reset_index(drop=True)
    time_diffs = df['unix_timestamp_norm'].diff()
    break_points = np.where(time_diffs > max_time_gap)[0]
    
    segments = []
    start_idx = 0
    
    for end_idx in break_points:
        segment = df.iloc[start_idx:end_idx].reset_index(drop=True)
        
        # Calculate distance between first and last point
        start_point = np.array([segment['x_data'].iloc[0], segment['y_data'].iloc[0]])
        end_point = np.array([segment['x_data'].iloc[-1], segment['y_data'].iloc[-1]])
        distance = np.linalg.norm(end_point - start_point)
        
        if distance >= min_distance:
            segments.append(segment)
        start_idx = end_idx
    
    # Handle last segment
    last_segment = df.iloc[start_idx:].reset_index(drop=True)
    if len(last_segment) > 0:
        start_point = np.array([last_segment['x_data'].iloc[0], last_segment['y_data'].iloc[0]])
        end_point = np.array([last_segment['x_data'].iloc[-1], last_segment['y_data'].iloc[-1]])
        distance = np.linalg.norm(end_point - start_point)
        
        if distance >= min_distance:
            segments.append(last_segment)

    for i, segment in enumerate(segments): # Apply interpolation
        segments[i] = interpolate_segments(segment, radians=radians)
    
    return segments

def generate_line_results(continuous_segments, filename="./results.csv"):
    means = []
    inclinations = []
    r2_scores = []
    slopes = []
    angle_ratios = []

    for i in range(len(continuous_segments)-1):
        angle1 = continuous_segments[i]['inclination'].iloc[0]
        angle2 = continuous_segments[i+1]['inclination'].iloc[0]
        
        # Avoid division by zero by checking slopes
        if angle2 != 0:
            ratio = angle1/angle2
        else:
            continue
            
        angle_ratios.append(ratio)

    results = pd.DataFrame()
    means = [seg['distance_to_line'].mean() for seg in continuous_segments] #Mean distance between any given point on the real line to the interpolated line
    inclinations = [seg['inclination'].iloc[0] for seg in continuous_segments] #Angle of the interpolated line
    r2_scores = [seg['r2'].iloc[0] for seg in continuous_segments] #R2 score of the interpolated line
    slopes = [seg['slope'].iloc[0] for seg in continuous_segments] #Y = mx+b.    Slope = m


    # Add statistics to results
    results.loc['Mean distance', 'Value'] = np.mean(means) # Mean distance between the mean distance of the real data to the interpolated line "How far the mower deviate from a perfect line"
    results.loc['Max distance', 'Value'] = np.max([seg['distance_to_line'].max() for seg in continuous_segments]) # Mean max distance between the real data to the interpolated line "How far the mower deviate from a perfect line"
    results.loc['Min distance', 'Value'] = np.min([seg['distance_to_line'].min() for seg in continuous_segments]) # Mean min distance between the real data to the interpolated line "How far the mower deviate from a perfect line"
    results.loc['Std distance', 'Value'] = np.std(means) #Std deviation of the Mean distance between the real data to the interpolated line "How far the mower deviate from a perfect line"
    results.loc['Mean inclination', 'Value'] = np.mean(inclinations) #Mean angle of interpolated lines
    results.loc['Std inclination', 'Value'] = np.std(inclinations) #Mean Std deviation of interpolated lines
    results.loc['Mean R²', 'Value'] = np.mean(r2_scores) #Mean R2 score of interpolated lines
    results.loc['Mean slope', 'Value'] = np.mean(slopes) #Mean slop of interpolated lines
    results.loc['Number of segments', 'Value'] = len(continuous_segments) #How many lines

    # Add worst segment info
    max_segment_idx = np.argmax(means)
    results.loc['Worst segment index', 'Value'] = max_segment_idx #Which line is the worst
    results.loc['Worst segment mean distance', 'Value'] = means[max_segment_idx] #Mean dist between real data and the interpolated line for worst case
    results.loc['Worst segment inclination', 'Value'] = inclinations[max_segment_idx] #inclination of interpolated line for worst case
    results.loc['Worst segment R²', 'Value'] = r2_scores[max_segment_idx] #R2 for worst iterpolated line

    # Add parallelism statistics to results
    results.loc['Mean angle ratio', 'Value'] = np.mean(angle_ratios) #Mean angle of interpolated lines
    results.loc['Max angle ratio', 'Value'] = np.max(angle_ratios)#Max angle of interpolated lines
    results.loc['Min angle ratio', 'Value'] = np.min(angle_ratios)#Min angle of interpolated lines
    results.loc['Std angle ratio', 'Value'] = np.std(angle_ratios)#Std deviation angle of interpolated lines

    # Find segments with most parallel lines (ratio closest to 1)
    parallelism_scores = np.abs(np.array(angle_ratios) - 1)
    most_parallel_idx = np.argmin(parallelism_scores)
    results.loc['Most parallel segments', 'Value'] = f"{most_parallel_idx} and {most_parallel_idx+1}" #Get the two lines that have the most similar angles
    results.loc['Most parallel ratio', 'Value'] = angle_ratios[most_parallel_idx] #How similar this angles are

    # Find least parallel segments
    least_parallel_idx = np.argmax(parallelism_scores)
    results.loc['Least parallel segments', 'Value'] = f"{least_parallel_idx} and {least_parallel_idx+1}" #Get the two lines that have the worst similar angles
    results.loc['Least parallel ratio', 'Value'] = angle_ratios[least_parallel_idx] #How similar this angles are

    # Round all values for cleaner display
    results = results.round(4)

    results.to_csv(filename,sep=';')

    return results

def plot_angle_distribution(continuous_segments, filename,radians = False):
    plt.figure(figsize=(15, 8))

    # Create segment indices for x-axis
    segment_indices = range(len(continuous_segments))

    # Get inclinations
    inclinations = [segment['inclination'].iloc[0] for segment in continuous_segments]
    r2_scores = [segment['r2'].iloc[0] for segment in continuous_segments]

    # Create scatter plot with color based on R² score
    scatter = plt.scatter(segment_indices, inclinations, c=r2_scores, cmap='viridis', 
                        s=100, alpha=0.6)

    # Add colorbar
    cbar = plt.colorbar(scatter)
    cbar.set_label('R² Score')

    # Add mean line
    mean_inclination = np.mean(inclinations)
    plt.axhline(y=mean_inclination, color='r', linestyle='--', 
                label=f'Mean Inclination: {mean_inclination:.2f}°')

    plt.title('Segment Inclinations')
    plt.xlabel('Segment Index')
    if radians:
        plt.ylabel('Inclination (Radians)')
    else:
        plt.ylabel('Inclination (degrees)')
    plt.grid(True, alpha=0.3)
    plt.legend()

    # Add value annotations
    if radians:
        for i, (inc, r2) in enumerate(zip(inclinations, r2_scores)):
            plt.annotate(f'{inc:.1f}', 
                        (i, inc), 
                        xytext=(0, 10), 
                        textcoords='offset points', 
                        ha='center')
    else:
        for i, (inc, r2) in enumerate(zip(inclinations, r2_scores)):
            plt.annotate(f'{inc:.1f}°', 
                        (i, inc), 
                        xytext=(0, 10), 
                        textcoords='offset points', 
                        ha='center')

    plt.savefig(filename+'.png')
    plt.show()
    
def add_distances_to_line_first_last(df_line):
    """
    Add perpendicular distances from each point to line as new column
    
    Args:
        df_line: DataFrame with x_data, y_data columns representing one segment
    Returns:
        DataFrame with new 'distance_to_line' column
    """
    # Get first and last points
    p1 = np.array([df_line['x_data'].iloc[0], df_line['y_data'].iloc[0]])
    p2 = np.array([df_line['x_data'].iloc[-1], df_line['y_data'].iloc[-1]])
    
    # Create line vector
    line_vec = p2 - p1
    line_length = np.linalg.norm(line_vec)
    line_unit_vec = line_vec / line_length
    
    # Get all points
    points = np.column_stack([df_line['x_data'], df_line['y_data']])
    
    # Calculate distances
    vec_to_points = points - p1
    proj_length = np.dot(vec_to_points, line_unit_vec)
    proj_vec = np.outer(proj_length, line_unit_vec)
    perp_vec = vec_to_points - proj_vec
    distances = np.linalg.norm(perp_vec, axis=1)
    
    # Add distances as new column
    df_line_with_distances = df_line.copy()
    df_line_with_distances['distance_to_line'] = distances
    
    return df_line_with_distances

def plot_segment_with_line(segment, save_location = False):
    # Create figure
    plt.figure(figsize=(12, 8))
    
    # Plot actual points
    plt.scatter(segment['x_data'], segment['y_data'], c='blue', s=20, alpha=0.6, label='Actual Path')
    
    # Plot the expected straight line
    start_point = [segment['x_data'].iloc[0], segment['y_data'].iloc[0]]
    end_point = [segment['x_data'].iloc[-1], segment['y_data'].iloc[-1]]
    plt.plot([start_point[0], end_point[0]], 
             [start_point[1], end_point[1]], 
             'r--', linewidth=2, label='Expected Line')
    
    # Highlight start and end points
    plt.scatter([start_point[0]], [start_point[1]], c='green', s=100, label='Start')
    plt.scatter([end_point[0]], [end_point[1]], c='red', s=100, label='End')
    
    plt.title(f'Segment Path vs Expected Line\nAverage Distance: {segment["distance_to_line"].mean():.2f}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    if save_location != False:
        plt.savefig(save_location+'.png')
        return
    plt.show()

def interpolate_segments(df_line, radians= False):
    """
    Add distances, interpolated points, and inclination analytics from best fit line
    """
    # Get points
    x = df_line['x_data'].values
    y = df_line['y_data'].values
    
    # Calculate best fit line
    coeffs = np.polyfit(x, y, 1)
    m, b = coeffs

    # Calculate inclination in degrees
    if radians == False:
        inclination = np.arctan(m) * 180 / np.pi
        if inclination < 0:
            inclination += 360
    else:
        inclination = np.arctan(m)
        if inclination < 0:
            inclination += np.pi
    
    # Calculate interpolated points and distances
    x_interpolated = x
    y_interpolated = m * x + b
    distances = np.abs(y - y_interpolated) / np.sqrt(1 + m**2)
    
    # Enhanced dataframe with all metrics
    df_line_with_distances = df_line.copy()
    df_line_with_distances['distance_to_line'] = distances
    df_line_with_distances['x_interpolated'] = x_interpolated
    df_line_with_distances['y_interpolated'] = y_interpolated
    df_line_with_distances['inclination'] = inclination
    df_line_with_distances['slope'] = m
    df_line_with_distances['intercept'] = b
    df_line_with_distances['r2'] = r2_score(y, y_interpolated)
    
    return df_line_with_distances

def plot_segment_with_interpolated(segment, save_location = False):
    plt.figure(figsize=(12, 8))
    plt.scatter(segment['x_data'], segment['y_data'], c='blue', s=20, alpha=0.6, label='Actual Path')
    plt.plot(segment['x_interpolated'], segment['y_interpolated'], 'r--', linewidth=2, label='Best Fit Line')
    plt.scatter(segment['x_data'].iloc[0], segment['y_data'].iloc[0], c='green', s=100, label='Start')
    plt.scatter(segment['x_data'].iloc[-1], segment['y_data'].iloc[-1], c='red', s=100, label='End')
    plt.title(f'Segment Path vs Best Fit Line\nAverage Distance: {segment["distance_to_line"].mean():.2f} \nR²: {segment["r2"].mean():.5f}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    if save_location != False:
        plt.savefig(save_location+'.png',)
        return
    plt.show()

def filter_segments_by_r2(segments, r2_threshold=0.8):
    """
    Filter segments based on R² score threshold
    """
    filtered_segments = []
    dropped_indices = []
    
    for i, segment in enumerate(segments):
        if segment['r2'].iloc[0] >= r2_threshold:
            filtered_segments.append(segment)
        else:
            dropped_indices.append(i)
    
    print(f"Dropped {len(dropped_indices)} segments with R² < {r2_threshold} from a total of {len(segments)}")
    if dropped_indices:
        print(f"Dropped segment indices: {dropped_indices}")
    
    return filtered_segments  

def filter_segments_by_inclination(segments, max_std=2.0):
    """
    Filter segments with inclinations outside a given number of standard deviations from mean
    
    Parameters:
    -----------
    segments : list
        List of DataFrame segments to filter
    max_std : float, default=2.0
        Maximum number of standard deviations from mean to include
        
    Returns:
    --------
    list
        Filtered list of segments within the inclination threshold
    """
    # Get inclinations
    inclinations = [segment['inclination'].iloc[0] for segment in segments]
    
    # Calculate mean and std
    mean_inc = np.mean(inclinations)
    std_inc = np.std(inclinations)
    
    # Define thresholds
    upper_threshold = mean_inc + max_std*std_inc
    lower_threshold = mean_inc - max_std*std_inc
    
    # Filter segments
    filtered_segments = []
    dropped_indices = []
    
    for i, segment in enumerate(segments):
        inc = segment['inclination'].iloc[0]
        if lower_threshold <= inc <= upper_threshold:
            filtered_segments.append(segment)
        else:
            dropped_indices.append(i)
    
    print(f"Mean inclination: {mean_inc:.2f}°")
    print(f"Standard deviation: {std_inc:.2f}°")
    print(f"Keeping segments with inclinations between {lower_threshold:.2f}° and {upper_threshold:.2f}°")
    print(f"Dropped {len(dropped_indices)} segments with indices: {dropped_indices}")
    
    return filtered_segments

def plot_hull_with_grid(hull, inclination, df_points, grid_size=200, spacing=0.1):
    plt.figure(figsize=(15, 15))

    # Plot hull boundary
    x, y = hull.exterior.xy
    plt.plot(x, y, 'k-', linewidth=2)
    minx, miny, maxx, maxy = hull.bounds
    angle = np.deg2rad(inclination)
    for i in range(grid_size):
        # Offset each line by spacing
        offset = i * spacing

        # Start point of each line
        x1 = offset
        y1 = miny -2

        # End point
        x2 = x1 + 1.3*(maxx-minx) * np.cos(angle)
        y2 = y1 + 1.3*(maxy-miny) * np.sin(angle)

        # Plot line
        plt.plot([x1, x2], [y1, y2], 'r-', linewidth=1)
    plt.scatter(df_points['x_data'], df_points['y_data'], s=1)
    plt.axis('equal')
    plt.title(f'Hull with Grid (Inclination: {inclination:.2f}°)')
    plt.show()

def calculate_mowed_area(df, hull, radius=0.15, step_size=10, save=True, filename="mowed_pertime.png"):
    """
    Calculate and track the mowed area over time.
    
    Parameters:
    -----------
    df : pd.DataFrame
        DataFrame containing robot tracking data
    hull : shapely.geometry.Polygon
        The boundary hull representing the total area
    radius : float, default=0.15
        Radius of the mower's effective cutting area
    step_size : int, default=10
        Number of points to process in each step
    save : bool, default=True
        Whether to save the plot to a file
    filename : str, default="mowed_pertime.png"
        Filename to save the plot if save=True
        
    Returns:
    --------
    pd.DataFrame
        DataFrame containing mowed area data over time
    """
    # Sort data by timestamp
    df.sort_values(by='unix_timestamp_norm', ascending=True, inplace=True)
    df.reset_index(drop=True, inplace=True)
    
    # Ensure hull is valid and calculate total area
    hull = make_valid(hull)
    hull_area = hull.area

    combined_union = None
    df_mowed = pd.DataFrame()

    # Process data in steps to track mowed area over time
    for i in range(0, len(df), step_size):
        df_slice = df.iloc[i:i + step_size]
        circles = [Point(x, y).buffer(radius) for x, y in zip(df_slice['x_data'], df_slice['y_data'])]
        
        if combined_union is None:
            combined_union = unary_union(circles)
        else:
            combined_union = combined_union.union(unary_union(circles))
        
        # Calculate intersection of mowed area with hull
        mowed_area = hull.intersection(combined_union)

        # Store results
        df_mowed.loc[i // step_size, 'total_mowed_area'] = mowed_area.area
        df_mowed.loc[i // step_size, 'unix_timestamp_norm'] = df_slice['unix_timestamp_norm'].iloc[-1]
        
        # Show progress for long-running calculations
        if i % (10 * step_size) == 0:
            print(f"Progress: {i / len(df):.2%}")

    # Calculate percentage of mowed area
    df_mowed['percentage_mowed_area'] = df_mowed['total_mowed_area'] / hull_area
    df_mowed['timestamp_minutes'] = df_mowed['unix_timestamp_norm']/60
    
    # Create plot
    plt.figure(figsize=(10, 6))
    ax = df_mowed.plot(x='timestamp_minutes', y='percentage_mowed_area', kind='line', 
                     figsize=(10, 6), title='Mowed Area Over Time')
    plt.xlabel('Time (minutes)')
    plt.ylabel('Percentage of Area Mowed')
    plt.grid(True, alpha=0.3)

    # Save or show plot
    if save:
        plt.savefig(filename, bbox_inches='tight')
    else:
        plt.tight_layout()
        plt.show()
        
    return df_mowed

def create_3d_surface(x, y, z, resolution=100, plot_original_points=True):
    """
    Create a 3D surface plot from scattered data points.
    
    Parameters:
    -----------
    x, y, z : array-like
        The coordinates of the data points
    resolution : int
        Resolution of the grid for interpolation
    plot_original_points : bool
        Whether to plot the original data points
        
    Returns:
    --------
    fig, ax : matplotlib figure and axes objects
    X, Y, Z : numpy arrays containing the interpolated surface data
    """
    # Create a grid for interpolation
    xi = np.linspace(min(x), max(x), resolution)
    yi = np.linspace(min(y), max(y), resolution)
    X, Y = np.meshgrid(xi, yi)
    
    # Interpolate z values on the grid
    Z = griddata((x, y), z, (X, Y), method='cubic')
    
    # Create the 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the surface
    surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm, alpha=0.8,
                          linewidth=0, antialiased=True)
    
    # Plot original points if requested
    if plot_original_points:
        ax.scatter(x, y, z, color='black', s=20)
    
    # Add labels and colorbar
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Surface from Data Points')
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)
    
    return fig, ax, X, Y, Z