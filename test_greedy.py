import math

# Dummy runs
all_runs = [
    {'y': 10, 'start': 0, 'end': 40},
    {'y': 10, 'start': 60, 'end': 100},
    {'y': 20, 'start': 0, 'end': 40},
    {'y': 20, 'start': 60, 'end': 100},
    {'y': 30, 'start': 0, 'end': 40},
    {'y': 30, 'start': 60, 'end': 100},
]

current_x = 0
current_y = 0
waypoints = []

while all_runs:
    best_dist = float('inf')
    best_run_idx = -1
    best_start_left = True
    
    for i, run in enumerate(all_runs):
        # Dist to left endpoint
        dist_left = math.hypot(run['start'] - current_x, run['y'] - current_y)
        # Dist to right endpoint
        dist_right = math.hypot(run['end'] - current_x, run['y'] - current_y)
        
        if dist_left < best_dist:
            best_dist = dist_left
            best_run_idx = i
            best_start_left = True
            
        if dist_right < best_dist:
            best_dist = dist_right
            best_run_idx = i
            best_start_left = False
            
    run = all_runs.pop(best_run_idx)
    
    if best_start_left:
        # Sweep left to right
        waypoints.append({'x': run['start'], 'y': run['y']})
        waypoints.append({'x': run['end'], 'y': run['y']})
        current_x = run['end']
        current_y = run['y']
    else:
        # Sweep right to left
        waypoints.append({'x': run['end'], 'y': run['y']})
        waypoints.append({'x': run['start'], 'y': run['y']})
        current_x = run['start']
        current_y = run['y']

for wp in waypoints:
    print(wp)
