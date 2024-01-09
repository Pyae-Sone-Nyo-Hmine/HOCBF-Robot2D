import matplotlib.pyplot as plt
import numpy as np

def grid(size,OBSTACLE_POSITIONS, DESTINATIONS, obstacles, destinations):
    
    # to run GUI event loop
    plt.ion()
    
    figure, ax = plt.subplots(figsize=(6, 6))

    
    plt.xlim(0, size)
    plt.ylim(0, size)
    ax.grid(True)
    plt.title("HOCBF Robot in 2D", fontsize=10)

    # Display obstacles on the grid
    for pos in OBSTACLE_POSITIONS:
        obstacle = plt.Circle(pos, 6, color='lightblue')
        obstacles.append(obstacle)
        ax.add_artist(obstacle)

    # Display destinations on the grid
    for dest in DESTINATIONS:
        destination = plt.Circle(dest, 0.8, color='green')
        destinations.append(destination)
        ax.add_artist(destination)
        ax.text(dest[0], dest[1], 'D', color='white', ha='center', va='center')

    return figure, ax
    

