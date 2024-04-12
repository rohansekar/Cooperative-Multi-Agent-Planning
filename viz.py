import pickle
import matplotlib.pyplot as plt

# Load the pickle file
with open('obstacles.pkl', 'rb') as file:
    obstacles = pickle.load(file)

# Prepare to plot the obstacles
print(obstacles)
fig, ax = plt.subplots()
x_coords, y_coords = zip(*obstacles)  # This separates the list of tuples into two lists of x and y coordinates

# Plot each obstacle as a point
ax.scatter(x_coords, y_coords, c='red', marker='o', label='Obstacles')

# Set plot labels and legend
ax.set_xlabel('X coordinates')
ax.set_ylabel('Y coordinates')
ax.set_title('Map of Obstacles')
ax.legend()

# Show the plot
plt.show()
