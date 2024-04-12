import pickle
import matplotlib.pyplot as plt

# Load the pickle file
with open('obstacles2.pkl', 'rb') as file:
    obstacles = pickle.load(file)

print(obstacles)
fig, ax = plt.subplots()
x_coords, y_coords = zip(*obstacles)  
x_coords=list(x_coords)
y_coords=list(y_coords)
# for i in range (len(x_coords)):
#     x_coords[i]*=0.15
#     y_coords[i]*=0.15
print(min(x_coords),max(x_coords))
print(min(y_coords),max(y_coords))





ax.scatter(x_coords, y_coords, c='red', marker='o', label='Obstacles')

ax.set_xlabel('X coordinates')
ax.set_ylabel('Y coordinates')
ax.set_title('Map of Obstacles')
ax.legend()
plt.show()
