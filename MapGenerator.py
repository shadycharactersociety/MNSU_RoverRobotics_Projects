# Map Creation Utility for Rover Coordinate List  3-5-24

from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist
import numpy as np
import pandas as pd

qty = int(input('How many reference points would you like? (#)\n'))  # Number of clusters generated

# Read in the file
with open('DemoRoomCords.txt', 'r') as file:
  filedata = file.read()

# Replace the target string
filedata = filedata.replace(', ', ' ')
# Write the file out again
with open('DemoRoomCords.txt', 'w') as file:
  file.write(filedata)

data = pd.read_csv('DemoRoomCords.txt', sep='\s+', header=None)  # Load data from file
data = pd.DataFrame(data)
pca = PCA(2)
df = pca.fit_transform(data)         # Transform the data
kmeans = KMeans(n_clusters=qty)        # Initialize the class object
label = kmeans.fit_predict(df)       # predict the labels of clusters.
centroids = kmeans.cluster_centers_  # Getting the Centroids

Cords = [[0 for x in range(2)] for y in range(qty)]  # Convert centroids array to numpy format

for i in range(qty):
    Cords[i][0] = centroids[i][0]
    Cords[i][1] = centroids[i][1]

for i in range(qty):  # Find & connect each point to its closest neighbor
    currentcord = np.array([[Cords[i][0], Cords[i][1]]])  # Define current coordinate
    Cords = np.delete(Cords, i, axis=0)  # Remove current point so it doesn't connect to itself
    distances = cdist(currentcord, Cords)  # create list of distances to neighboring points
    min_index = np.argmin(distances)  # find array position of smallest distance
    x = [currentcord[0][0], Cords[min_index][0]]  # define x components of the two points
    y = [currentcord[0][1], Cords[min_index][1]]  # define y components of the two points
    plt.plot(x, y)  # plot connected points on graph
    Cords = np.insert(Cords, i, currentcord, axis=0)  # put current point back into list

u_labels = np.unique(label)

showcords = int(input('Would you like original coordinates on the map? (Yes = 1, No = 0)\n'))
if showcords == 1:
    for i in u_labels:  # Plot the results
        plt.scatter(df[label == i, 0], df[label == i, 1], label=i)   # Original Scatter Plot
showcentroids = int(input('Would you like original centroids on the map? (Yes = 1, No = 0)\n'))
if showcentroids == 1:
    plt.scatter(centroids[:, 0], centroids[:, 1], s=80, color='k')  # Calculated Centroid Points
plt.show()  # output plot

with open('MapCords.txt', 'a') as f:
    for i in range(len(centroids)):
        f.write('\n')
        f.write(str(centroids[i, 0]))
        f.write(' ')
        f.write(str(centroids[i, 1]))