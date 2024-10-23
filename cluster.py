import os
import json
import numpy as np
import pandas as pd
import umap
import matplotlib.pyplot as plt
import plotly.express as px
from sklearn.preprocessing import StandardScaler
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
import argparse

def load_vectors_from_json(directory):
    file_names = []
    vectors = []
    labels = []
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            with open(file_path, 'r') as file:
                data = json.load(file)
                if "vector" in data:
                    file_names.append(filename)
                    vectors.append(data["vector"])
                    if data["stuck"] == 1:
                        labels.append("stuck")
                    elif data["collision"] == 1:
                        labels.append("collision")
                    else:
                        labels.append("safe")
    return file_names, np.array(vectors), labels

def my_umap(vectors):
    scaler = StandardScaler()
    vectors_scaled = scaler.fit_transform(vectors)
    
    reducer = umap.UMAP(n_components=2, random_state=42)
    embedding = reducer.fit_transform(vectors_scaled)
    return embedding

def visualize_umap(embedding, labels):
    plt.figure(figsize=(10, 7))
    # plt.scatter(embedding[:, 0], embedding[:, 1], cmap='Spectral', s=10)
    plt.scatter(embedding[:, 0], embedding[:, 1], c=labels, s=10)
    plt.title('UMAP projection of scenario vectors')
    plt.xlabel('UMAP Dimension 1')
    plt.ylabel('UMAP Dimension 2')
    plt.show()

def elbow_method(embedding, max_k=10):
    wcss = []

    for k in range(1, max_k + 1):
        kmeans = KMeans(n_clusters=k, random_state=42)
        kmeans.fit(embedding)
        wcss.append(kmeans.inertia_)

    plt.figure(figsize=(10, 6))
    plt.plot(range(1, max_k + 1), wcss, marker='o')
    plt.title('Elbow Method')
    plt.xlabel('Number of Clusters (K)')
    plt.ylabel('WCSS')
    plt.xticks(range(1, max_k + 1))
    plt.grid()
    plt.show()

def k_means_clustering(embedding, n_clusters):
    kmeans = KMeans(n_clusters=n_clusters, random_state=42)
    kmeans.fit(embedding)
    labels = kmeans.labels_
    
    return labels

def dbscan_clustering(embedding, eps=0.5, min_samples=5):
    dbscan = DBSCAN(eps, min_samples)
    labels = dbscan.fit_predict(embedding)
    return labels

def plot_clusters(embedding, labels, n_clusters):
    plt.figure(figsize=(10, 7))
    
    for i in range(n_clusters):
        plt.scatter(embedding[labels == i, 0], embedding[labels == i, 1], label=f'Cluster {i}', s=10)
    
    plt.title('K-means Clustering Results')
    plt.xlabel('UMAP Dimension 1')
    plt.ylabel('UMAP Dimension 2')
    plt.legend()
    plt.show()

def parse_arguments():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("-d", "--dir", help="Directory of corner cases json files.", type=str, default="corner_case")
    arguments = parser.parse_args()
    return arguments

def main():
    args = parse_arguments()
    current_directory = os.getcwd()
    file_names, vectors, labels = load_vectors_from_json(current_directory + "/" + args.dir)
    embedding = my_umap(vectors)
    # elbow_method(vectors)
    
    # if len(vectors) > 0:
    #     visualize_umap(embedding, labels)
    df = pd.DataFrame(embedding, columns=['UMAP Dimension 1', 'UMAP Dimension 2'])
    df['File Name'] = file_names

    fig = px.scatter(df, 
                    x='UMAP Dimension 1', 
                    y='UMAP Dimension 2', 
                    hover_name='File Name',
                    title='UMAP Visualization with File Names',
                    color=labels)

    fig.show()
    # n_clusters = 8
    # labels, kmeans_model = k_means_clustering(embedding, n_clusters)
    # plot_clusters(embedding, labels, n_clusters)

if __name__ == "__main__":
    main()