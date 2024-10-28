import os
import json
import numpy as np
import pandas as pd
import umap
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go
from sklearn.preprocessing import StandardScaler
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
import argparse
import seaborn as sns

def load_vectors(directory):
    vectors = []
    case_labels = []
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            with open(file_path, 'r') as file:
                data = json.load(file)
                if "vector" in data and data["vector"] is not None:
                    vectors.append(data["vector"])
                    if data["stuck"] == 1:
                        case_labels.append("stuck")
                    elif data["collision"] == 1:
                        case_labels.append("collision")
                    else:
                        case_labels.append("safe")
    return np.array(vectors), case_labels

def load_vectors_visual(directory):
    file_names = []
    sp_xy_list = []
    dp_xy_list = []
    npc_bp_list = []
    weathers = []
    vectors = []
    labels = []
    delta_z_list = []
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            with open(file_path, 'r') as file:
                data = json.load(file)
                if "vector" in data:
                    file_names.append(filename)
                    sp_xy_list.append(data["vector"][:2])
                    dp_xy_list.append(data["vector"][4:6])
                    npc_bp_list.append([npc["blueprint"] for npc in data["npc_list"]])
                    weathers.append(data["weather"])
                    delta_z_list.append(data["vector"][-5])
                    vectors.append(data["vector"])
                    if data["stuck"] == 1:
                        labels.append("stuck")
                    elif data["collision"] == 1:
                        labels.append("collision")
                    else:
                        labels.append("safe")
    return file_names, np.array(vectors), labels, sp_xy_list, dp_xy_list, npc_bp_list, weathers, delta_z_list

def my_umap(vectors):
    scaler = StandardScaler()
    vectors_scaled = scaler.fit_transform(vectors)
    
    umap_model = umap.UMAP(n_components=2)
    embedding = umap_model.fit_transform(vectors_scaled)
    return embedding, umap_model

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

def dbscan_clustering(embedding, eps=0.3, min_samples=5):
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
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
    parser.add_argument("-d", "--dir", help="Directory of corner cases json files.", type=str, default="output")
    parser.add_argument("-c", "--cluster-only", action="store_true", default=False)
    arguments = parser.parse_args()
    return arguments

def main():
    args = parse_arguments()
    
    if args.cluster_only:
        df = pd.read_csv("umap_data.csv")
        embedding = df[['UMAP Dimension 1', 'UMAP Dimension 2']].values
    else:
        current_directory = os.getcwd()
        file_names, vectors, case_labels, sp_xy_list, dp_xy_list, npc_bp_list, weathers, delta_z_list = load_vectors_visual(current_directory + "/" + args.dir)
        embedding, umap_model = my_umap(vectors)
        
        df = pd.DataFrame(embedding, columns=['UMAP Dimension 1', 'UMAP Dimension 2'])
        df["file_name"] = file_names
        df["case_label"] = case_labels
        df["sp_xy"] = sp_xy_list
        df["dp_xy"] = dp_xy_list
        df["npc_bp"] = npc_bp_list
        df["weather"] = weathers
        df["delta_z"] = delta_z_list
        df.to_csv('umap_data.csv', index=False)
    
    # elbow_method(vectors)
    # n_clusters = 8
    # cluster_labels = k_means_clustering(embedding, n_clusters)
    cluster_labels = dbscan_clustering(embedding, eps=0.3, min_samples=4)    
    df["cluster"] = cluster_labels
    df["hover_text"] = df.apply(lambda row: "{}: sp=[{}], dp=[{}], delta_z={:.1f}, weather.deposits={:.1f}, npc_bp={}".format(
        row["file_name"], 
        ', '.join(f"{value:.1f}" for value in row["sp_xy"]), 
        ', '.join(f"{value:.1f}" for value in row["dp_xy"]), 
        row["delta_z"],
        row["weather"]["precipitation_deposits"],
        row["npc_bp"]
    ), axis=1)
    fig = go.Figure()
    shape_mapping = {
        "safe": "circle",
        "collision": "x",
        "stuck": "square"
    }
    num_clusters = df['cluster'].nunique()
    colors = sns.color_palette("husl", num_clusters)
    colors_255 = ["rgb({}, {}, {})".format(int(r * 255), int(g * 255), int(b * 255)) for r, g, b in colors]
    
    for cluster in df['cluster'].unique():
        cluster_data = df[df['cluster'] == cluster]
        for case_label in shape_mapping.keys():
            label_data = cluster_data[cluster_data['case_label'] == case_label]
            # if not label_data.empty:
            if case_label in ["stuck", "collision"]:
                fig.add_trace(go.Scatter(
                    x=label_data['UMAP Dimension 1'],
                    y=label_data['UMAP Dimension 2'],
                    mode='markers',
                    marker=dict(
                        symbol=shape_mapping[case_label],
                        size=10,
                        # color=f'rgb({max(0, cluster * 60)}, {100 + cluster * 50}, {50 + cluster * 50})'
                        color=colors_255[cluster],
                        line=dict(width=1,color="rgb(250,250,250)") if case_label != "safe" else None
                    ),
                    name=f'Cluster {cluster} - {case_label}',
                    hovertext=label_data["hover_text"],
                    hoverinfo='text'
                ))


    fig.update_layout(
        title='UMAP Clustering Visualization',
        xaxis_title='UMAP Dimension 1',
        yaxis_title='UMAP Dimension 2',
        legend_title='UMAP Visualization'
    )
    fig.show()

if __name__ == "__main__":
    main()