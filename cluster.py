import os
import json
import numpy as np
import umap
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
import argparse

def load_vectors_from_json(directory):
    vectors = []
    labels = []
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            with open(file_path, 'r') as file:
                data = json.load(file)
                if "vector" in data:
                    vectors.append(data["vector"])
                    if data["stuck"] == 1:
                        labels.append("red")
                    elif data["collision"] == 1 and "ambulance" in filename:
                        labels.append("green")
                    elif data["collision"] == 1:
                        labels.append("yellow")
                    else:
                        labels.append("blue")
    return np.array(vectors), labels

def visualize_umap(vectors, labels):
    scaler = StandardScaler()
    vectors_scaled = scaler.fit_transform(vectors)
    
    reducer = umap.UMAP(n_components=2, random_state=42)
    embedding = reducer.fit_transform(vectors_scaled)

    plt.figure(figsize=(10, 7))
    # plt.scatter(embedding[:, 0], embedding[:, 1], cmap='Spectral', s=10)
    plt.scatter(embedding[:, 0], embedding[:, 1], c=labels, s=10)
    plt.title('UMAP projection of scenario vectors')
    plt.xlabel('UMAP Dimension 1')
    plt.ylabel('UMAP Dimension 2')
    plt.show()

def parse_arguments():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("-d", "--dir", help="Directory of corner cases json files.", type=str, default="corner_case")
    arguments = parser.parse_args()
    return arguments

def main():
    args = parse_arguments()
    current_directory = os.getcwd()
    vectors, labels = load_vectors_from_json(current_directory + "/" + args.dir)

    if len(vectors) > 0:
        visualize_umap(vectors, labels)

if __name__ == "__main__":
    main()