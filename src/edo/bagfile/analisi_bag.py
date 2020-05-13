import numpy as np
import pandas as pd
import rosbag
import matplotlib.pyplot as plt

def salva(plot, nome):
    fig = plot.get_figure()
    fig.savefig(nome)

if __name__ == "__main__":
    bag = np.array(list(rosbag.Bag("subset.bag").read_messages()))
    tempi = map(lambda x: x[1].header.stamp.nsecs, bag)
    # Posizioni un braccio per colonna
    posizioni = pd.DataFrame(map(lambda x: x[1].position, bag))
    velocita = pd.DataFrame(map(lambda x: x[1].velocity, bag))

    posizioni.index = tempi
    velocita.index = tempi

    iloc = [850,855]

    salva(posizioni.iloc[iloc[0]:iloc[1]].plot(title="posizione"), "posizioni_presalto.png")
    salva(velocita.iloc[iloc[0]:iloc[1]].plot(title="velocita"), "velocita_presalto.png")
