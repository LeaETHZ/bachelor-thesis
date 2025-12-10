import matplotlib.pyplot as plt


def easy_plot(x, y_list, labels, title="Goal Tolerance Evaluation (100 Cases)", 
              xlabel="Goal Tolerance [cm]", ylabel="Metric",
              marker="o", save_path=None):

    plt.figure(figsize=(6,4))

    # Plot all curves
    for y, label in zip(y_list, labels):
        plt.plot(x, y, marker=marker, label=label)

    plt.xticks(x)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"Saved plot to {save_path}")

    plt.show()


# Example use every time you run the file:
tolerance =    [ 5, 10, 15, 20, 25, 30, 35, 40, 45] #x
success_rate = [36, 62, 63, 65, 65, 65, 65, 66, 66] #y
path_length = [53.17 , 49.95, 48.73, 48.20, 47.71, 47.18, 46.85, 46.58, 46.14]
distance_to_goal = [4.92,7.6, 11.38,15.11, 21.56, 28.57, 33.58, 37.95, 44.79]


# easy_plot(
#     tolerance, distance_to_goal,
    
# )

easy_plot(
    tolerance,
    [distance_to_goal, success_rate],
    labels=["Distance to Goal", "Path Length"]
)
