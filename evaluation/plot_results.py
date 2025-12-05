import matplotlib.pyplot as plt

def easy_plot(x, y, title="Goal Tolerance Evaluation (100 Cases)", xlabel="Goal Tolerance [cm]", ylabel="Step Counter",
              marker="o", save_path=None):

    plt.figure(figsize=(6,4))
    plt.plot(x, y, marker=marker)

    plt.xticks(x)
    plt.yticks(y)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"Saved plot to {save_path}")

    plt.show()


# Example use every time you run the file:
tolerance = [40, 30, 25, 20, 15, 10 , 5] #x
success_rate = [97, 97, 97, 97, 95, 93, 50] #y
average_path_length = [41.87, 42.72, 43.12, 43.49, 44.45, 45.60, 47.54]

easy_plot(
    tolerance, average_path_length,
    
)