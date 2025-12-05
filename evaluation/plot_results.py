import matplotlib.pyplot as plt

def easy_plot(x, y, title="Padding Evaluation (500 Cases)", xlabel="Padding [cm]", ylabel="Success Rate [%]",
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
x = [0, 0.85, 1.7, 3.4, 5.0, 7.0, 10.0, 15.0, 20.0]
y = [86.8, 75.8, 75.8, 68, 57.4, 41.4, 35.2, 18.6, 8.6]

easy_plot(
    x, y,
    
)