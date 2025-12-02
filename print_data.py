import numpy as np

file = "arrayJulia.npy"  

# Load the array
arr = np.load(file)

# Print full array without truncation
np.set_printoptions(threshold=np.inf)  

print("Array shape:", arr.shape)
print("\nFull array:\n")
print(arr)