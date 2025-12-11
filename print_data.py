import numpy as np

file = "updated_array.npy"  

# Load the array
arr = np.load(file)

# Print full array without truncation
np.set_printoptions(threshold=np.inf)  

print("Array shape:", arr.shape)
print("\nFull array:\n")
print(arr)