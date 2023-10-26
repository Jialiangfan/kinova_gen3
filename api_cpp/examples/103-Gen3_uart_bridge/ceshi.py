import matplotlib.pyplot as plt

# Load data
with open("desired_data.txt", "r") as f:
    desired_data = [list(map(float, line.strip().split())) for line in f]
with open("real_data.txt", "r") as f:
    real_data = [list(map(float, line.strip().split())) for line in f]

# Extract X and Y coordinates
desired_x = [data[0] for data in desired_data]
desired_y = [data[1] for data in desired_data]
real_x = [data[0] for data in real_data]
real_y = [data[1] for data in real_data]

# Plot the data
plt.figure(figsize=(10, 10))
plt.plot(desired_x, desired_y, label="Desired Path", color="blue")
plt.plot(real_x, real_y, label="Real Path", color="red", linestyle="--")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Desired vs Real Path of the Robot")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()

