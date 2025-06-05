import numpy as np
import os

# "Add your code" exercise
message = "\nEntry script(this is different from Docker's entrypoint) run successfully!\n"
print(message)

# "Defining `DTproject` dependencies" exercise
print("Numpy addition result: ", np.add(3.14, 2.71))

vehicle_name = os.environ['VEHICLE_NAME']
message = f"\nHello from {vehicle_name}!\n"
print(message)
