<img src="REAP.gif" alt="REAP-T Demo GIF" width="500">
# Robust to Early Termination Model Predictive Control Toolbox (REAP-T)

## 🛠️ Getting Started:
1. **Extract the YALMIP zip file** to the location of the REAP-T file.
2. **Run the `DisREAP_UI` function** in MATLAB.

## 🚀 Usage:
1. **Import your system and desired configurations**:
   - After running `DisREAP_UI`, import your system model and specify your desired configurations,  enable plots, and simulate the system to observe results.
2. **Execute the package** to start the control process.

   
## 📋 **Input Fields Explained**


### **System Matrices**
1. **Matrix A**  
   Represents the state-transition matrix of the system.  
   **Example**:  
A = [0 1; -2 -3]

2. **Matrix B**  
Input matrix defining how the control inputs affect the system.  
**Example**:
B = [0; 1]

3. **Matrix C**  
Output matrix, mapping the state to system outputs.  
**Example**:  
C = [1 0]

4. **Matrix D**  
Feedthrough matrix, directly connecting input to output.  
**Example**:
D = 0


### **Constraints**
5. **X Constraint U.B.** (Upper Bound)  
Upper bounds for the system state vector \(x\).  
**Example**: `[5; 5]`

6. **X Constraint L.B.** (Lower Bound)  
Lower bounds for the system state vector \(x\).  
**Example**: `[-5; -5]`

7. **U Constraint U.B.** (Upper Bound)  
Upper bounds for the control input vector \(u\).  
**Example**: `[1]`

8. **U Constraint L.B.** (Lower Bound)  
Lower bounds for the control input vector \(u\).  
**Example**: `[-1]`


### **Cost Function Weights**
9. **Matrix Qx**  
Weighting matrix for penalizing state deviations in the cost function.  
**Example**:  
Qx = [1 0; 0 1]


10. **Matrix Qu**  
 Weighting matrix for penalizing control efforts in the cost function.  
 **Example**:  
 Qu = [0.01]


11. **Initial Condition** (\(x_0\))  
 Initial state of the system.  
 **Example**: `[0; 0]`





### **Simulation Parameters**
12. **Prediction Horizon**  
 The number of future steps the controller optimizes for.  
 **Example**: `10`

13. **Desired Target**  
Desired value for the reference trajectory or desired steady-state configuration (Equilibrium point)
14. **# Time Instants**  
 Total number of time steps for the simulation.  
 **Example**: `100`

15. **Sampling Period** (\(\Delta T\))  
 Time interval between successive steps.  
 **Example**: `0.2`

16. **Terminal Constraint** 
 The mode of the terminal constraints; could be prediction-based or Lyapunov-based.  


### **Plots**
17. **Select Plots**  
 Choose which aspects of the system to visualize:  
 - **States**: Display state trajectories.  
 - **Control Inputs**: Show control signals over time.  
 - **Output**: Visualize system output.  
 - **Sigma**: Plot algorithm-specific sigma values.

---

### **Algorithm Mode**
18. **Algorithm Mode Dropdown**  
 - **Automatic**: Let the system decide optimal parameters (disables `Omegastar` input).  
 - **Manual**: Allows customization of the `Omegastar` parameter.

---

This interface makes it easier to define and simulate discrete-time systems with constraints and optimization. Start exploring your systems now! 🚀


## 📚 Examples:
For guidance, the package includes two examples:
1. **Parrot Bebop 2 Drone**:
   - This example demonstrates the implementation of DisREAP on a Parrot Bebop 2 drone.
2. **F-16 Aircraft Model**:
   - This example showcases DisREAP applied to a F-16 Aircraft Model.

## ▶️ Running Examples:
1. **Navigate to the examples directory**:
   - Open the folder containing the examples within the extracted files.
2. **Run the example scripts**:
   - Follow the instructions in the example scripts to see the results and understand how to set up and execute your own system.

## 🤝 Support:
For any issues or questions, please refer to the [issues page](https://github.com/mhsnar/DiscreteREAP/issues) on GitHub.

## Citing DisREAP:

If you use the DisREAP Toolbox, please use the following BibTeX entry:
```bibtex
@article{amiri2025reap,
  title={REAP-T: A MATLAB Toolbox for Implementing Robust-to-Early Termination Model Predictive Control},
  author={Amiri, Mohsen and Hosseinzadeh, Mehdi},
  journal={arXiv preprint arXiv:2507.00863},
  year={2025}
}




