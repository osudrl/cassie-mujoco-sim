# dynamic-cassie

This project computes minimum perturbation, rotational coordinates for OSU's
Cassie robot. There are two project components: the MATLAB optimizer, and a
python wrapper that does the data interpolation.

## Python Wrapper

The `optimal_cassie.py` wrapper invokes the `cassie.mat` file, which contains
samples, grid, and X, Y, Z optimal coordinates for Cassie. It defines several
functions to load and use the coordinates.

### load_data
Loads data stored in `cassie.mat`, and returns it in a dictionary.

### interpolate_so3
Interpolation workhorse. Takes loaded data, a query point, and 3D rotation angle
order, and returns an `interp` dictionary containing the resulting transform
in a variety of forms.

### interpolating_function
A simplified version of the above two functions. This returns a _function_,
which takes just a query point and returns a 3D rotation matrix. This function
can take the same arguments as the above two functions to change how the
interpolating function is constructed.

## MATLAB Optimizer

The optimizer uses the included `6dof_3lvl_sweep.txt` data file to construct a
local connection for Cassie. It then computes minimum perturbation coordinates,
using a method published by Travers., et. al. All resulting data is saved in
the included `cassie.mat` file; this should not need to be run except for higher
resolution/higher DoF sweeps of the robot.

> Note: The optimizer uses some functions from OSU-LRAM's `sysplotter`. A
> local installation is required to run the optimizer.