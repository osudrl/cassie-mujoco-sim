# optimal_cassie.py
# contains functions to load and query optimal coordinate data

import numpy as np
from scipy.io import loadmat
from scipy.interpolate import interpn
from scipy.linalg import expm

# gets contents of a .mat file (usually cassie.mat)
# inputs:
    # mat_file: location of data file
# outputs:
    # trans_dict: dictionary containing N-D transform grids
def load_data(mat_file='cassie', **kwargs):
  # use loadmat to get matrices
  trans_dict = loadmat(mat_file, simplify_cells=True)
  return trans_dict

# interpolates contents of cassie.mat at an alpha, and returns a rotation matrix
# inputs:
    # trans_dict: dictionary containing N-D transform grids
    # alpha: N-D query point
    # order: any string value, 'xyz' or similar, communicating angle order
# outputs:
    # interp: dictionary containing x,y,z values, and rotation matrices
def interpolate_so3(trans_dict, alpha, order='zyx', **kwargs):
  # interpolate ea. dim. at query point
  points = tuple(trans_dict['samples'])
  interp = dict()
  interp['x'] = interpn(points, trans_dict['X'], alpha)
  interp['y'] = interpn(points, trans_dict['Y'], alpha)
  interp['z'] = interpn(points, trans_dict['Z'], alpha)
  # generate matrix lie algebra (so) representation
  f = np.array([[0, 0, 0], [0, 0, -1], [0, 1, 0]])
  s = np.array([[0, 0, 1], [0, 0, 0], [-1, 0, 0]])
  t = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
  interp['inf_mat'] = f*interp[order[0]] + s*interp[order[1]] + t*interp[order[2]]
  # exponentiate for matrix lie group (SO) representation
  interp['rot_mat'] = expm(interp['inf_mat'])
  # return interpolation structure
  return interp

# constructs a single, simple, interpolating function
# inputs:
    # mat_file: location of data file
    # order: any string value, 'xyz' or similar, communicating angle order
# outputs:
  # function, taking a single query point alpha and returning a rotation matrix
def interpolating_function(**kwargs):
  # load data
  trans_dict = load_data(**kwargs)
  # return interpolate_so3 without alpha
  return lambda alpha: interpolate_so3(trans_dict, alpha, **kwargs)['rot_mat']

if __name__ == "__main__":
  # data loading
  trans_dict = load_data()

  # interpolation, returning full interpolation data
  alpha = np.array([0.1, 0.75, -1.2, 0, 0.75, -1.2])
  interp = interpolate_so3(trans_dict, alpha)
  print('full interpolation data:', '\n', interp)
  
  # construction of interpolating function
  interp_fn = interpolating_function()
  print('simplified rotation matrix:',  '\n', interp_fn(alpha))
  # construction of interpolating function, different order
  interp_fn_xyz = interpolating_function(order='xyz')
  print('simplified rotation matrix, different order:',  '\n', interp_fn_xyz(alpha))