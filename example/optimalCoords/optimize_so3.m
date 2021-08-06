% finds SO(3) minimum perturbation coordinates for an n-dim system

% objective function based on Ross Hatton's SO(3) coordinate optimization
% quadrature rules based on Becker Carey Oden 5.3.2

% inputs:
    % grid_points: cell array containing vector dimension(s) of points
    % A_orig: 
        % local connection tensoral object
        % ought to be a (n_dim)x(n_dim)x...x(n_dim) cell of 3x... matrices
% outputs:
    % beta: 3D coordinate transform
    % A_opt:
        % local connection tensoral object, in optimal coordinates

function [grid, X, Y, Z] = optimize_so3(grid_points, A_orig, reference)
    % dimensionality
    n_dim = length(grid_points);
    
    %% hypercube mesh setup
    % generate hypercube mesh to work over
    grid = cell(1, n_dim);
    [grid{:}] = ndgrid(grid_points{:});
    [nodes, cubes] = hypercube_mesh(grid);
    % generate shape functions
    [basis_fns, d_basis_fns] = hypercube_element_shape_functions(n_dim);
    % generate points, weights for quadrature integration
    [quad_points, quad_weights] = hypercube_quadrature_init(n_dim);
    % variables for clarity
    n_nodes = size(nodes, 1);
    n_points = size(quad_points{1},1);
    n_vertices = size(cubes, 2);
    
    %% compute values of basis fns/derivatives at quad vertices
    % matrix of basis values at quadpoints (n_points rows of 2^n_dim columns)
    % for ea. quad point, value of surrounding points in a hypercube
    basis_cell = cellfun(@(fn) fn(quad_points),basis_fns,...
                         'UniformOutput', false);
    bases_at_quad = cat(2, basis_cell{:});
    % 2d cell of basis derivatives
    d_basis_cell = cellfun(@(fn) fn(quad_points), d_basis_fns,...
                           'UniformOutput', false);
    % convert to cell of derivative matrices (n_dim matrices)
    % value of derivative of basis, for each quad point's surroundings,
    % along each dimension
    d_bases_at_quad = cell(1, n_dim);
    for i = 1:n_dim
        % concat all (2^n_dim) derivative points into matrix
        % stored in cell corresp. to dimension
        d_bases_at_quad{i} = cat(2, d_basis_cell{:,i});
    end
    
    %% scale quadrature points according to problem coordinates
    % big assumption: regular grid spacing
    % preallocate space
    bases_at_quad_act = cell(n_dim, 1); % points in each dimension
    quad_deriv_points_act = cell(n_dim, n_dim); % points w.r.t. ea. dim.
    for i = 1:n_dim
        % SCALES ea. value by size of cube in ea. dim
        % assumes a regular grid, and can be done because linear basis
        bases_at_quad_act{i} = bases_at_quad * nodes(cubes(1,:), i);
        
        % do derivatives for this dim
        for j =1:n_dim
            % derivatives along all other (jth) dimensions for ith dim
            quad_deriv_points_act{i, j} = d_bases_at_quad{j} * nodes(cubes(1,:), i);
        end
    end
    
    %% scale basis function derivatives using the Jacobian
    % get Jacobian from first quadpoint (invariant across regular grid)
    J_all = celltensorconvert(quad_deriv_points_act); %nxn cell -> cell of nxn
    J = J_all{1};
    % scale derivatives w.r.t. problem coordinates
    d_basis_mat = cell2mat(d_basis_cell);
    d_basis_mat_act = (J\d_basis_mat')'; %effectively "divides" by J
    % split into (n_bases x n_dim) 2d cell of n_quad_points vectors
    d_basis_cell_act = mat2cell(d_basis_mat_act,...
                                n_points*ones(n_vertices,1),...
                                ones(1,n_dim));
    % finally, construct rescaled cell of derivative matrices
    for i = 1:n_dim %loop along ea. dimension
        d_bases_at_quad{i} = cat(2, d_basis_cell_act{:,i});
    end
    
    %% collect ingredients for calculus over the field
    % _dim concatenates ea. dimension into a single matrix
    
    % quadrature weights (for integration across dimensions)
    quad_weights_dim = repmat(quad_weights', 1, n_dim); %1x(n_points)n_dim
    
    % nabla (del, gradient)
    % derivative of each basis fn, in each direction, at each vertex
    del_dim = cat(1, d_bases_at_quad{:}); %(n_points)(n_dim) x n_vertices
    
    % gradient of basis functions
    grad_rho_dim = del_dim; % equal; all fns over field are linear w.r.t. basis fns
    
    % rho
    rho_q = bases_at_quad;
    rho_q_dim = repmat(rho_q, n_dim, 1);
    
    % gradient dot product, integrated early
    grad_rho_dot_del = cell(1, n_vertices); %values at each vertex
    for i = 1:n_vertices
        gradient = repmat(grad_rho_dim(:,i), 1, n_vertices) .* del_dim;
        grad_rho_dot_del{i} = quad_weights_dim * gradient;
    end
    
    %% construct linear system from objective functions
    % organization:
        % rows are blocked out by equation
        % columns are blocked out by weights (x, y, z)
    % LHS:
        % 3 equations, evaluated at n_nodes points
        % assigning 3 weights (x, y, z) for each node\
    LHS = zeros(3*n_nodes);
    % RHS: vector result of an area integral
    % manifests as a matrix, so rows of bases/dimensions will be summed
    RHS = zeros(3*n_nodes, n_nodes*n_dim);
    
    % reorganize LC (by rotational component, and by node, for later)
    % each (n_nodes)x(n_dim) organized in x,y,z pages
    A_nodes = cat(3, grid_to_columns(A_orig(1,:)),...
                     grid_to_columns(A_orig(2,:)),...
                     grid_to_columns(A_orig(3,:)));
    
    % add contributions from each node (rows of LHS, RHS)
    for n = 1:n_nodes
        cubes_containing_node = cubes(any(cubes==n, 2),:);
        
        % contributions are integral values at this node, summed over all
        % adjacent hypercubes
        for c = 1:size(cubes_containing_node, 1)
            % identify corner with node of interest
            corner_idx = cubes_containing_node(c, :) == n;
            
            % get LC at cube vertices (breaking into components)
            A_x_vert = A_nodes(cubes_containing_node(c,:),:,1);
            A_y_vert = A_nodes(cubes_containing_node(c,:),:,2);
            A_z_vert = A_nodes(cubes_containing_node(c,:),:,3);
            % get LC at quadrature points
            A_x_q = bases_at_quad * A_x_vert;
            A_y_q = bases_at_quad * A_y_vert;
            A_z_q = bases_at_quad * A_z_vert;
            % get componentwise dot products
            A_xx = A_x_q(:).^2;
            A_yy = A_y_q(:).^2;
            A_zz = A_z_q(:).^2;
            
            
            % compute "vector field squared" terms
            x_rho_A_q = rho_q_dim(:, corner_idx) .* (A_zz + A_yy);
            y_rho_A_q = rho_q_dim(:, corner_idx) .* (A_zz - A_xx);
            z_rho_A_q = rho_q_dim(:, corner_idx) .* (A_yy + A_xx);
            % integrate
            x_rho_A = quad_weights_dim * (repmat(x_rho_A_q, 1, n_vertices) .* rho_q_dim);
            y_rho_A = quad_weights_dim * (repmat(y_rho_A_q, 1, n_vertices) .* rho_q_dim);
            z_rho_A = quad_weights_dim * (repmat(z_rho_A_q, 1, n_vertices) .* rho_q_dim);
            
            % compute "leading gradient" terms
            grad_rho_A_x_q = grad_rho_dim(:, corner_idx) .* A_x_q(:);
            grad_rho_A_y_q = grad_rho_dim(:, corner_idx) .* A_y_q(:);
            grad_rho_A_z_q = grad_rho_dim(:, corner_idx) .* A_z_q(:);
            % integrate
            grad_rho_A_x_rho = quad_weights_dim * (repmat(grad_rho_A_x_q, 1, n_vertices) .* rho_q_dim);
            grad_rho_A_y_rho = quad_weights_dim * (repmat(grad_rho_A_y_q, 1, n_vertices) .* rho_q_dim);
            grad_rho_A_z_rho = quad_weights_dim * (repmat(grad_rho_A_z_q, 1, n_vertices) .* rho_q_dim);
            
            % compute "leading rho" terms
            rho_A_x_grad_q = repmat(rho_q_dim(:, corner_idx), 1, n_vertices) .* repmat(A_x_q(:), 1, n_vertices) .* del_dim;
            rho_A_y_grad_q = repmat(rho_q_dim(:, corner_idx), 1, n_vertices) .* repmat(A_y_q(:), 1, n_vertices) .* del_dim;
            rho_A_z_grad_q = repmat(rho_q_dim(:, corner_idx), 1, n_vertices) .* repmat(A_z_q(:), 1, n_vertices) .* del_dim;
            % integrate
            rho_A_x_grad = quad_weights_dim * rho_A_x_grad_q;
            rho_A_y_grad = quad_weights_dim * rho_A_y_grad_q;
            rho_A_z_grad = quad_weights_dim * rho_A_z_grad_q;
            
            
            % build each term (everywhere in cube)
            x.xcols = grad_rho_dot_del{corner_idx} + x_rho_A;
            x.ycols = grad_rho_A_z_rho - rho_A_z_grad;
            x.zcols = -grad_rho_A_y_rho + rho_A_y_grad;
            
            y.xcols = grad_rho_A_z_rho + rho_A_z_grad;
            y.ycols = grad_rho_dot_del{corner_idx} + y_rho_A;
            y.zcols = -grad_rho_A_x_rho - rho_A_x_grad;
            
            z.xcols = grad_rho_A_y_rho - rho_A_y_grad;
            z.ycols = -grad_rho_A_x_rho + rho_A_x_grad;
            z.zcols = grad_rho_dot_del{corner_idx} + z_rho_A;
            
            % identify indices
            nodes_in_cube = cubes_containing_node(c,:);
            x.row = n;
            x.col = nodes_in_cube;
            y.row = n + n_nodes;
            y.col = nodes_in_cube + n_nodes;
            z.row = n + 2*n_nodes;
            z.col = nodes_in_cube + 2*n_nodes;
            
            % construct LHS matrix
            LHS(x.row, x.col) = LHS(x.row, x.col) + x.xcols;
            LHS(x.row, y.col) = LHS(x.row, y.col) + x.ycols;
            LHS(x.row, z.col) = LHS(x.row, z.col) + x.zcols;
            
            LHS(y.row, x.col) = LHS(y.row, x.col) + y.xcols;
            LHS(y.row, y.col) = LHS(y.row, y.col) + y.ycols;
            LHS(y.row, z.col) = LHS(y.row, z.col) + y.zcols;
            
            LHS(z.row, x.col) = LHS(z.row, x.col) + z.xcols;
            LHS(z.row, y.col) = LHS(z.row, y.col) + z.ycols;
            LHS(z.row, z.col) = LHS(z.row, z.col) + z.zcols;
            
            
            % compute RHS gradient terms, integrating as we go
            for d = 1:n_dim
                grad_rho_A_x = quad_weights(:)' *...
                               (repmat(d_bases_at_quad{d}(:,corner_idx), 1, n_vertices) .*...
                                repmat(A_x_q(:,d), 1, n_vertices));
                grad_rho_A_y = quad_weights(:)' *...
                               (repmat(d_bases_at_quad{d}(:,corner_idx), 1, n_vertices) .*...
                                repmat(A_y_q(:,d), 1, n_vertices));
                grad_rho_A_z = quad_weights(:)' *...
                               (repmat(d_bases_at_quad{d}(:,corner_idx), 1, n_vertices) .*...
                                repmat(A_z_q(:,d), 1, n_vertices));
                
                % assign RHS
                rhs_cols = nodes_in_cube + (d-1) * n_nodes;
                RHS(x.row, rhs_cols) = RHS(x.row, rhs_cols) + grad_rho_A_x;
                RHS(y.row, rhs_cols) = RHS(y.row, rhs_cols) + grad_rho_A_y;
                RHS(z.row, rhs_cols) = RHS(z.row, rhs_cols) + grad_rho_A_z;
            end
        end
    end
    
    %% solve for weights
    % potential to remove rows/cols corresp. to reference configuration
    % introduces a static rotation to coordinates (TBD if valid; commented)
    if ~exist('reference', 'var')
        % get "middle" of supplied samples as reference
        reference = cellfun(@(vec) vec(ceil(length(vec)/2)), grid_points);
    end
    ref_node = find(all(nodes == reference, 2));
    ref_idxs = ref_node + [0 1 2]*n_nodes;
    % remove reference node(s)
    %LHS(ref_idxs, :) = [];
    %LHS(:, ref_idxs) = [];
    %RHS(ref_idxs, :) = [];
    
    % solve matrix eqn
    beta = lsqminnorm(LHS,RHS);
    
    %% construct X, Y, Z interpolating grids
    % get grid points as single column
    dims_cat = cellfun(@(dim) repmat(dim, n_nodes/length(dim), 1), grid_points, 'UniformOutput', false);
    grid_cat = cat(1, dims_cat{:});
    
    % evaluate beta at gridpoints
    beta_eval = beta * grid_cat;
    
    % reshape to X, Y, Z matrices
    X = reshape(beta_eval(1:n_nodes), size(grid{1}));
    Y = reshape(beta_eval(1+n_nodes:2*n_nodes), size(grid{1}));
    Z = reshape(beta_eval(1+2*n_nodes:end), size(grid{1}));
end
    