% constructs optimal rotational coordinates from Cassie inertial info

% qpos constructed in cassie-mujoco-sim
% M_min must have correct ordering: [g: xyz, r: rpy, LLLLL, RRRRR]

%% variables
% hip roll, hip pitch, hip yaw, knee, ankle
% organized as [left, right] by index
motors = [[1 3 4], [1 3 4]+5];

%% setup
% constrain state space
dims = length(motors);
q_idx = [8 9 10 15 21 22 23 24 29 35]; %motor indices for qpos
q_act = q_idx(motors);

% load data
raw = fileread('6dof_3lvl_sweep.txt');
data_full = jsondecode(raw);
cassie_sweep = data_full{2}; %ignore metadata

% get config data
q_mat = cat(1, cassie_sweep.qpos); %matrix of configs
q_mat = q_mat(:,q_act);
% get samples present for each var
samples = arrayfun(@(i) unique(q_mat(:,i)), 1:dims, 'UniformOutput', false);
num_samples = cellfun(@length, samples);
total_samples = prod(num_samples);
% confirm configuration covers motors
num_uniq = size(unique(q_mat,'rows'),1);
assert(num_uniq == total_samples,...
       ['Sample data must cover all desired configurations; ' num2str(num_uniq) '/' num2str(total_samples) ' covered']);
% trim data to what we'll use
[q_mat, idxs] = unique(q_mat, 'rows');
   
% get mass data
M = cat(3, cassie_sweep.M_min); %pages of mass matrices
m_des = [4:6 6+motors]; %desired (rotation, motor) rows, cols
M = M(m_des, m_des, idxs); %only use matrices for corresp. config data


%% create local connection
% generate LC storage
if numel(num_samples) == 1
    A = cell(num_samples, 1); %only sampling one motor
else
    A = cell(num_samples); %(samples)x(samples)x...x(samples)
end

% iterate through configurations to build LC
for i = 1:size(q_mat,1)
    % use config to know where to store LC matrix
    loc = cell(size(samples));
    for j = 1:length(loc)
        loc{j} = find(samples{j} == q_mat(i,j));
    end
    % generate A with contents of M_min
    A{loc{:}} = get_local_conn(M(:,:,i));
end

% put LC into vector field terms
A = celltensorconvert(A);

%% generate optimal coordinate transform
[grid, X, Y, Z] = optimize_so3(samples, A);
save('cassie.mat', 'motors', 'A', 'samples', 'grid', 'X', 'Y', 'Z');

%% helper functions
% generates local connection from a single M matrix
function A = get_local_conn(M_mat)
    % matrix structure: [rx, ry, rz, L motors, R motors]
    M_gg = M_mat(1:3,1:3); %effect of external rotations on rotation
    M_gr = M_mat(1:3,4:end); %effect of shape changes on rotation
    A = M_gg \ M_gr; % equivalent to inv(M_gg) * M_gr
end