% Loads the data used for the symmetry-based control scheme
% Returns a function handle that binds this input data

load('symctrl_data.mat')

SymCtrl = @(i_prev_tri, i_cur_tri, dir_change) CableCommand(i_prev_tri, i_cur_tri, dir_change, CABLE_VECTOR, CABLE_MAPS, IMAPS_OF_CROSSINGS)
