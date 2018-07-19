function cable_command = CableCommand( i_prev_tri, i_cur_tri, dir_change_code, cable_vector, cable_maps, imaps_of_crossings )

% Triangle IDs beginning from 1

% For dir change codes, "backward" refers to re-crossing the
%     same edge that has just been crossed recently.

% Dir change codes for 3-cable i_cur_tri:
% 1 = right-handed forward
% 2 = left-handed forward
% 3 = backward

% Dir change codes for 2-cable i_cur_tri:
% 1 = forward
% 2 = backward

% Indices in imported data begin from 0
% To use tri ids as key in python dict, subtract 1
crossing_key = sprintf('c%04d',(i_prev_tri)*100 + i_cur_tri);
i_maps       = imaps_of_crossings.(crossing_key);

% Using indices from python in matlab; add 1
i_map    = 1 + i_maps(dir_change_code);
i_cables = 1 + cable_maps(i_map,:);

% Apply the map to permute the cable values appropriately
cable_command = cable_vector(i_cables);

end
