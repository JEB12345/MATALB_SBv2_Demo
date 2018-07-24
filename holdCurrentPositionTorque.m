%% Hebi Stuff
hebiStuff;

%% Keep current position

disp('Press a key to keep current position');
pause();
fbk = Group.getNextFeedback;
Cmd.position = fbk.position;
Cmd.effort = [];
Group.send(Cmd);


%% Keep current torque

disp('Press a key to keep current torque');
pause();
fbk = Group.getNextFeedback;
Cmd.position = ones(1,24)*NaN;
Cmd.effort = fbk.effort;
Group.send(Cmd);