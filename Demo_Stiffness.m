
% %% Hebi Stuff
hebiStuff;

load('defaultGains.mat');

Group.send('gains', defaultGains);

Group.setCommandLifetime(0);
Cmd.position = ones(1,24)*NaN;

Cmd.position = ones(1,24)*0.0;
Group.send(Cmd);

reply = input('Enter (almost) Any Key for Alternating Stiffness Change. Type q to quit.','s');

count = 0;
while reply ~= 'q'
    
    if(count > 1)
        Cmd.position = ones(1,24)*0.0;
    elseif(count == 1)
        Cmd.position = ones(1,24)*-2.0;
    else
        Cmd.position = ones(1,24)*3.0;
    end
    
    Group.send(Cmd);
    count = count+1;
    
    if(count >= 3)
        count = 0;
    end    
    reply = input('Type q to quit.','s');
end