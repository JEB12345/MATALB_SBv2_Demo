clear all

%%

SBtransform = [
    19 20 21 15 8 2 14 9 4 13 7 6 1 18 12 3 17 11 5 16 10 23 24 22;
    18 3 13 22 4 15 7 14 20 9 17 1 21 2 11 8 16 24 10 19 5 12 23 6 
    ];

slope = [-1.13272388059701,-1.14042158304600,-1.17338351598342,-1.06945782765140,-1.11349186621372,-1.14497930796643,-1.12547082818874,-1.08895423738307,-1.08169633323703,-1.12573961622541,-1.12234550720277,-1.12037814814265,-1.11246037551817,-1.11100024243662,-1.10440750557301,-1.13555410889649,-1.11643316697733,-1.09564556365700,-1.12041645665323,-1.13072891187365,-1.14080189055838,-1.17501817049112,-1.13024261777840,-1.10180013546606];
offset = [108.254000000000,100.192000000000,124.433000000000,101.073000000000,110.140000000000,114.641000000000,108.872000000000,109.739000000000,104.905000000000,111.072000000000,113.721000000000,106.830000000000,114.410000000000,108.207000000000,94.9220000000000,114.409000000000,109.408000000000,110.794000000000,107.684000000000,104.415000000000,105.778000000000,105.691000000000,109.936000000000,106.484000000000] + 5 ;

[v,i]=sort(SBtransform(1,:));
SBtransformSort = SBtransform(:,i);

unpackingLenTen = csvread('cyl_horiz_pack_len_ten_prestress.csv');

hebiStuff;

Cmd.position = ones(1,24)*0.0;
Group.send(Cmd);

numModules = Group.getNumModules;

disp('Press any key to continue...')
pause()


%% Stiffness and Cable Gains for Conversion
stiffness = 6000.0;


%% Impedance Controller
%% Make Group, Command Structure, and Feedback

% %% Set Motor Gains
gains = Group.getGains();
gains.controlStrategy = ones(1,Group.getNumModules)*3;
gains.positionKp = ones(1,Group.getNumModules)*1;
gains.effortKp = ones(1,Group.getNumModules)*0.2;
gains.effortKi = ones(1,Group.getNumModules)*0.01;

gains.effortMaxOutput = ones(1,Group.getNumModules)*3.0;
gains.effortMinOutput = ones(1,Group.getNumModules)*-3.0;

Group.send('gains', gains);

%% Packing Loop

disp('Press any key to pack the robot...');

for i = 1 : (size(unpackingLenTen,1) - 180)
    
    current_lengths = unpackingLenTen(i,1:24);
    current_tensions = unpackingLenTen(i,25:end);
    
    current_tensions(current_tensions > 400.0) = 400.0;
    current_moments = current_tensions*0.006;
    current_restLengths = current_lengths;
    
    newRestLengths(SBtransformSort(2,:)) = current_restLengths;
    newMoments(SBtransformSort(2,:)) = current_moments;
    newMoments;
        
    cmdMotorPositions = (100*newRestLengths).*slope + offset;

    %% While Loop
    for stupid=1:1
        Cmd.position = cmdMotorPositions;
        Cmd.effort = newMoments*0.2;
        Group.send(Cmd);
        pause(0.025)
    end

end

disp('DOWN');

fbk = Group.getNextFeedback();

% turns is the number of times that the absolute encoder rolled over
% modPos is the position relative to the rolled over absolute encoder
for i = 1:numModules 
    turns(i,1) = floor((fbk.position(i) + 8*pi)/(16*pi));
    modPos(i,1) = mod(fbk.position(i) + 8*pi, 16*pi) - 8*pi;
end

% Save current values to file, just in case...
fileID = fopen('LastValues.txt','w');
fprintf(fileID,'%s\n',datestr(now));
fprintf(fileID, '\nPositions\n');
fprintf(fileID,'%f\n',fbk.position);
fprintf(fileID, '\nturns\n');
fprintf(fileID,'%f\n',turns);
fprintf(fileID, '\nmodPos\n');
fprintf(fileID,'%f\n',modPos);
fclose(fileID);

%% Oscillate around current position to place caps

disp('Press a key to oscillate.');
pause();

DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
    'String', 'Stop oscillating', ...
    'Callback', 'delete(gcbf)');

pause(1)

freqHz = 0.3;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 50 );    % [rad]

fbk = Group.getNextFeedback;
currentPos = fbk.position;
         
timer = tic();

% Lower the feedback frequency to avoid Java errors
currFbkFreq = Group.getFeedbackFrequency;
Group.setFeedbackFrequency(100);

while (ishandle(H))
    fbk = Group.getNextFeedback;
    
    for m = 1:24
        newPos(1,m) = currentPos(1,m) + amp * sin(freq * toc(timer));
    end
     
    Cmd.position = newPos;
    
    Group.send(Cmd); 
    pause(0.025)
end

% Set back the feedback frequency
Group.setFeedbackFrequency(currFbkFreq);

clearvars freqHz freq amp DlgH H timer currentPos newPos currFbkFreq;
%% Reboot the motor!
disp('Reboot the motor, wait a few seconds, and press any key to continue')
pause();

%% Setup
clearvars -except turns modPos numModules SBtransformSort slope offset unpackingLenTen;
close all;

hebiStuff;

%% Check if all motors are connected

if Group.getNumModules ~= numModules
    disp('Number of modules does not match!')
    disp('Please CTRL+C out of the program')
    pause()
end

%% Set a new reference position 

% Do not manually move the motors after this point, to avoid losing the
% original position
fbk = Group.getNextFeedback(); 

% Check if we possibly rolled over to the next absolute encoder range
% e.g. we recorded a modPos = 24, but after reboot the position is -24, it it
% possible that the motor turned just a few radiants and rolled over at
% 8*pi (25.1327).
for i = 1:numModules
    if (abs((mod(fbk.position(i) + 8*pi, 16*pi) - 8*pi) - modPos(i))) > (8*pi) %Max error range of 8*pi
        disp(['Current Turns value for motor ' num2str(i) ' = ' num2str(turns(i))])
        if (mod(fbk.position(i) + 8*pi, 16*pi) - 8*pi) < 0 % we rolled over to the next range
            turns(i) = turns(i) + 1;
        elseif (mod(fbk.position(i) + 8*pi, 16*pi) - 8*pi) > 0 % we rolled back to the previous range
            turns(i) = turns(i) - 1;
        end

        disp(['New Turns value for motor ' num2str(i) ' = ' num2str(turns(i))])
    end
end

% Set back the reference position of the absolute encoder to a multiple of 16*pi
for i = 1:numModules
    newValue(1,i) = (mod(fbk.position(i) + 8*pi, 16*pi) - 8*pi) + 16*pi*turns(i);
end

Group.send('ReferencePosition', newValue);

pause(0.5);

%% Keep current position and remove caps

disp('Press a key to keep current position');
pause();
fbk = Group.getNextFeedback;
Cmd.position = fbk.position;
Cmd.effort = [];
Group.send(Cmd);


%% Unpacking Loop

disp('Press any key to start unpacking')
pause();

for i = (size(unpackingLenTen,1) - 180) : -1 : 1
    
    current_lengths = unpackingLenTen(i,1:24);
    current_tensions = unpackingLenTen(i,25:end);
    
    current_tensions(current_tensions > 400.0) = 400.0;
    current_moments = current_tensions*0.006;
    current_restLengths = current_lengths;
    
    newRestLengths(SBtransformSort(2,:)) = current_restLengths;
    newMoments(SBtransformSort(2,:)) = current_moments;
    newMoments;
        
    cmdMotorPositions = (100*newRestLengths).*slope + offset;
    %pause
    %% While Loop
    for stupid=1:1
        Cmd.position = cmdMotorPositions;
        Cmd.effort = newMoments*0.2;
        Group.send(Cmd);
        pause(0.025)
    end

end

%%
Group.setCommandLifetime(0);

Cmd.position = ones(1,24)*0.0;
Group.send(Cmd);