%% FACE

slope = [-1.13272388059701,-1.14042158304600,-1.17338351598342,-1.06945782765140,-1.11349186621372,-1.14497930796643,-1.12547082818874,-1.08895423738307,-1.08169633323703,-1.12573961622541,-1.12234550720277,-1.12037814814265,-1.11246037551817,-1.11100024243662,-1.10440750557301,-1.13555410889649,-1.11643316697733,-1.09564556365700,-1.12041645665323,-1.13072891187365,-1.14080189055838,-1.17501817049112,-1.13024261777840,-1.10180013546606];
offset = [108.254000000000,100.192000000000,124.433000000000,101.073000000000,110.140000000000,114.641000000000,108.872000000000,109.739000000000,104.905000000000,111.072000000000,113.721000000000,106.830000000000,114.410000000000,108.207000000000,94.9220000000000,114.409000000000,109.408000000000,110.794000000000,107.684000000000,104.415000000000,105.778000000000,105.691000000000,109.936000000000,106.484000000000] - 5;

slope = mean(slope);
offset = mean(offset)-0.5;

FACES_TO_FLOP = 1;

%% scale bias Rutgers
intercept = 9.5;
%scale = 0.4; % not enough
%scale = 0.45; % not enough
%scale = 0.5; % not enough
%scale = 2.5; % too much
%scale = 1.5; % better, but still a bit much
%scale = 1.25; % really close
scale = 1.25;

%%
if (strcmp(char(java.lang.System.getProperty('user.name')),'Massimo'))
    addpath('/Users/Massimo/Documents/SUPERballController/sbv2_handtuned_flop_controller/faceClassifier');
    addpath('/Users/Massimo/Documents/SUPERballController/sbv2_handtuned_flop_controller/flop_faceDetection');
else
    addpath('/home/jebruce/Projects/MATLAB/sbv2_handtuned_flop_controller/faceClassifier');
    addpath('/home/jebruce/Projects/MATLAB/sbv2_handtuned_flop_controller/flop_faceDetection');
end

load('IMUTrainingRutgers.mat');
loadSymCtrl;


%% Hebi Stuff
hebiStuff;

load('defaultGains.mat');
Group.send('gains', defaultGains);

Group.setFeedbackFrequency(500);
modules.setFeedbackFrequency(500);

%% Go to initial position

Group.setCommandLifetime(0);
Cmd.position = ones(1,24)*NaN;

Cmd.position = ones(1,24)*0.0;
Group.send(Cmd);

clear j;


%% Loop
loopPath = [4, 3, 6, 0, 1, 2]; 



% Since this demo only works one a single loop, 
%check if we are on that loop
checkCorrectFace = 0;
nextFaceIndex = [];

while(~checkCorrectFace)
currFace = DetectCurrentFace(Group);
nextFaceIndex = find(loopPath == currFace)+1;

    if(isempty(nextFaceIndex))
        disp('This demo does not work on the current face!');
        disp('Please rotate the robot to a different face, and press any key to continue.');
        pause();
    else
        disp(['The next face will be: ' num2str(nextFaceIndex)]);
        checkCorrectFace = 1;
    end
end

disp('Press any key to continue...');
pause(); % wait till use is ready

Group.setCommandLifetime(0);

count = 0;

while count ~= FACES_TO_FLOP
    %currFace = DetectCurrentFace(Group);
    nextFaceIndex = find(loopPath == currFace)+1;
    if (nextFaceIndex > 6)
        nextFaceIndex = 1;
    end
    replyFace = loopPath(nextFaceIndex)
    lengths = SymCtrl(replyFace,currFace,2); 
    lengths = (lengths - intercept)*scale + intercept;
    lengths = lengths/10;
    cmdMotorPositions = (100*lengths).*slope + offset;
    
    % Send new positions to motors
    Cmd.position = cmdMotorPositions;
    Group.send(Cmd);
    
    tic;
    while (currFace ~= replyFace)
        currFace = DetectCurrentFace(Group);
        
        % Quit if elapsed time > 30s
        if (toc > 10)
            disp('It seems like something went wrong...');
            count = FACES_TO_FLOP;
            return;
        end
    end  
    replyFace = currFace;
    
    count = count + 1;
end

% %% Loop
% Group.setCommandLifetime(0);
% for i=1:length(time)
%     tStart = tic;
%     while toc(tStart) < time(i)    
%         cmdMotorPositions = (100*lengths(i,:)).*slope + offset;
%         % Send new positions to motors
%         Cmd.position = cmdMotorPositions;
%         Cmd.velocity = ones(1,24)*5;
%         Group.send(Cmd);
%         pause(0.1);
%     end
%     input('press key')
% end

%%
Group.setCommandLifetime(0);
%% Go to initial position

Group.setCommandLifetime(0);
Cmd.position = ones(1,24)*NaN;

Cmd.position = ones(1,24)*0.0;
Group.send(Cmd);

%Group.stopLogFull('LogFormat', 'mat');