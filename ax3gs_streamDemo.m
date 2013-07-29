% Copyright (c) 2009-2012, Newcastle University, UK.
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are met: 
% 1. Redistributions of source code must retain the above copyright notice, 
%    this list of conditions and the following disclaimer.
% 2. Redistributions in binary form must reproduce the above copyright notice, 
%    this list of conditions and the following disclaimer in the documentation 
%    and/or other materials provided with the distribution.
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
% POSSIBILITY OF SUCH DAMAGE. 
%
% Props to:
% http://www.mathworks.co.uk/matlabcentral/fileexchange/31958-serialdatastream/content/serialDataStream.m
% Matthew Hopcroft
% mhopeng@gmail.com
%
% http://www.x-io.co.uk/
% Seb Madgwick  
%
% Mar2012 v1.1 update description
% Jun2011 v1.0
%
% Cassim Ladha Nov 2012
%
% USE:
% 1 - Config the serial port, search "INSERT YOUR SERIAL PORT HERE"
% 2 - Decide on 2d OR 3d plot, search "CHOOSE PLOT TYPE"
function serialDataStream

% Define the Serial PORT (BTooth and USB are the same process)
if(ispc)
    % INSERT YOUR SERIAL PORT HERE
    USBport = 'COM9';
else    
    % availableDevices = ls('/dev/tty.CWA3*'); % OS X
    % INSERT YOUR SERIAL PORT HERE
    USBport = '/dev/tty.CWA3-74-SPP';
end

% setup the port params
obj=serial(USBport);
obj.BaudRate=19200;
obj.Terminator='CR/LF';
%obj.ByteOrder='bigEndian';
obj.ReadAsyncMode='continuous';
obj.DataBits= 8;
obj.StopBits = 1;
obj.Parity= 'none';
obj.RequestToSend= 'off';
obj.FlowControl = 'none';

%setup the interrupt 
%  The "BytesAvailableFcn" function will be called whenever
%  BytesAvailableFcnCount number of bytes have been received from the USB
%  device.
%  The name of the BytesAvailableFcn function in this example is
%  "getNewData", and it has one additional input argument ("arg1" - not actually used) .

obj.InputBufferSize=2^24; % in bytes
obj.BytesAvailableFcnMode='byte';
Fs = 100;
FrameRate = 25;
obj.BytesAvailableFcnCount=((6*7+1 +2)*(Fs/FrameRate)); %upto 32bytes per sample
arg1=0;
obj.BytesAvailableFcn = {@getNewData,arg1};

% these vars are used to pass data between interupt and main loop
obj.UserData.newData=[];
obj.UserData.isNew=0;

% startup sequence for the AX9s
% NOTE: the terminator character set above will be appended.

if strcmp(obj.Status,'closed'), fopen(obj); end
pause(1);
fprintf(obj,'rate x 1 100');
response = fscanf(obj);
fprintf(response);
fprintf(obj,'mode 2');
response = fscanf(obj);
fprintf(response);

% Variables
global PLOTLOOP; PLOTLOOP=1; % A global variable is used to exit the loop

global dataBuffer; dataBuffer = [0];
% CHOOSE PLOT TYPE
plotType = 0; % 0= 2D plot, 1= 3D plot

% Setup IMU Variables
quaternion = [1 0 0 0];

%AHRS = MahonyAHRS('SamplePeriod', 1/Fs, 'Kp', 10, 'Ki',0.025); %measured fs roughly!!!
AHRS = MadgwickAHRS('SamplePeriod', 1/Fs, 'Quaternion', quaternion, 'Beta',1); %measured fs roughly!!!

newData=[];

if(plotType == 0)
    plot_window_size = 50;
    time_plot=0; % used for x-axis in plotting
    k=0; % used for graph scrolling

    % Setup axis for the graph plot
    sensorPlot = figure('Name', 'Sensor Data','Visible','off');

    axis(1) = subplot(311);
    hold on;
    set(axis(1),'xlim',[0 plot_window_size]);
    set(axis(1),'ylim',[-30 30]);
    plot_gyro_x = plot(axis(1),0,0,'r-');
    plot_gyro_y = plot(axis(1),0,0,'g-');
    plot_gyro_z = plot(axis(1),0,0,'b-');
    gyro_data=0;
    xlabel('Time (s)');
    ylabel('Angular rate (deg/s)');
    title('Gyroscope');
    plot_gyro_legend = legend('X', 'Y', 'Z');

    axis(2) = subplot(312);
    hold on;
    set(axis(2),'xlim',[0 plot_window_size]);
    set(axis(2),'ylim',[-3 3]);
    plot_acc_x = plot(axis(2),0,0,'r-');
    plot_acc_y = plot(axis(2),0,0,'g-');
    plot_acc_z = plot(axis(2),0,0,'b-');
    acc_data=0;
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    title('Accelerometer');
    legend('X', 'Y', 'Z');

    axis(3) = subplot(313);
    hold on;
    set(axis(3),'xlim',[0 plot_window_size]);
    set(axis(3),'ylim',[-180 180]);
    plot_euler_x = plot(axis(3),0,0,'r-');
    plot_euler_y = plot(axis(3),0,0,'g-');
    plot_euler_z = plot(axis(3),0,0,'b-');
    title('Euler angles');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('\phi', '\theta', '\psi');

    linkaxes(axis, 'x');

    % This allows us to stop the test by pressing a key
    set(sensorPlot,'KeyPressFcn', @stopStream);

end

if(plotType ==1)
    planePlot = figure('Name', 'Sensor Data','Visible','off');
    % This allows us to stop the test by pressing a key
    set(planePlot,'KeyPressFcn', @stopStream); 
end

% Send commands to the device to start the data stream.
fprintf(obj,'stream 1');
response = fscanf(obj);
fprintf(response);


while PLOTLOOP
    
    % wait until we have new data
    if obj.UserData.isNew==1
        
        % get the data from serial port object (data will be row-oriented)    
        newData=obj.UserData.newData';
        
        % indicate that data has been read before calculations start so can
        % background interrupt
        obj.UserData.isNew=0;
        numSamples = length(newData);
        if(~(rem(numSamples,6)))
            numSamples = numSamples/6;
            
            for(count= 1:numSamples)
                accel = newData(1,((count*6)-5):(count*6)-3)./4096;
                gyro = (newData(1,((count*6)-2):(count*6)).*2293.8)./32768;

                AHRS.UpdateIMU(deg2rad(gyro), accel);
            end
            
        end
    
        quaternion= AHRS.Quaternion;
    
        %euler = rad2deg(quatern2euler(quaternConj(quaternion)));	% use conjugate for sensor frame relative to Earth
        euler = (quatern2euler(quaternConj(quaternion)));	% use conjugate for sensor frame relative to Earth

        roll = deg2rad (euler(1,1));
        pitch = deg2rad (euler(1,2));
        yaw = deg2rad (euler(1,3));

        if(plotType ==0)
            plot_index = rem(k, plot_window_size)+1;
            time_plot(plot_index) = plot_index;

            sensorPlot;

            axis(1);
            gyro_data(1,plot_index) = gyro(1,1);
            gyro_data(2,plot_index) = gyro(1,2);
            gyro_data(3,plot_index) = gyro(1,3);
            set(plot_gyro_x,'XData',time_plot,'YData',gyro_data(1,:));
            set(plot_gyro_y,'XData',time_plot,'YData',gyro_data(2,:));
            set(plot_gyro_z,'XData',time_plot,'YData',gyro_data(3,:));

            axis(2);
            acc_data(1,plot_index) = accel(1,1);
            acc_data(2,plot_index) = accel(1,2);
            acc_data(3,plot_index) = accel(1,3);
            set(plot_acc_x,'XData',time_plot,'YData',acc_data(1,:));
            set(plot_acc_y,'XData',time_plot,'YData',acc_data(2,:));
            set(plot_acc_z,'XData',time_plot,'YData',acc_data(3,:));

            axis(3);
            euler_data(1,plot_index) = euler(1,1);
            euler_data(2,plot_index) = euler(1,2);
            euler_data(3,plot_index) = euler(1,3);
            set(plot_euler_x,'XData',time_plot,'YData',rad2deg(euler_data(1,:)));
            set(plot_euler_y,'XData',time_plot,'YData',rad2deg(euler_data(2,:)));
            set(plot_euler_z,'XData',time_plot,'YData',rad2deg(euler_data(3,:)));

            k = k+1; %increment the time slider for next time
            set(sensorPlot,'Visible','on');        
            drawnow
        end

        if(plotType == 1)
           planePlot;
           plot3D (0,0,0,(euler(1,1)+180),euler(1,2),euler(1,3),0.1,0, 'A-10');
           set(planePlot,'Visible','on');        
           drawnow
        end

    end
    
    % The loop will exit when the user presses return, using the
    %  KeyPressFcn of the plot window
    
end

%% 6. Finish & Cleanup
% Add whatever commands are required for closing your device.

% Send commands to the device stop the data transmission
fprintf(obj,'stream 0');
pause(1);
response = fscanf(obj);
fprintf(response);
% flush the input buffer
ba=get(obj,'BytesAvailable');
if ba > 0, fread(obj,ba); end

% Close the serial port
fclose(obj);
delete(obj);


return


%% Data Processing Function
function getNewData(obj,event,arg1)
% GETNEWDATA processes data that arrives at the serial port.
%  GETNEWDATA is the "BytesAvailableFcn" for the serial port object, so it
%  is called automatically when BytesAvailableFcnCount bytes of data have
%  been received at the serial port.

% Read the data from the port.
% For binary data, use fread. You will have to supply the number of bytes
%  to read and the format for the data. See the MATLAB documentation.
% For ASCII data, you might still use fread with format of 'char', so that
%  you do not have to handle the termination characters.
%[Dnew, Dcount, Dmsg]=fread(obj);

global dataBuffer;

avail = obj.BytesAvailable;
if (avail <= 0) 
    return; 
end
dataBuffer = [ dataBuffer; (fread(obj, avail, 'uint8')) ];

lineStart = 1;
consume = 0;
for lineEnd=1:length(dataBuffer)
    if dataBuffer(lineEnd) == 10
        line = char(dataBuffer(lineStart:lineEnd));
        try
            % Return the data to the main loop for plotting/processing
            [Dnew, Dcount] = sscanf(line,'%d,%d,%d,%d,%d,%d');
        catch
            fprintf(1, 'Error\n');
        end
        
        if obj.UserData.isNew==0
            % indicate that we have new data
            obj.UserData.isNew=1; 
            obj.UserData.newData=Dnew;
        else
            % If the main loop has not had a chance to process the previous batch
            % of data, then append this new data to the previous "new" data
            obj.UserData.newData=[obj.UserData.newData
                                    Dnew];
        end
        
        consume = lineEnd;
        lineStart = (lineEnd + 1); % gets rid of 'cr' 'cr/lf'
    end
end

% Trim the buffer of any lines consumed
if (consume > 0)
    dataBuffer = dataBuffer(consume:length(dataBuffer));
end


return


%% Loop Control Function
function [] = stopStream(src,evnt)
% STOPSTREAM is a local function that stops the main loop by setting the
%  global variable to 0 when the user presses return.
global PLOTLOOP;

if strcmp(evnt.Key,'return')
    PLOTLOOP = 0;
    fprintf(1,'Return key pressed.\r\n');
end


return

