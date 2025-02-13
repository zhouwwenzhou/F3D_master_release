%% Fast Orientation in 3 Degrees of Freedom Motion
%
% Author: Wenzhou Zhou
% Copyright 2025 Wenzhou Zhou. All rights reserved.
% National University of Defense Technology, Changsha, P.R. China.
% Created: July 1, 2024
% Last Modified: Feb. 13, 2025
% Contact: zwz1119@163.com

clear;
clc;
close;
filefolder=fullfile('.\test\');
diroutput=dir(fullfile(filefolder,'.'));
diroutput=diroutput(~[diroutput.isdir]);
[~, idx] = sort([diroutput.datenum]); %red dir pic
diroutput = diroutput(idx);
if isempty(diroutput)
    error('Waring: No No No pic! Examine file path!');
end

pol_image_name={diroutput.name}';
%% Initialize
[row,column]=size(double(imread([filefolder,diroutput(1).name])));  
radius =100;
fx=1280;
fy=1280;
nx=column;
ny=row;
cx=column/2;
cy=row/2;
frequency=1; 

% CHANGSHA
location.longitude = 112.992;
location.latitude = 28.221;
location.altitude = 61.66;



%% 
totalimage=size(pol_image_name,1);
snapshot=zeros(row,column,frequency);
pic_mean=zeros(totalimage,1);
pic_mean1=zeros(totalimage,1);
pic_mean2=zeros(totalimage,1);
pic_mean3=zeros(totalimage,1);
pic_mean4=zeros(totalimage,1);

azimuth_sun_spa=zeros(totalimage,1);
zenith_sun_spa=zeros(totalimage,1);

camera_matrix=[fx 0 cx;
               0 fy cy;
               0 0 1];
%% IMU
yaw   = 0*pi/180;
roll  = 0*pi/180;
pitch = 0*pi/180;

Cnb1=[1 0 0;0 cos(roll) sin(roll);0 -sin(roll) cos(roll)];
Cnb2=[cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch)];
Cnb3=[cos(yaw) sin(yaw) 0;-sin(yaw) cos(yaw) 0;0 0 1];
Cnb = Cnb1*Cnb2*Cnb3;
Clb=Cnb1*Cnb2;
Cbl=Clb';
%% 
roll_cali=0*pi/180;% 
pitch_cali=0*pi/180;%
yaw_cali=0*pi/180;% 

Cpb1=[1 0 0;0 cos(roll_cali) sin(roll_cali);0 -sin(roll_cali) cos(roll_cali)];
Cpb2=[cos(pitch_cali) 0 -sin(pitch_cali);0 1 0;sin(pitch_cali) 0 cos(pitch_cali)];
Cpb3=[cos(yaw_cali) sin(yaw_cali) 0;-sin(yaw_cali) cos(yaw_cali) 0;0 0 1];
Cpb = Cpb1*Cpb2*Cpb3;
Cbp = Cpb';
Cpl = Cpb*Cbl;

%%
yuanxin_matrix=camera_matrix*Cbp*[0 0 1]';
yuanxin=yuanxin_matrix(1:2);

yanmo_REC=zeros(ny,nx);

%% 
 for jj=1:ny
     for kk=1:nx
            if (jj-yuanxin(2))^2+(kk-yuanxin(1))^2<radius^2%75^2 <110^2
                yanmo_REC(jj,kk)=1;
            else
                yanmo_REC(jj,kk)=0;
            end
     end
 end
%%
    tic
for i= 1:totalimage/frequency
    for j=1:frequency
        snapshot=double(imread([filefolder,pol_image_name{(i-1)*frequency+j}]));                
    end
POLregion1=snapshot;


aop_all(i) = F3D(POLregion1,yanmo_REC);

%% Cal heading 
% pol_image_date = {diroutput.date}';
% date_str = pol_image_date{i}; 
% date = get_time( date_str );
% date.UTC = 8;
% sun = sun_position(date, location);  
% azimuth_sun_spa(i)=sun.azimuth;
% zenith_sun_spa(i)=sun.zenith;
% aop_all(i)=aop_all(i)+sun.azimuth;

fprintf('%d of %d time=%d s\n ',i,totalimage,toc);
end
fprintf('TCPF=%d s\n ',toc/totalimage);
 

%% Error evaluation 

 every_turn_angle = 10;


 angle_all_deg=(aop_all)*180/pi;
 angle_all_deg_c=yaw_to_360(angle_all_deg);
 angle_all_deg_c= angle_all_deg_c-angle_all_deg_c(1);
 
 turn_truth_angle = [0:every_turn_angle:180];
 error_absolute=abs(angle_all_deg_c)-turn_truth_angle';
 error_absolute=error_absolute-mean(error_absolute);
 rmse_absolute =sqrt(mean((error_absolute ).^2));
 fprintf('rmse_absolute  = %f��\n', rmse_absolute);












% In an English-speaking environment, this function might throw an error. 
% The function is used to read the timestamp of an image. 
% You can comment out the relevant code, as it has little impact on the results.
% function date = get_time( date_str )
%      date.year = str2num(date_str(end-12:end-8)); %% 
%      if date_str(5)=='��'
%        date.month = str2num(date_str(4));
%      end
%      if date_str(6)=='��'
%        date.month = str2num(date_str(4:5));
%      end
%      date.day = str2num(date_str(1:2));
%      date.hour = str2num(date_str(end-7:end-6));
%      date.min = str2num(date_str(end-4:end-3));
%      date.sec = str2num(date_str(end-1:end)); 
% end





