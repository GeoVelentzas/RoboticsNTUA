%% ********** kinematic simulation of 3-dof staubli robot *************%%%
clear all; 
close all; 

%%%%%%%%%%%%%%%%%%%%%%%% lengths of links % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
l(1) = 420;         % in mm 
l(2) = 450;         % in mm
l(3) = 650+85+100;  % l3+l4+le in mm

%%%%%%%%%%%%%%%%%%%%%%%%% sampling frequency %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.001;         % typical 1 msec 



%% ******************** Motion Planning **********************************


%%%%%%%%%%%%%%%%%%%% desired time at each section %%%%%%%%%%%%%%%%%%%%%%%%
% time to reach the starting point in sec
T1 = 2; 

% time to complete the straight line motion in sec
T2 = 5; 

% time to stop to a desired end point sec
T3 = 2;      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%% desired positions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial position of the endpoint
px0 = 600;  py0 = 750;    pz0 = 1000;

% position of the starting line point
px1 = 725;  py1 = 825;  pz1 = 900;

% position of the ending line point
px2 = 1000; py2 =-450;  pz2 = 150;

% position of the point to stop the movement
px3 = 950;  py3 = -560;    pz3 = 100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%% desired velocities %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial velocities
vx0 = 0;    vy0 = 0;    vz0 = 0;

% computation of desired velocities when entering the line
vx1 = (px2-px1)/T2;  vy1 = (py2-py1)/T2;  vz1 = (pz2-pz1)/T2;

% velocities when exiting the line
vx2 = vx1;  vy2 = vy1;  vz2 = vz1;

% final velocities (stop)
vx3 = 0;    vy3 = 0;    vz3 = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%% desired accelerations %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial desired acceleration
gx0 = 0;    gy0 = 0;    gz0 = 0;

% acceleration when entering the straight line
gx1 = 0;    gy1 = 0;    gz1 = 0;

% acceleration when exiting the straight line
gx2 = 0;    gy2 = 0;    gz2 = 0;

% acceleration at the end point
gx3 = 0;    gy3 = 0;    gz3 = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%% POLYNOMIAL INTERPOLATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vector of timesteps
t1 = 0:dt:T1; 

% By constructing the linear system A*S=B for every dimention in a way such
% that S will contain the coefficients fo the 5th degree polynomial for
% having a continous (as a function in time) acceleration, meaning we can
% control jerk. 
A1 = [1       0       0       0       0       0;
      0       1       0       0       0       0;
      0       0       1       0       0       0;
      1       T1     T1^2    T1^3    T1^4    T1^5;
      0       1      2*T1   3*T1^2  4*T1^3  5*T1^4;
      0       0       2      6*T1  12*T1^2  20*T1^3];

B1x = [px0;    vx0;   gx0;     px1;     vx1;    gx1];
B1y = [py0;    vy0;   gy0;     py1;     vy1;    gy1];
B1z = [pz0;    vz0;   gz0;     pz1;     vz1;    gz1];

S1x=linsolve(A1,B1x);
S1y=linsolve(A1,B1y);
S1z=linsolve(A1,B1z);

% S1 now contains the coefficients of the 5th degree polynomial
a0x=S1x(1); a1x=S1x(2); a2x=S1x(3); a3x=S1x(4); a4x=S1x(5); a5x=S1x(6);
a0y=S1y(1); a1y=S1y(2); a2y=S1y(3); a3y=S1y(4); a4y=S1y(5); a5y=S1y(6);
a0z=S1z(1); a1z=S1z(2); a2z=S1z(3); a3z=S1z(4); a4z=S1z(5); a5z=S1z(6);

% computation of the desired positions when approaching the straight line
pdx1 = a0x+a1x*t1+a2x*t1.^2+a3x*t1.^3+a4x*t1.^4+a5x*t1.^5;
pdy1 = a0y+a1y*t1+a2y*t1.^2+a3y*t1.^3+a4y*t1.^4+a5y*t1.^5;
pdz1 = a0z+a1z*t1+a2z*t1.^2+a3z*t1.^3+a4z*t1.^4+a5z*t1.^5;

% computation of the desired velocities when approaching the straight line
vdx1 = a1x+2*a2x*t1+3*a3x*t1.^2+4*a4x*t1.^3+5*a5x*t1.^4;
vdy1 = a1y+2*a2y*t1+3*a3y*t1.^2+4*a4y*t1.^3+5*a5y*t1.^4;
vdz1 = a1z+2*a2z*t1+3*a3z*t1.^2+4*a4z*t1.^3+5*a5z*t1.^4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%% 	MOTION PLANNING DURING WELDING  %%%%%%%%%%%%%%%%%%%%%%%
% vector of timesteps during welding
t2=dt:dt:T2;  % notice we start from dt here

% vector of desired positions during welding
pdx2 = px1+vx1*t2;
pdy2 = py1+vy1*t2;
pdz2 = pz1+vz1*t2;

% vector of desired velocities during welding
vdx2(1:length(t2)) = vx1;
vdy2(1:length(t2)) = vy1;
vdz2(1:length(t2)) = vz1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%% POLYNOMIAL INTERPOLATION DURING EXIT %%%%%%%%%%%%%%%%%%%%%%
% vector of timesteps
t3=dt:dt:T3;

% Again we construct a linear system to solve for every dimension
A2 = [1       0       0       0       0       0;
      0       1       0       0       0       0;
      0       0       1       0       0       0;
      1       T3     T3^2    T3^3    T3^4    T3^5;
      0       1      2*T3   3*T3^2  4*T3^3  5*T3^4;
      0       0       2      6*T3  12*T3^2  20*T3^3];

B2x = [px2;    vx2;   gx2;     px3;     vx3;    gx3];
B2y = [py2;    vy2;   gy2;     py3;     vy3;    gy3];
B2z = [pz2;    vz2;   gz2;     pz3;     vz3;    gz3];

S2x=linsolve(A2,B2x);
S2y=linsolve(A2,B2y);
S2z=linsolve(A2,B2z);

% S2 now has the coefficients of the 5th degree polynomial
b0x=S2x(1); b1x=S2x(2); b2x=S2x(3); b3x=S2x(4); b4x=S2x(5); b5x=S2x(6);
b0y=S2y(1); b1y=S2y(2); b2y=S2y(3); b3y=S2y(4); b4y=S2y(5); b5y=S2y(6);
b0z=S2z(1); b1z=S2z(2); b2z=S2z(3); b3z=S2z(4); b4z=S2z(5); b5z=S2z(6);

% computation of the desired positions when exiting
pdx3 = b0x+b1x*t3+b2x*t3.^2+b3x*t3.^3+b4x*t3.^4+b5x*t3.^5;
pdy3 = b0y+b1y*t3+b2y*t3.^2+b3y*t3.^3+b4y*t3.^4+b5y*t3.^5;
pdz3 = b0z+b1z*t3+b2z*t3.^2+b3z*t3.^3+b4z*t3.^4+b5z*t3.^5;

% computation of the desired velocities when exiting
vdx3 = b1x+2*b2x*t3+3*b3x*t3.^2+4*b4x*t3.^3+5*b5x*t3.^4;
vdy3 = b1y+2*b2y*t3+3*b3y*t3.^2+4*b4y*t3.^3+5*b5y*t3.^4;
vdz3 = b1z+2*b2z*t3+3*b3z*t3.^2+4*b4z*t3.^3+5*b5z*t3.^4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%% CONCATENATION OF THE DESIRED POSITIONS AND VELOCITIES %%%%%%%%%%%

% vectors of the desired position at every timestep
pdx = [pdx1 pdx2 pdx3];
pdy = [pdy1 pdy2 pdy3];
pdz = [pdz1 pdz2 pdz3];

% vectors of the desired velocities at every timestep
vdx = [vdx1 vdx2 vdx3];
vdy = [vdy1 vdy2 vdy3];
vdz = [vdz1 vdz2 vdz3];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**************************************************************************






%% *********  USER INTERFACE FOR CHOICE OF DESIRED SOLUTION ***************
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

first=0;  second=0;  third=0;  fourth=0;

%scrsz = get(0,'ScreenSize');
%figure('Position',[(scrsz(3)/2)-150 (scrsz(4)/2)-400 300 800]); 

h1 = uicontrol('Style', 'pushbutton', 'String', 'front-elbow up',...
   'Position', [10 310 120 70],...
   'Callback', 'first=1;');
h2 = uicontrol('Style', 'pushbutton', 'String', 'front-elbow down',...
   'Position', [10 210 120 70],...
   'Callback', 'second=1;');
h3 = uicontrol('Style', 'pushbutton', 'String', 'back-elbow down',...
   'Position', [10 110 120 70],...
   'Callback', 'third=1;');
h4 = uicontrol('Style', 'pushbutton', 'String', 'back-elbow up',...
   'Position', [10 10 120 70],...
   'Callback', 'fourth=1;');


p1=subplot('position',[0.45 0.65 0.5 0.15]);
hold on; box on;
title('\fontsize{18} {\color{blue}Choose Solution                   }');
plot([10 10],[10 50],'linewidth',2);  plot(10,50,'o','linewidth',2);
plot([10 50],[50 80],'linewidth',2);  plot(50,80,'o','linewidth',2);
plot([50 90],[80 50],'linewidth',2);  plot(90,50,'r*','linewidth',2);
plot([0 10],[10 20],'linewidth',2);   plot([0 10],[10 10],'linewidth',2);
axis([-15 100 0 100]);

p2=subplot('position',[0.45 0.45 0.5 0.15]);
hold on; box on;
plot([10 10],[10 50],'linewidth',2);  plot(10,50,'o','linewidth',2);
plot([10 50],[50 20],'linewidth',2);  plot(50,20,'o','linewidth',2);
plot([50 90],[20 50],'linewidth',2);  plot(90,50,'r*','linewidth',2);
plot([0 10],[10 20],'linewidth',2);   plot([0 10],[10 10],'linewidth',2);
axis([-15 100 0 100]);

p3=subplot('position',[0.45 0.25 0.5 0.15]);
hold on; box on;
plot([10 10],[10 50],'linewidth',2);  plot(10,50,'o','linewidth',2);
plot([10 50],[50 20],'linewidth',2);  plot(50,20,'o','linewidth',2);
plot([50 90],[20 50],'linewidth',2);  plot(90,50,'r*','linewidth',2);
plot([20 10],[10 20],'linewidth',2);   plot([10 20],[10 10],'linewidth',2);
axis([-15 100 0 100]);

p4=subplot('position',[0.45 0.05 0.5 0.15]);
hold on; box on;
plot([10 10],[10 50],'linewidth',2);  plot(10,50,'o','linewidth',2);
plot([10 50],[50 80],'linewidth',2);  plot(50,80,'o','linewidth',2);
plot([50 90],[80 50],'linewidth',2);  plot(90,50,'r*','linewidth',2);
plot([20 10],[10 20],'linewidth',2);  plot([10 20],[10 10],'linewidth',2);
axis([-15 100 0 100]);


 while (first==0)&&(second==0)&&(third==0)&&(fourth==0)
 pause(0.05);
 end
close;




%% *****************  REVERSE KINEMATIC MODEL ****************************
% computation of desired angles for the joints


% useful variables
rxy = (pdx.^2+pdy.^2).^(1/2);
dxy = (pdx.^2+pdy.^2);

if first==1;
    %1st solution , (front,elbow up)
    qd1= atan2(pdy,pdx);
    c1 = cos(qd1);
    s1 = sin(qd1);
    qd3 =acos(((pdz-l(1)).^2+dxy-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
    s3 = sin(qd3);
    qd2 = atan2(rxy,(pdz-l(1)))-asin(l(3)*s3./((dxy+(pdz-l(1)).^2).^(1/2)));
    c2 = cos(qd2);
    s2 = sin(qd2);
    c23 = cos(qd2+qd3);
    s23 = sin(qd2+qd3);

elseif second==1;
    %2nd solution , (front,elbow down)
    qd1= atan2(pdy,pdx);
    c1 = cos(qd1);
    s1 = sin(qd1);
    qd3 = -acos(((pdz-l(1)).^2+dxy-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
    s3 = sin(qd3);
    qd2 = atan2(rxy,(pdz-l(1)))-asin(l(3)*s3./((dxy+(pdz-l(1)).^2).^(1/2)));
    c2 = cos(qd2);
    s2 = sin(qd2);
    c23 = cos(qd2+qd3);
    s23 = sin(qd2+qd3);

elseif third==1;
    %3rd solution (back,elbow down)
    qd1 = -pi+atan2(pdy,pdx);
    c1 = cos(qd1);
    s1 = sin(qd1);
    qd3 = acos(((pdz-l(1)).^2+dxy-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
    s3 = sin(qd3);
    qd2 = atan2(-rxy,(pdz-l(1)))-asin(l(3)*s3./((dxy+(pdz-l(1)).^2).^(1/2)));
    c2 = cos(qd2);
    s2 = sin(qd2);
    c23 = cos(qd2+qd3);
    s23 = sin(qd2+qd3);

elseif fourth==1;
    %4th solution (back,elbow up)
    qd1 = -pi+atan2(pdy,pdx);
    c1 = cos(qd1);
    s1 = sin(qd1);
    qd3 = -acos(((pdz-l(1)).^2+dxy-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
    s3 = sin(qd3);
    qd2 = atan2(-rxy,(pdz-l(1)))-asin(l(3)*s3./((dxy+(pdz-l(1)).^2).^(1/2)));
    c2 = cos(qd2);
    s2 = sin(qd2);
    c23 = cos(qd2+qd3);
    s23 = sin(qd2+qd3);
end
%**************************************************************************




   
%% ********************* KINEMATIC MODEL ********************************** 

% position of the first joint
px1d(1:length(c1)) = 0;
py1d(1:length(c1)) = 0;
pz1d(1:length(c1)) = l(1);

% position of the second joint
px2d = c1.*(l(2)*s2);
py2d = s1.*(l(2)*s2);
pz2d = l(1) + l(2)*c2;

% position of end effector
px3d = c1.*(l(2)*s2+l(3)*s23); 
py3d = s1.*(l(2)*s2+l(3)*s23);
pz3d = l(1) + l(2)*c2 + l(3)*c23;
%**************************************************************************



%% ******************** REVERSE DYNAMIC MODEL *****************************
% computation of the desirer angular velocities of the joints


%%%%%%%%%%%%%%%% JACOBIAN AND ANGULAR VELOCITIES %%%%%%%%%%%%%%%%%%%%%%%%%%

% length of the total vector time length
kmax = length(t1)+length(t2)+length(t3);

% initializations of variables
qd_1 = zeros([1 kmax]);
qd_2 = zeros([1 kmax]);
qd_3 = zeros([1 kmax]);

for i=1:1:kmax 
    
    % computation of Jacobian at every timestep
    Jac=[-s1(i).*(l(2)*s2(i)+l(3)*s23(i))   c1(i).*(l(2)*c2(i)+l(3)*c23(i))    l(3)*c1(i).*c23(i);
          c1(i).*(l(2)*s2(i)+l(3)*s23(i))   s1(i).*(l(2)*c2(i)+l(3)*c23(i))    l(3)*s1(i).*c23(i);
           0                                -(l(2)*s2(i)+l(3)*s23(i))               -l(3)*s23(i)];
       
    % computation of the inverse Jacobian at every timestep
    Jacinv=inv(Jac);
    
    % computation of the desired angular velocities at every timestep
    qd_1(i) = Jacinv(1,:)*[vdx(i) vdy(i) vdz(i)]';
    qd_2(i) = Jacinv(2,:)*[vdx(i) vdy(i) vdz(i)]';
    qd_3(i) = Jacinv(3,:)*[vdx(i) vdy(i) vdz(i)]';
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%**************************************************************************



%% ******************* KINEMATIC SIMULATION ****************************** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %% 


%% ***********************************************************************

%%%%%%%%%% DIAGRAMS OF MOTION - SIMULATION VISUALIZATION %%%%%%%%%%%%%%%%%
scrsz = get(0,'ScreenSize');
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
hold on 
grid on
xlim([-100 1200]);
ylim([-600 1000]);
zlim([0 1000]);
axis square
view(60,15);
box on
xlabel('x (mm)'); 
ylabel('y (mm)'); 
zlabel('z (mm)'); 
title('\fontsize{18} {\color{blue}3D Kinematic Simulation}');
plot3(0,0,0,'rs','LineWidth',3); 

% desired approach line
plot3(pdx1,pdy1,pdz1,'r');

% desired straight line while welding
plot3([px1,px2],[py1,py2],[pz1,pz2],'k','LineWidth',6);

% desired exiting line
plot3(pdx3,pdy3,pdz3,'r');

% for visualizing the base of the robot
pbx1=-50*c1;   pby1=-50*s1;    pbz1=0*c1;
pbx2=0*c1;     pby2=0*s1;      pbz2=pbz1+100;




for tk=1:20:kmax
    
       cla;
       
       view(60-(tk/400),15+(tk/2000));
       
       plot3([px1,px2],[py1,py2],[pz1,pz2],'k','LineWidth',3);
       
       plot3([0,px1d(tk)],[0,py1d(tk)],[0,pz1d(tk)],'LineWidth',3);					
       plot3(px1d(tk),py1d(tk),pz1d(tk),'ro','LineWidth',6);    
   
       plot3([px1d(tk),px2d(tk)],[py1d(tk),py2d(tk)],[pz1d(tk),pz2d(tk)],'LineWidth',3);	
       plot3(px2d(tk),py2d(tk),pz2d(tk),'ro','LineWidth',6);  
   
       plot3([px2d(tk),px3d(tk)],[py2d(tk),py3d(tk)],[pz2d(tk),pz3d(tk)],'LineWidth',3);	
       plot3(px3d(tk),py3d(tk),pz3d(tk),'y*','LineWidth',6); 
   
       plot3([pbx1(tk),pbx2(tk)],[pby1(tk),pby2(tk)],[pbz1(tk),pbz2(tk)],'LineWidth',3);
       plot3([pbx1(tk),pbx2(tk)],[pby1(tk),pby2(tk)],[0,0],'LineWidth',3);
       
       pause(0.01);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pause(1.0);







%%%%%%%%%%%%%%%%%%%%%%%% STICK DIAGRAM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%  DIAGRAMS IN 3 DIMENSIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scrsz = get(0,'ScreenSize');
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
hold on 
grid on
axis square
view(60,15);
box on
xlabel('x (mm)'); 
ylabel('y (mm)'); 
zlabel('z (mm)'); 
title('\fontsize{18} {\color{blue}3D Kinematic Simulation}');
plot3(0,0,0,'rs','LineWidth',3); 

% plot of desired approach line
plot3(pdx1,pdy1,pdz1,'r');

% plot of desired straight line of welding with strong black color
plot3([px1,px2],[py1,py2],[pz1,pz2],'k','LineWidth',6);

% plot of exiting line
plot3(pdx3,pdy3,pdz3,'r');

% plot position every dtk samples
dtk=100; 

for tk=1:dtk:kmax,   	
    
   pause(0.05);	% drawnow can also be used;
   
   plot3([0,px1d(tk)],[0,py1d(tk)],[0,pz1d(tk)],'LineWidth',1);					
   plot3(px1d(tk),py1d(tk),pz1d(tk),'ro','LineWidth',3);    
   
   plot3([px1d(tk),px2d(tk)],[py1d(tk),py2d(tk)],[pz1d(tk),pz2d(tk)],'LineWidth',1);	
   plot3(px2d(tk),py2d(tk),pz2d(tk),'ro','LineWidth',3);  
   
   plot3([px2d(tk),px3d(tk)],[py2d(tk),py3d(tk)],[pz2d(tk),pz3d(tk)],'LineWidth',1);	
   plot3(px3d(tk),py3d(tk),pz3d(tk),'y*','LineWidth',3); 
   
end     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%%%%%  DIAGRAMS IN X-Y PLANE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
hold on 
grid on
axis square
xlabel('x (mm)'); 
ylabel('y (mm)'); 
zlabel('z (mm)'); 
title('\fontsize{18} {\color{blue}X-Y Kinematic Simulation}');
plot(0,0,'rs','LineWidth',3); 

plot(pdx1,pdy1,'b');
plot([px1,px2],[py1,py2],'k','LineWidth',6);
plot(pdx3,pdy3,'r');

for tk=1:dtk:kmax,     	
    
   pause(0.05);	
   
   plot([0,px1d(tk)],[0,py1d(tk)],'LineWidth',1);					
   plot(px1d(tk),py1d(tk),'ro','LineWidth',3);    
   
   plot([px1d(tk),px2d(tk)],[py1d(tk),py2d(tk)],'LineWidth',1);	
   plot(px2d(tk),py2d(tk),'ro','LineWidth',3);  
   
   plot([px2d(tk),px3d(tk)],[py2d(tk),py3d(tk)],'LineWidth',1);	
   plot(px3d(tk),py3d(tk),'y*','LineWidth',3);
   
end     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%%%%%  DIAGRAMS IN Y-Z PLANE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
hold on 
grid on
axis square
xlabel('y (mm)'); 
ylabel('z (mm)');  
title('\fontsize{18} {\color{blue}Y-Z Kinematic Simulation}');
plot(0,0,'rs','LineWidth',3);

plot(pdy1,pdz1,'b');
plot([py1,py2],[pz1,pz2],'k','LineWidth',6);
plot(pdy3,pdz3,'r');

for tk=1:dtk:kmax,   	
    
   pause(0.05);    
   
   plot([0,py1d(tk)],[0,pz1d(tk)],'LineWidth',1);					
   plot(py1d(tk),pz1d(tk),'ro','LineWidth',3);    
   
   plot([py1d(tk),py2d(tk)],[pz1d(tk),pz2d(tk)],'LineWidth',1);	
   plot(py2d(tk),pz2d(tk),'ro','LineWidth',3);  
   
   plot([py2d(tk),py3d(tk)],[pz2d(tk),pz3d(tk)],'LineWidth',1);	
   plot(py3d(tk),pz3d(tk),'y+','LineWidth',3);  
   
end     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






%%%%%%%%%%%%%%%%%%%  DIAGRAMS IN X-Z PLANE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
hold on 
grid on
axis square
xlabel('x (mm)'); 
ylabel('z (mm)');  
title('\fontsize{18} {\color{blue}X-Z Kinematic Simulation}');
plot(0,0,'rs','LineWidth',3); 

plot(pdx1,pdz1,'b');
plot([px1,px2],[pz1,pz2],'k','LineWidth',6);
plot(pdx3,pdz3,'r');

for tk=1:dtk:kmax,   	
    
   pause(0.05);   
   
   plot([0,px1d(tk)],[0,pz1d(tk)],'LineWidth',1);					
   plot(px1d(tk),pz1d(tk),'ro','LineWidth',3);    
   
   plot([px1d(tk),px2d(tk)],[pz1d(tk),pz2d(tk)],'LineWidth',1);	
   plot(px2d(tk),pz2d(tk),'ro','LineWidth',3);  
   
   plot([px2d(tk),px3d(tk)],[pz2d(tk),pz3d(tk)],'LineWidth',1);	
   plot(px3d(tk),pz3d(tk),'y*','LineWidth',3);  
   
end     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%% DIAGRAMS OF ANGULAR POSITIONS AND VELOCITIES %%%%%%%%%%%%%%%%
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
t=linspace(0,T1+T2+T3,kmax);

subplot(3,2,1);
plot(t,qd1);
title('angle of 1st joint (rad)');
xlabel('time (sec)');

subplot(3,2,2);
plot(t,qd_1);
title('angular velocity of 1st joint (rad/sec)');
xlabel('time (sec)');

subplot(3,2,3);
plot(t,qd2);
title('angle of 2nd joint (rad)');
xlabel('time (sec)');

subplot(3,2,4);
plot(t,qd_2);
title('angular velocity of 2nd joint (rad/sec)');
xlabel('time (sec)');

subplot(3,2,5);
plot(t,qd3);
title('angle of 3rd joint (rad)');
xlabel('time (sec)');

subplot(3,2,6);
plot(t,qd_3);
title('angular velocity of 3rd joint (rad/sec)');
xlabel('time (sec)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


pause(1.0);


%%%%%%%%%%%% DIAGRAMS OF CARTESIAN POSITIONS AND VELOCITIES %%%%%%%%%%%%%%%
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 

subplot(3,2,1);
plot(t,pdx);
title('desired x position (mm)');
xlabel('time (sec)');

subplot(3,2,2);
plot(t,vdx);
title('desired x velocity (mm/sec)');
xlabel('time (sec)');

subplot(3,2,3);
plot(t,pdy);
title('desired y position (mm)');
xlabel('time (sec)');

subplot(3,2,4);
plot(t,vdy);
title('desired y velocity (mm/sec)');
xlabel('time (sec)');

subplot(3,2,5);
plot(t,pdz);
title('desired z position (mm)');
xlabel('time (sec)');

subplot(3,2,6);
plot(t,vdz);
title('desired z velocity (mm/sec)');
xlabel('time (sec)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


















