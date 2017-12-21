%% ********** ������������ ���������� ��������� **********************%%%
clear all; 
close all; 

%%%%%%%%%%%%%%%%%%%%%%%% ���� ��� ��������� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
l(1) = 420;         % �� mm 
l(2) = 450;         % �� mm
l(3) = 650+85+100;  % l3+l4+le �� mm

%%%%%%%%%%%%%%%%%%%% �������� �������������� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.001;         %������ 1 msec 



%% ************ ���������� ���������� ������� ****************************


%%%%%%%%%%%%%%%%%%%% ������� ���������� ������ %%%%%%%%%%%%%%%%%%%%%%%%%%%
%������ ����������� ��� ������� �� sec
T1 = 2; 

%������ ��������� ��� ������� �� sec
T2 = 5; 

%������ ������������ �� sec
T3 = 2;      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%% ������� ������ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������ ���� ��� ������� ��������� ������
px0 = 600;  py0 = 750;    pz0 = 1000;

%������ ������ ��� ���������� �������
px1 = 725;  py1 = 825;  pz1 = 900;

%������ ������ ��� ���������� �������
px2 = 1000; py2 =-450;  pz2 = 150;

%������ ������ ��� ������� ��������� ������
px3 = 950;  py3 = -560;    pz3 = 100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%% ������� ��������� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������ �������� ��� ������� ��������� ������
vx0 = 0;    vy0 = 0;    vz0 = 0;

%�������� �� ��� ����� ������ �� ����������� ��� ������
vx1 = (px2-px1)/T2;  vy1 = (py2-py1)/T2;  vz1 = (pz2-pz1)/T2;

%�������� �� ��� ����� ������� ��� ��� ������
vx2 = vx1;  vy2 = vy1;  vz2 = vz1;

%�������� �� ��� ����� ��������� � �������
vx3 = 0;    vy3 = 0;    vz3 = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%% ������� ������������ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������ ���������� ��� ������� ��������� ������
gx0 = 0;    gy0 = 0;    gz0 = 0;

%�������� �� ��� ����� ������ �� ����������� ��� ������
gx1 = 0;    gy1 = 0;    gz1 = 0;

%�������� �� ��� ����� ������� ��� ��� ������
gx2 = 0;    gy2 = 0;    gz2 = 0;

%�������� �� ��� ����� ��������� � �������
gx3 = 0;    gy3 = 0;    gz3 = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%% ����������� ��������� ���� ��� ���������� %%%%%%%%%%%%%%%%%%%%
%�������� �������� ������� ���� ��� ����������
t1 = 0:dt:T1; 

%�������� �� ������� A*S=B ��� ���� �������� �� �����o ����� ���� ���� S 
%�� ������ ���� ����������� ��� ���������� 5�� ������ ��� ���������� �� 
%������ ����������. �� ���������� �� ���������������� ��'������� ����
%������ ���� �������� �� matlab �� ����� �� �������.
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

%���� ����� S1 ���������� �� ����������� ��� ���������� 5�� ������
a0x=S1x(1); a1x=S1x(2); a2x=S1x(3); a3x=S1x(4); a4x=S1x(5); a5x=S1x(6);
a0y=S1y(1); a1y=S1y(2); a2y=S1y(3); a3y=S1y(4); a4y=S1y(5); a5y=S1y(6);
a0z=S1z(1); a1z=S1z(2); a2z=S1z(3); a3z=S1z(4); a4z=S1z(5); a5z=S1z(6);

% ����������� ��� ����������� ��� ���������� ������ ���� ��� ����������
pdx1 = a0x+a1x*t1+a2x*t1.^2+a3x*t1.^3+a4x*t1.^4+a5x*t1.^5;
pdy1 = a0y+a1y*t1+a2y*t1.^2+a3y*t1.^3+a4y*t1.^4+a5y*t1.^5;
pdz1 = a0z+a1z*t1+a2z*t1.^2+a3z*t1.^3+a4z*t1.^4+a5z*t1.^5;

%����������� ��� ����������� ��� ���������� ��������� ���� ��� ����������
vdx1 = a1x+2*a2x*t1+3*a3x*t1.^2+4*a4x*t1.^3+5*a5x*t1.^4;
vdy1 = a1y+2*a2y*t1+3*a3y*t1.^2+4*a4y*t1.^3+5*a5y*t1.^4;
vdz1 = a1z+2*a2z*t1+3*a3z*t1.^2+4*a4z*t1.^3+5*a5z*t1.^4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%% ���������� ������� ���� �� ��������� %%%%%%%%%%%%%%%%%%%%%%%
%�������� �������� ������� ���� ��� ���������
t2=dt:dt:T2;  % *������ ��� dt ��� �� ��� ��� ��������� ��������

% ����������� ��� ����������� ��� ���������� ������ ���� �� ���������
pdx2 = px1+vx1*t2;
pdy2 = py1+vy1*t2;
pdz2 = pz1+vz1*t2;

%����������� ��� ����������� ��� ���������� ��������� ���� �� ���������
vdx2(1:length(t2)) = vx1;
vdy2(1:length(t2)) = vy1;
vdz2(1:length(t2)) = vz1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%% ����������� ��������� ���� ��� ����������� %%%%%%%%%%%%%%%%%
%�������� �������� ������� ���� ��� �����������
t3=dt:dt:T3;

%��� ���� �������� �� ������� A*S=B ��� ���� ��������
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

%���� ����� S2 ���������� �� ����������� ��� ���������� 5�� ������
b0x=S2x(1); b1x=S2x(2); b2x=S2x(3); b3x=S2x(4); b4x=S2x(5); b5x=S2x(6);
b0y=S2y(1); b1y=S2y(2); b2y=S2y(3); b3y=S2y(4); b4y=S2y(5); b5y=S2y(6);
b0z=S2z(1); b1z=S2z(2); b2z=S2z(3); b3z=S2z(4); b4z=S2z(5); b5z=S2z(6);

% ����������� ��� ����������� ��� ���������� ������ ���� ��� �����������
pdx3 = b0x+b1x*t3+b2x*t3.^2+b3x*t3.^3+b4x*t3.^4+b5x*t3.^5;
pdy3 = b0y+b1y*t3+b2y*t3.^2+b3y*t3.^3+b4y*t3.^4+b5y*t3.^5;
pdz3 = b0z+b1z*t3+b2z*t3.^2+b3z*t3.^3+b4z*t3.^4+b5z*t3.^5;

%����������� ��� ����������� ��� ���������� ��������� ���� ��� �����������
vdx3 = b1x+2*b2x*t3+3*b3x*t3.^2+4*b4x*t3.^3+5*b5x*t3.^4;
vdy3 = b1y+2*b2y*t3+3*b3y*t3.^2+4*b4y*t3.^3+5*b5y*t3.^4;
vdz3 = b1z+2*b2z*t3+3*b3z*t3.^2+4*b4z*t3.^3+5*b5z*t3.^4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%% CONCATENATION ��� ���������� ������ ��� ��������� %%%%%%%%%%%%%

% �������� ���������� ������ �� ���� ������� ������
pdx = [pdx1 pdx2 pdx3];
pdy = [pdy1 pdy2 pdy3];
pdz = [pdz1 pdz2 pdz3];

%�������� ���������� ��������� �� ���� ������� ������
vdx = [vdx1 vdx2 vdx3];
vdy = [vdy1 vdy2 vdy3];
vdz = [vdz1 vdz2 vdz3];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%**************************************************************************






%% ***** ���������� USER INTERFACE ��� ������� ���������� ����� ***********
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

first=0;  second=0;  third=0;  fourth=0;

scrsz = get(0,'ScreenSize');
figure('Position',[(scrsz(3)/2)-150 (scrsz(4)/2)-400 300 800]); 

h1 = uicontrol('Style', 'pushbutton', 'String', 'front-elbow up',...
   'Position', [10 650 100 100],...
   'Callback', 'first=1;');
h2 = uicontrol('Style', 'pushbutton', 'String', 'front-elbow down',...
   'Position', [10 450 100 100],...
   'Callback', 'second=1;');
h3 = uicontrol('Style', 'pushbutton', 'String', 'back-elbow down',...
   'Position', [10 250 100 100],...
   'Callback', 'third=1;');
h4 = uicontrol('Style', 'pushbutton', 'String', 'back-elbow up',...
   'Position', [10 50 100 100],...
   'Callback', 'fourth=1;');


p1=subplot('position',[0.45 0.80 0.5 0.15]);
hold on
title('\fontsize{18} {\color{blue}Choose Solution                   }');
plot([10 10],[10 50],'linewidth',2);  plot(10,50,'o','linewidth',2);
plot([10 50],[50 80],'linewidth',2);  plot(50,80,'o','linewidth',2);
plot([50 90],[80 50],'linewidth',2);  plot(90,50,'r*','linewidth',2);
plot([0 10],[10 20],'linewidth',2);   plot([0 10],[10 10],'linewidth',2);
axis([-15 100 0 100]);

p2=subplot('position',[0.45 0.55 0.5 0.15]);
hold on
plot([10 10],[10 50],'linewidth',2);  plot(10,50,'o','linewidth',2);
plot([10 50],[50 20],'linewidth',2);  plot(50,20,'o','linewidth',2);
plot([50 90],[20 50],'linewidth',2);  plot(90,50,'r*','linewidth',2);
plot([0 10],[10 20],'linewidth',2);   plot([0 10],[10 10],'linewidth',2);
axis([-15 100 0 100]);

p3=subplot('position',[0.45 0.3 0.5 0.15]);
hold on
plot([10 10],[10 50],'linewidth',2);  plot(10,50,'o','linewidth',2);
plot([10 50],[50 20],'linewidth',2);  plot(50,20,'o','linewidth',2);
plot([50 90],[20 50],'linewidth',2);  plot(90,50,'r*','linewidth',2);
plot([20 10],[10 20],'linewidth',2);   plot([10 20],[10 10],'linewidth',2);
axis([-15 100 0 100]);

p4=subplot('position',[0.45 0.05 0.5 0.15]);
hold on
plot([10 10],[10 50],'linewidth',2);  plot(10,50,'o','linewidth',2);
plot([10 50],[50 80],'linewidth',2);  plot(50,80,'o','linewidth',2);
plot([50 90],[80 50],'linewidth',2);  plot(90,50,'r*','linewidth',2);
plot([20 10],[10 20],'linewidth',2);  plot([10 20],[10 10],'linewidth',2);
axis([-15 100 0 100]);


 while (first==0)&&(second==0)&&(third==0)&&(fourth==0)
 pause(0.05);
 end
close;




%% ***************  ���������� ���������� ������� *************************
%������������ ��� ������������ ��� ���������� �������� ������ ��� ���������
%��� arrays qd1 , qd2 , qd3. �������� �� comment ��� uncomment �� 
%������������� ��� ��������� ���� ��� ��� �������� ��� ������������ ���
%��������� �����.


%������� ��� �� ��� ����������
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




   
%% ****************** ���� ���������� ������� ***************************** 

%���� ��� ����� ��� 1�� ���������
px1d(1:length(c1)) = 0;
py1d(1:length(c1)) = 0;
pz1d(1:length(c1)) = l(1);

%���� ��� ����� ��� 2�� ���������
px2d = c1.*(l(2)*s2);
py2d = s1.*(l(2)*s2);
pz2d = l(1) + l(2)*c2;

%���� ��� ������� ��������� ������ - ���������
px3d = c1.*(l(2)*s2+l(3)*s23); 
py3d = s1.*(l(2)*s2+l(3)*s23);
pz3d = l(1) + l(2)*c2 + l(3)*c23;
%**************************************************************************



%% **************** ���������� ��������� ������� **************************
%������������ ��� ������������ �� ������� �������� ��� ���������� ��������
%��������� ������� ������� ������ ��� arrays qd_1 , qd_2, qd_3


%%%%%%%%%%%%%%%% JACOBIAN AND ANGULAR VELOCITIES %%%%%%%%%%%%%%%%%%%%%%%%%%

%����� ��� ��������� �������� �����������
kmax = length(t1)+length(t2)+length(t3);

%�������������� ��� �������� �������
qd_1 = zeros([1 kmax]);
qd_2 = zeros([1 kmax]);
qd_3 = zeros([1 kmax]);

for i=1:1:kmax 
    
    %����������� ���������� ���� ������� ������
    Jac=[-s1(i).*(l(2)*s2(i)+l(3)*s23(i))   c1(i).*(l(2)*c2(i)+l(3)*c23(i))    l(3)*c1(i).*c23(i);
          c1(i).*(l(2)*s2(i)+l(3)*s23(i))   s1(i).*(l(2)*c2(i)+l(3)*c23(i))    l(3)*s1(i).*c23(i);
           0                                -(l(2)*s2(i)+l(3)*s23(i))               -l(3)*s23(i)];
       
    %����������� ����������� ���������� ���� ������� ������ 
    Jacinv=inv(Jac);
    
    %����������� �������� ��������� ���� ������� ������
    qd_1(i) = Jacinv(1,:)*[vdx(i) vdy(i) vdz(i)]';
    qd_2(i) = Jacinv(2,:)*[vdx(i) vdy(i) vdz(i)]';
    qd_3(i) = Jacinv(3,:)*[vdx(i) vdy(i) vdz(i)]';
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%**************************************************************************



%% ****************** ���������� ����������� ****************************** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %% 


%% ***********************************************************************

%%%%%%%%%% ����������� ��� ������� - ���������� ��� ������������ %%%%%%%%%%
%������� ��� ����� ��������� ��� ��� ��������������� ��� ����������
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

%�������� ��� �������� �����������
plot3(pdx1,pdy1,pdz1,'r');

%�������� ��� ������� �� ������ ����� �����
plot3([px1,px2],[py1,py2],[pz1,pz2],'k','LineWidth',6);

%�������� ��� �������� ������������
plot3(pdx3,pdy3,pdz3,'r');

%�� ������ ��� ���� ��� �� �������� � ���������� ��� 1�� ��������
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

%%%%%%%%%%%%%%%%%  ����������� ���� ����� ���������� %%%%%%%%%%%%%%%%%%%%%%
%������� ��� ����� ��������� ��� ��� ��������������� ��� ����������
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

%�������� ��� �������� �����������
plot3(pdx1,pdy1,pdz1,'r');

%�������� ��� ������� �� ������ ����� �����
plot3([px1,px2],[py1,py2],[pz1,pz2],'k','LineWidth',6);

%�������� ��� �������� ������������
plot3(pdx3,pdy3,pdz3,'r');

%������� ��� ����� ���� dtk ��������
dtk=100; 

for tk=1:dtk:kmax,   	
    
   pause(0.05);	%����� ��� �������� ���������� ��� ������������    
   
   plot3([0,px1d(tk)],[0,py1d(tk)],[0,pz1d(tk)],'LineWidth',1);					
   plot3(px1d(tk),py1d(tk),pz1d(tk),'ro','LineWidth',3);    
   
   plot3([px1d(tk),px2d(tk)],[py1d(tk),py2d(tk)],[pz1d(tk),pz2d(tk)],'LineWidth',1);	
   plot3(px2d(tk),py2d(tk),pz2d(tk),'ro','LineWidth',3);  
   
   plot3([px2d(tk),px3d(tk)],[py2d(tk),py3d(tk)],[pz2d(tk),pz3d(tk)],'LineWidth',1);	
   plot3(px3d(tk),py3d(tk),pz3d(tk),'y*','LineWidth',3); 
   
end     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%%%%%  ����������� ��� �-� ������� %%%%%%%%%%%%%%%%%%%%%%%%

%������� ��� ����� ��������� ��� ��� ��������������� ��� ����������
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





%%%%%%%%%%%%%%%%%%%%%  ����������� ��� Y-Z ������� %%%%%%%%%%%%%%%%%%%%%%%%

%������� ��� ����� ��������� ��� ��� ��������������� ��� ����������
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






%%%%%%%%%%%%%%%%%%%  ����������� ��� �-� ������� %%%%%%%%%%%%%%%%%%%%%%%%%%
%������� ��� ����� ��������� ��� ��� ��������������� ��� ����������
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
hold on 
grid on
axis square
xlabel('x (mm)'); 
ylabel('� (mm)');  
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




%%%%%%%%%%%%% ����������� �������� ������ ��� ��������� %%%%%%%%%%%%%%%%%%%
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
t=linspace(0,T1+T2+T3,kmax);

subplot(3,2,1);
plot(t,qd1);
title('\fontsize{14}����� ������� ��� 1�� �������� (rad)');
xlabel('time (sec)');

subplot(3,2,2);
plot(t,qd_1);
title('\fontsize{14}������� �������� ��� 1�� ��������(rad/sec)');
xlabel('time (sec)');

subplot(3,2,3);
plot(t,qd2);
title('\fontsize{14}����� ������� ��� 2�� ��������(rad)');
xlabel('time (sec)');

subplot(3,2,4);
plot(t,qd_2);
title('\fontsize{14}������� �������� ��� 2�� ��������(rad/sec)');
xlabel('time (sec)');

subplot(3,2,5);
plot(t,qd3);
title('\fontsize{14}����� ������� ��� 3�� ��������(rad)');
xlabel('time (sec)');

subplot(3,2,6);
plot(t,qd_3);
title('\fontsize{14}������� �������� ��� 3�� ��������(rad/sec)');
xlabel('time (sec)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


pause(1.0);


%%%%%%%%%%%% ����������� ����������� ������ ��� ��������� %%%%%%%%%%%%%%%%%
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 

subplot(3,2,1);
plot(t,pdx);
title('\fontsize{14}��������� ���� ���� x - ����� (mm)');
xlabel('time (sec)');

subplot(3,2,2);
plot(t,vdx);
title('\fontsize{14}��������� �������� ���� ��� x - ����� (mm/sec)');
xlabel('time (sec)');

subplot(3,2,3);
plot(t,pdy);
title('\fontsize{14}��������� ���� ���� y - ����� (mm)');
xlabel('time (sec)');

subplot(3,2,4);
plot(t,vdy);
title('\fontsize{14}��������� �������� ���� ��� y - ����� (mm/sec)');
xlabel('time (sec)');

subplot(3,2,5);
plot(t,pdz);
title('\fontsize{14}��������� ���� ���� z - ����� (mm)');
xlabel('time (sec)');

subplot(3,2,6);
plot(t,vdz);
title('\fontsize{14}��������� �������� ���� ��� z - ����� (mm/sec)');
xlabel('time (sec)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



save;

















