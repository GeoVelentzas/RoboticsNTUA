%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% ΚΙΝΗΜΑΤΙΚΟΣ ΕΛΕΓΧΟΣ ΡΟΜΠΟΤ ΜΕ ΠΛΕΟΝΑΖΟΝΤΕΣ ΒΑΘΜΟΥΣ %%%%%%%%%%%%
close all;
clear all;
clc;
scrsz = get(0,'ScreenSize');

%%%%%%%%%%ΚΕΡΔΗ ΓΙΑ ΤΟΝ ΕΛΕΓΧΟ ΠΡΩΤΗΣ ΚΑΙ ΔΕΥΤΕΡΗΣ ΥΠΟΕΡΓΑΣΙΑΣ%%%%%%%%%%%%%
kp1 = 100; %1η
kc  = 50;  %μη κανονικοποιημένο κέρδος 2ης υποεργασιας



%%%%%%%%%%%%%%%%%%%%%%% ΜΗΚΗ ΤΩΝ ΣΥΝΔΕΣΜΩΝ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% ΜΠΟΡΕΙ ΝΑ ΓΙΝΕΙ ΑΛΛΑΓΗ ΑΠΟ ΤΟΝ ΧΡΗΣΤΗ %%%%%%%%%%%%%%%%%%%%%%%
h    = 0.6;
l(1) = 1.0;
l(2) = 0.5;
l(3) = 1.0;
l(4) = 0.7;
l(5) = 0.8;
l(6) = 0.4;


%%%%%%%%%%%%%%%%%%%%%%% ΠΕΡΙΟΔΟΣ ΔΕΙΓΜΑΤΟΛΕΙΨΙΑΣ %%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.001;    %τυπικα 1msec



%%%%%%%%%%%%%%%%%%% ΣΥΝΟΛΙΚΗ ΔΙΑΡΚΕΙΑ ΤΗΣ ΠΡΟΣΟΜΟΙΩΣΗΣ %%%%%%%%%%%%%%%%%%%%
Tf = 10.0; 	    %συνολικός χρόνος
t=0:dt:Tf;      %διανυσμα χρονου
kmax=Tf/dt + 1; %συνολικα βηματα



%%%%%%%%%%%%%%%%%%%%%%% ΑΡΧΙΚΗ ΔΙΑΤΑΞΗ ΤΟΥ ΡΟΜΠΟΤ %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% ΜΠΟΡΕΙ ΝΑ ΓΙΝΕΙ ΑΛΛΑΓΗ ΑΠΟ ΤΟ ΧΡΗΣΤΗ %%%%%%%%%%%%%%%%%%
 d = 0.2; 
 qd1_0 = -30*pi/180; qd2_0 = 0; qd3_0 = 50*pi/180 ; 
 qd4_0 = -20*pi/180;  qd5_0 = 10*pi/180; qd6_0 = -10*pi/180;


 
%%%%%%%%%%%%%%%%%%%%%%%%%%% ΑΡΧΙΚΟΠΟΙΗΣΕΙΣ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%αρχικες γωνιες
s1 = sin(qd1_0); 
s12 = sin(qd1_0+qd2_0); 
s123 = sin(qd1_0+qd2_0+qd3_0);
s1234 = sin(qd1_0+qd2_0+qd3_0+qd4_0);
s12345 = sin(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0);
s123456 = sin(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0);
c1 = cos(qd1_0); 
c12 = cos(qd1_0+qd2_0); 
c123 = cos(qd1_0+qd2_0+qd3_0);
c1234 = cos(qd1_0+qd2_0+qd3_0+qd4_0);
c12345 = cos(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0);
c123456 = cos(qd1_0+qd2_0+qd3_0+qd4_0+qd5_0+qd6_0);
%αρχικες θεσεις
xd1_0 = d;
yd1_0 = 0;
xd2_0 = d;
yd2_0 = h;
xd3_0 = xd2_0-l(1)*s1;
yd3_0 = yd2_0+l(1)*c1;
xd4_0 = xd3_0-l(2)*s12;
yd4_0 = yd3_0+l(2)*c12;
xd5_0 = xd4_0-l(3)*s123;
yd5_0 = yd4_0+l(3)*c123;
xd6_0 = xd5_0-l(4)*s1234;
yd6_0 = yd5_0+l(4)*c1234;
xd7_0 = xd6_0-l(5)*s12345;
yd7_0 = yd6_0+l(5)*c12345;
xde_0 = xd7_0-l(6)*s123456;
yde_0 = yd7_0+l(6)*c123456;


%%%%%%%%%%%%%%%%%%%%%  1η ΥΠΟΕΡΓΑΣΙΑ ΑΡΧΙΚΗ ΘΕΣΗ %%%%%%%%%%%%%%%%%%%%%%%%%%
xde = xde_0;   %αρχική θέση τελικού στοιχείου να συμπίπτει με τον στοχο
yde = yde_0;   %για αποφυγή λανθασμένης διάταξης
               %μπορείτε να το αλλάξετε


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EΜΠΟΔΙΑ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig0=figure;
figure(fig0);
obdist = 2;                 %απόσταση των εμποδίων μεταξύ τους 
obr = 0.4;                  %ακτίνα εμποδίων
if obdist<3*obr; obdist = 3*obr; end
yob1 = yd3_0;               %αρχικη τεταγμένη του αριστερου εμποδιου
xob1 = xd3_0 - (obdist/2);  %αρχική τετμημένη του αριστερού εμποδίου
xob2 = xob1+obdist;         %αρχική τετμημένη δεξιού εμποδίου
yob2 = yob1;



%%%%%%%%%%%%% GUI ΙNTERFACE: ΒUTTONS ΕΛΕΓΧΟΥ ΤΩΝ ΕΜΠΟΔΙΩΝ %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% ΚΑΙ ΤΟΥ ΣΤΟΧΟΥ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stop = 0;          %μεταβλητη για  δυνατότητα τερματισμου της προσομοιωσης
step = 0.05;     %βημα μετακινησης σε καθε πατημα του μπουτον
h1 = uicontrol('Style', 'pushbutton', 'String', 'U',...
   'Position', [35 140 35 30],...
   'Callback', 'yob1=yob1+step;');
h2 = uicontrol('Style', 'pushbutton', 'String', 'D',...
   'Position', [35 110 35 30],...
   'Callback', 'yob1=yob1-step;');
h3 = uicontrol('Style', 'pushbutton', 'String', 'L',...
   'Position', [0 140 35 30],...
   'Callback', 'xob1=xob1-step;');
h4 = uicontrol('Style', 'pushbutton', 'String', 'R',...
   'Position', [0 110 35 30],...
   'Callback', 'xob1=xob1+step;');

h5 = uicontrol('Style', 'pushbutton', 'String', 'U',...
   'Position', [35 230 35 30],...
   'Callback', 'yde=yde+step;');
h6 = uicontrol('Style', 'pushbutton', 'String', 'D',...
   'Position', [35 200 35 30],...
   'Callback', 'yde=yde-step;');
h7 = uicontrol('Style', 'pushbutton', 'String', 'L',...
   'Position', [0 230 35 30],...
   'Callback', 'xde=xde-step;');
h8 = uicontrol('Style', 'pushbutton', 'String', 'R',...
   'Position', [0 200 35 30],...
   'Callback', 'xde=xde+step;');


h9 = uicontrol('Style', 'pushbutton', 'String', 'STOP',...
   'Position', [10 30 45 40],...
   'Callback', 'stop=1;');







%%%%%%%%%%%%%%%%%%%%%%%%% START PLOTING FIGURE %%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(fig0);
axis([-1/2*(sum(l)+h)-0.2 1/2*(sum(l)+h)+0.2 -0.2 sum(l)+h+0.2]);
axis on
hold on
box on
%grid on
xlabel('x (m)');
ylabel('y (m)');
dtk = 1;%διαστημα βηματων μεταξυ διαδοχικων plots (αν κολλαει!)
plot(xd1_0, yd1_0, 's');
x1 = linspace(0,2*pi,50); y1 = x1;%δημιουργία εμποδίων με σωστές διαστάσεις
x1 = xob1 + obr*sin(x1);  x2 = x1 + obdist;
y1 = yob1 + obr*cos(y1);  y2 = y1;
fill(x1,y1,'r');fill(x2,y2,'r');





%%%%%%%%%%%%%%%%%%%%%%%%%% ΔΙΑΓΡΑΜΜΑ ΕΚΚΙΝΗΣΗΣ %%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot(xde, yde, 'o','MarkerSize',10,'MarkerFaceColor','g');

plot([xd1_0, xd2_0], [yd1_0, yd2_0],'k');
plot([xd2_0, xd3_0], [yd2_0, yd3_0],'k');
plot([xd3_0, xd4_0], [yd3_0, yd4_0],'k');
plot([xd4_0, xd5_0], [yd4_0, yd5_0],'k');
plot([xd5_0, xd6_0], [yd5_0, yd6_0],'k');
plot([xd6_0, xd7_0], [yd6_0, yd7_0],'k');
plot([xd7_0, xde_0], [yd7_0, yde_0],'k');

plot(xd1_0, yd1_0, 'ks','LineWidth',2,'MarkerSize',15,'MarkerFaceColor','w');
plot(xd2_0, yd2_0, 'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
plot(xd3_0, yd3_0, 'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
plot(xd4_0, yd4_0, 'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
plot(xd5_0, yd5_0, 'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
plot(xd6_0, yd6_0, 'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
plot(xd7_0, yd7_0, 'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');

plot(xd2_0, yd2_0, 'ko','MarkerSize',2,'MarkerFaceColor','k');
plot(xd3_0, yd3_0, 'ko','MarkerSize',2,'MarkerFaceColor','k');
plot(xd4_0, yd4_0, 'ko','MarkerSize',2,'MarkerFaceColor','k');
plot(xd5_0, yd5_0, 'ko','MarkerSize',2,'MarkerFaceColor','k');
plot(xd6_0, yd6_0, 'ko','MarkerSize',2,'MarkerFaceColor','k');
plot(xd7_0, yd7_0, 'ko','MarkerSize',2,'MarkerFaceColor','k');
plot(xde_0, yde_0, 'kx','LineWidth',2,'MarkerSize',10);


plot(-1/2*(sum(l)+h),0,'m^','MarkerSize',10,'MarkerFaceColor','m');          %αρχικη θεση χρονου
plot(-1/2*(sum(l)+h),(sum(l)+h),'mv','MarkerSize',10,'MarkerFaceColor','m'); %τελικη θεση χρονου
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% ΚΑΠΟΙΕΣ ΑΡΧΙΚΟΠΟΙΗΣΕΙΣ ΠΑΡΑΜΕΤΡΩΝ ΓΙΑ PREALLOCATION ΠΡΙΝ ΤΟΝ ΒΡΟΧΟ %%%%
tt=0; tk=1;
dd(tk)= d;
qd(tk,1)=qd1_0; qd(tk,2)=qd2_0; qd(tk,3)=qd3_0; qd(tk,4)=qd4_0;
qd(tk,5)=qd5_0; qd(tk,6)=qd6_0; 
xdes = zeros(1,kmax);
ydes = zeros(1,kmax);
vdesx = zeros(1,kmax);
vdesy = zeros(1,kmax);




%%%%%%%%%%%%%%%%%%%%%%%  ΚΡΙΣΙΜΑ ΣΗΜΕΙΑ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H  = linspace(0,h,2*h/obr);        if 2*obr>h;     H  = [0 h];    end       
L1 = linspace(0,l(1),2*l(1)/obr);  if 2*obr>l(1);  L1 = [0 l(1)]; end    
L2 = linspace(0,l(2),2*l(2)/obr);  if 2*obr>l(2);  L2 = [0 l(2)]; end  
L3 = linspace(0,l(3),2*l(3)/obr);  if 2*obr>l(3);  L3 = [0 l(3)]; end   
L4 = linspace(0,l(4),2*l(4)/obr);  if 2*obr>l(4);  L4 = [0 l(4)]; end  
L5 = linspace(0,l(5),2*l(5)/obr);  if 2*obr>l(5);  L5 = [0 l(5)]; end   
L6 = linspace(0,l(6),2*l(6)/obr);  if 2*obr>l(6);  L6 = [0 l(6)]; end  




%%%%%%%%%%%%%% ΚΑΝΟΝΙΚΟΠΟΙΗΣΗ ΤΟΥ ΚΕΡΔΟΥΣ ΤΗΣ 2ΗΣ ΥΠΟΕΡΓΑΣΙΑΣ %%%%%%%%%%%%%
n = length([H L1 L2 L3 L4 L5 L6])- 6; %συνολικα κρισιμα σημεια
kc = kc/n;      %βλεπε και αναφορα (λειπει το προσημο θα το βαλω στα ξ)


%% ****************** ΚΙΝΗΜΑΤΙΚΗ ΠΡΟΣΟΜΟΙΩΣΗ ********************* %%
%%  ΕΠΑΝΑΛΗΨΗ ΔΙΑΔΙΚΑΣΙΑΣ ΜΕΧΡΙ ΤΟ ΠΕΡΑΣ ΤΟΥ ΧΡΟΝΙΚΟΥ ΔΙΑΣΤΗΜΑΤΟΣ  %% 
while (tt<=Tf)&&(stop==0)
    
   %ΥΠΟΛΟΓΙΣΜΟΣ ΒΟΗΘΗΤΙΚΩΝ ΤΡΙΓΩΝΟΜΕΤΡΙΚΩΝ ΣΕ ΚΑΘΕ ΒΗΜΑ
   s1 = sin(qd(tk,1)); 
   s12 = sin(qd(tk,1)+qd(tk,2)); 
   s123 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3));
   s1234 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4));
   s12345 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5));
   s123456 = sin(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6));
   c1 = cos(qd(tk,1)); 
   c12 = cos(qd(tk,1)+qd(tk,2)); 
   c123 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3));
   c1234 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4));
   c12345 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5));
   c123456 = cos(qd(tk,1)+qd(tk,2)+qd(tk,3)+qd(tk,4)+qd(tk,5)+qd(tk,6));
   
   
   %ΟΡΘΟ ΓΕΩΜΕΤΡΙΚΟ ΜΟΝΤΕΛΟ - ΘΕΣΕΙΣ ΤΩΝ ΑΡΘΡΩΣΕΩΝ
   p0x(tk,:) = dd(tk);
   p0y(tk,:) = 0;
   p1x(tk,:) = dd(tk) + 0*H(1:end-1);
   p1y(tk,:) = h - H(1:end-1);
   p2x(tk,:) = p1x(tk,1) - L1(end:-1:2)*s1; 
   p2y(tk,:) = p1y(tk,1) + L1(end:-1:2)*c1;
   p3x(tk,:) = p2x(tk,1) - L2(end:-1:2)*s12; 
   p3y(tk,:) = p2y(tk,1) + L2(end:-1:2)*c12;
   p4x(tk,:) = p3x(tk,1) - L3(end:-1:2)*s123;
   p4y(tk,:) = p3y(tk,1) + L3(end:-1:2)*c123;
   p5x(tk,:) = p4x(tk,1) - L4(end:-1:2)*s1234;
   p5y(tk,:) = p4y(tk,1) + L4(end:-1:2)*c1234;
   p6x(tk,:) = p5x(tk,1) - L5(end:-1:2)*s12345; %παιρνω και την τελευταια αρθρωση
   p6y(tk,:) = p5y(tk,1) + L5(end:-1:2)*c12345;
   p7x(tk,:) = p6x(tk,1) - L6(end:-1:2)*s123456;
   p7y(tk,:) = p6y(tk,1) + L6(end:-1:2)*c123456;
   
   
   %ΥΠΟΛΟΓΙΣΜΟΣ ΙΑΚΩΒΙΑΝΗΣ ΠΟΥ ΧΡΕΙΑΖΟΜΑΣΤΕ ΓΙΑ ΤΟ ΤΕΛΙΚΟ ΣΤΟΙΧΕΙΟ
   Jac(1,1) = 1;
   Jac(1,2) = -p7y(tk,1) + h;
   Jac(1,3) = Jac(1,2) + l(1)*c1;
   Jac(1,4) = Jac(1,3) + l(2)*c12;
   Jac(1,5) = Jac(1,4) + l(3)*c123;
   Jac(1,6) = Jac(1,5) + l(4)*c1234;
   Jac(1,7) = Jac(1,6) + l(5)*c12345;  
   Jac(2,1) = 0;
   Jac(2,2) = p7x(tk,1) - dd(tk);
   Jac(2,3) = Jac(2,2) + l(1)*s1;
   Jac(2,4) = Jac(2,3) + l(2)*s12;
   Jac(2,5) = Jac(2,4) + l(3)*s123;
   Jac(2,6) = Jac(2,5) + l(4)*s1234;
   Jac(2,7) = Jac(2,6) + l(5)*s12345;

   %ΥΠΟΛΟΓΙΣΜΟΣ ΨΕΥΔΟΑΝΤΙΣΤΡΟΦΗΣ ΙΑΚΩΒΙΑΝΗΣ ΜΗΤΡΑΣ
   Jac_psinv = pinv(Jac); %το matlab κανει με SVD
  
  
   
   %%%%%%%%%%%%%%%%%%%%%%%%%% 1Η ΥΠΟΕΡΓΑΣΙΑ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %ΘΕΣΗ ΣΤΟΧΟΥ
   xdes(tk) = xde;
   ydes(tk) = yde; 
   %ΤΑΧΥΤΗΤΑ ΣΤΟΧΟΥ
   if tk>1
       vdesx(tk) = (xdes(tk)-xdes(tk-1))/dt;
       vdesy(tk) = (ydes(tk)-ydes(tk-1))/dt;
   end
   
   xde_(tk) = vdesx(tk) + kp1*(xde-p7x(tk,1));  
   yde_(tk) = vdesy(tk) + kp1*(yde-p7y(tk,1));  
   task1 = Jac_psinv * [xde_(tk); yde_(tk)];    
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   
   
   %%%%%%%%%%%%%%%%%%%%%%%%% 2Η ΥΠΟΕΡΓΑΣΙΑ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   p0 = [p0x(tk,:);p0y(tk,:)];  %ουσιαστικά ενα σημείο (η θεση του καρτ)
   p1 = [p1x(tk,:);p1y(tk,:)];  %κρισιμα σημεια κατα μηκος του h (1oυ link)
   p2 = [p2x(tk,:);p2y(tk,:)];  %.. κλπ
   p3 = [p3x(tk,:);p3y(tk,:)];
   p4 = [p4x(tk,:);p4y(tk,:)];
   p5 = [p5x(tk,:);p5y(tk,:)];
   p6 = [p6x(tk,:);p6y(tk,:)];  %κρισιμα σημεία καταμήκος του l5 (6oυ link)
   p7 = [p7x(tk,:);p7y(tk,:)];  %δεν τα χρησιμοποιώ 
   pob1 = [xob1;yob1];          %θεση του πρωτου εμποδιου
   xob2 = xob1+obdist;
   pob2 = [xob2; yob2];         %θεση του δευτερου εμποδιου
    
   %ΥΠΟΛΟΓΙΣΜΟΣ ΤΟΥ ξ ΛΟΓΩ ΤΗΣ ΘΕΣΗΣ ΤΟΥ ΚΑΡΤ
   ksi_0(1,1) = -1/(norm(p0-pob1)^2-obr^2)^2*2*(p0x(tk)-xob1)- 1/(norm(p0-pob2)^2-obr^2)^2*2*(p0x(tk)-xob2);
   ksi_0(2,1) = 0; ksi_0(3,1) = 0; ksi_0(4,1) = 0; ksi_0(5,1) = 0; ksi_0(6,1) = 0; ksi_0(7,1) = 0;
   
   %ΥΠΟΛΟΓΙΣΜΟΣ ΟΛΩΝ ΤΩΝ ξ ΛΟΓΩ ΤΩΝ ΚΡΙΣΙΜΩΝ ΣΗΜΕΙΩΝ ΚΑΤΑ ΜΗΚΟΣ ΤΟΥ ΠΡΩΤΟΥ
   %ΣΥΝΔΕΣΜΟΥ (MHKOYΣ h) ΚΑΙ ΣΤΗ ΣΥΝΕΧΕΙΑ ΑΘΡΟΙΣΜΑ ΑΥΤΩΝ ΣΤΟ ksi1 
   for i = 1 : size(p1,2)
   ksi_1(1,i) = -1/(norm(p1(:,i)-pob1)^2-obr^2)^2*2*(p1x(tk,i)-xob1)- 1/(norm(p1(:,i)-pob2)^2-obr^2)^2*2*(p1x(tk,i)-xob2);
   ksi_1(2,i) = 0; ksi_1(3,i) = 0; ksi_1(4,i) = 0; ksi_1(5,i) = 0; ksi_1(6,i) = 0; ksi_1(7,i) = 0;
   end
   ksi1 = sum(ksi_1,2);
   
   %ΥΠΟΛΟΓΙΣΜΟΣ ΟΛΩΝ ΤΩΝ ξ ΛΟΓΩ ΤΩΝ ΚΡΙΣΙΜΩΝ ΣΗΜΕΙΩΝ ΚΑΤΑ ΜΗΚΟΣ ΤΟΥ ΔΕΥΤΕΡΟΥ
   %ΣΥΝΔΕΣΜΟΥ (MHKOYΣ l1) ΚΑΙ ΣΤΗ ΣΥΝΕΧΕΙΑ ΑΘΡΟΙΣΜΑ ΑΥΤΩΝ ΣΤΟ ksi2 
   for i = 1 : size(p2,2)
   ksi_2(1,i) = -1/(norm(p2(:,i)-pob1)^2-obr^2)^2*2*(p2x(tk,i)-xob1)- 1/(norm(p2(:,i)-pob2)^2-obr^2)^2 *2*(p2x(tk,i)-xob2);
   ksi_2(2,i) = -1/(norm(p2(:,i)-pob1)^2-obr^2)^2*2*((p2x(tk,i)-xob1)*(-L1(end-i+1)*c1)+ (p2y(tk,i)-yob1)*(-L1(end-i+1)*s1))...
       - 1/(norm(p2(:,i)-pob2)^2-obr^2)^2 * 2*((p2x(tk,i)-xob2)*(-L1(end-i+1)*c1)+ (p2y(tk,i)-yob2)*(-L1(end-i+1)*s1));
   ksi_2(3,i) = 0; ksi_2(4,i) = 0; ksi_2(5,i) = 0; ksi_2(6,i) = 0; ksi_2(7,i) = 0;
   end
   ksi2 = sum(ksi_2,2);
   
   %...κλπ ΤΟΥ ΤΡΙΤΟΥ
   for i = 1 : size(p3,2)
   ksi_3(1,i) = -1/(norm(p3(:,i)-pob1)^2-obr^2)^2*2*(p3x(tk,i)-xob1)- 1/(norm(p3(:,i)-pob2)^2-obr^2)^2 *2*(p3x(tk,i)-xob2);
   ksi_3(2,i) = -1/(norm(p3(:,i)-pob1)^2-obr^2)^2*2*((p3x(tk,i)-xob1)*(-l(1)*c1-L2(end-i+1)*c12)+ (p3y(tk,i)-yob1)*(-l(1)*s1-L2(end-i+1)*s12))...
       - 1/(norm(p3(:,i)-pob2)^2-obr^2)^2 * 2*((p3x(tk,i)-xob2)*(-l(1)*c1-L2(end-i+1)*c12)+ (p3y(tk,i)-yob2)*(-l(1)*s1-L2(end-i+1)*s12));
   ksi_3(3,i) = -1/(norm(p3(:,i)-pob1)^2-obr^2)^2*2*((p3x(tk,i)-xob1)*(-L2(end-i+1)*c12)+ (p3y(tk,i)-yob1)*(-L2(end-i+1)*s12))...
       - 1/(norm(p3(:,i)-pob2)^2-obr^2)^2 * 2*((p3x(tk,i)-xob2)*(-L2(end-i+1)*c12)+ (p3y(tk,i)-yob2)*(-L2(end-i+1)*s12));
   ksi_3(4,i) = 0; ksi_3(5,i) = 0; ksi_3(6,i) = 0; ksi_3(7,i) = 0;
   end
   ksi3 = sum(ksi_3,2);
   
   %ΤΟΥ ΤΕΤΑΡΤΟΥ
   for i = 1 : size(p4,2)
   ksi_4(1,i) = -1/(norm(p4(:,i)-pob1)^2-obr^2)^2*2*(p4x(tk,i)-xob1)- 1/(norm(p4(:,i)-pob2)^2-obr^2)^2 *2*(p4x(tk,i)-xob2);
   ksi_4(2,i) = -1/(norm(p4(:,i)-pob1)^2-obr^2)^2*2*((p4x(tk,i)-xob1)*(-l(1)*c1-l(2)*c12-L3(end-i+1)*c123)+ (p4y(tk,i)-yob1)*(-l(1)*s1-l(2)*s12-L3(end-i+1)*s123))...
       - 1/(norm(p4(:,i)-pob2)^2-obr^2)^2 * 2*((p4x(tk,i)-xob2)*(-l(1)*c1-l(2)*c12-L3(end-i+1)*c123)+ (p4y(tk,i)-yob2)*(-l(1)*s1-l(2)*s12-L3(end-i+1)*s123));
   ksi_4(3,i) = -1/(norm(p4(:,i)-pob1)^2-obr^2)^2*2*((p4x(tk,i)-xob1)*(-l(2)*c12-L3(end-i+1)*c123)+ (p4y(tk,i)-yob1)*(-l(2)*s12-L3(end-i+1)*s123))...
       - 1/(norm(p4(:,i)-pob2)^2-obr^2)^2 * 2*((p4x(tk,i)-xob2)*(-l(2)*c12-L3(end-i+1)*c123)+ (p4y(tk,i)-yob2)*(-l(2)*s12-L3(end-i+1)*s123));
   ksi_4(4,i) = -1/(norm(p4(:,i)-pob1)^2-obr^2)^2*2*((p4x(tk,i)-xob1)*(-L3(end-i+1)*c123)+ (p4y(tk,i)-yob1)*(-L3(end-i+1)*s123))...
       - 1/(norm(p4(:,i)-pob2)^2-obr^2)^2 * 2*((p4x(tk,i)-xob2)*(-L3(end-i+1)*c123)+ (p4y(tk,i)-yob2)*(-L3(end-i+1)*s123));
   ksi_4(5,i) = 0; ksi_4(6,i) = 0; ksi_4(7,i) = 0;
   end
   ksi4 = sum(ksi_4,2);
   
   %ΤΟΥ ΠΕΜΠΤΟΥ;
   for i = 1 : size(p5,2)
   ksi_5(1,i) = -1/(norm(p5(:,i)-pob1)^2-obr^2)^2*2*(p5x(tk,i)-xob1)- 1/(norm(p5(:,i)-pob2)^2-obr^2)^2 *2*(p5x(tk,i)-xob2);
   ksi_5(2,i) = -1/(norm(p5(:,i)-pob1)^2-obr^2)^2*2*((p5x(tk,i)-xob1)*(-l(1)*c1-l(2)*c12-l(3)*c123-L4(end-i+1)*c1234)+ (p5y(tk,i)-yob1)*(-l(1)*s1-l(2)*s12-l(3)*s123-L4(end-i+1)*s1234))...
       - 1/(norm(p5(:,i)-pob2)^2-obr^2)^2 * 2*((p5x(tk,i)-xob2)*(-l(1)*c1-l(2)*c12-l(3)*c123-L4(end-i+1)*c1234)+ (p5y(tk,i)-yob2)*(-l(1)*s1-l(2)*s12-l(3)*s123-L4(end-i+1)*s1234));
   ksi_5(3,i) = -1/(norm(p5(:,i)-pob1)^2-obr^2)^2*2*((p5x(tk,i)-xob1)*(-l(2)*c12-l(3)*c123-L4(end-i+1)*c1234)+ (p5y(tk,i)-yob1)*(-l(2)*s12-l(3)*s123-L4(end-i+1)*s1234))...
       - 1/(norm(p5(:,i)-pob2)^2-obr^2)^2 * 2*((p5x(tk,i)-xob2)*(-l(2)*c12-l(3)*c123-L4(end-i+1)*c1234)+ (p5y(tk,i)-yob2)*(-l(2)*s12-l(3)*s123-L4(end-i+1)*s1234));
   ksi_5(4,i) = -1/(norm(p5(:,i)-pob1)^2-obr^2)^2*2*((p5x(tk,i)-xob1)*(-l(3)*c123-L4(end-i+1)*c1234)+ (p5y(tk,i)-yob1)*(-l(3)*s123-L4(end-i+1)*s1234))...
       - 1/(norm(p5(:,i)-pob2)^2-obr^2)^2 * 2*((p5x(tk,i)-xob2)*(-l(3)*c123-L4(end-i+1)*c1234)+ (p5y(tk,i)-yob2)*(-l(3)*s123-L4(end-i+1)*s1234));
   ksi_5(5,i) = -1/(norm(p5(:,i)-pob1)^2-obr^2)^2*2*((p5x(tk,i)-xob1)*(-L4(end-i+1)*c1234)+ (p5y(tk,i)-yob1)*(-L4(end-i+1)*s1234))...
       - 1/(norm(p5(:,i)-pob2)^2-obr^2)^2 * 2*((p5x(tk,i)-xob2)*(-L4(end-i+1)*c1234)+ (p5y(tk,i)-yob2)*(-L4(end-i+1)*s1234));
   ksi_5(6,i) = 0; ksi_5(7,i) = 0;
   end
   ksi5 = sum(ksi_5,2);
   
   %ΤΟΥ ΕΚΤΟΥ
   for i = 1 : size(p6,2)
   ksi_6(1,i) = -1/(norm(p6(:,i)-pob1)^2-obr^2)^2*2*(p6x(tk,i)-xob1)- 1/(norm(p6(:,i)-pob2)^2-obr^2)^2 *2*(p6x(tk,i)-xob2);
   ksi_6(2,i) = -1/(norm(p6(:,i)-pob1)^2-obr^2)^2*2*((p6x(tk,i)-xob1)*(-l(1)*c1-l(2)*c12-l(3)*c123-l(4)*c1234-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob1)*(-l(1)*s1-l(2)*s12-l(3)*s123-l(4)*s1234-L5(end-i+1)*s12345))...
       - 1/(norm(p6(:,i)-pob2)^2-obr^2)^2 * 2*((p6x(tk,i)-xob2)*(-l(1)*c1-l(2)*c12-l(3)*c123-l(4)*c1234-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob2)*(-l(1)*s1-l(2)*s12-l(3)*s123-l(4)*s1234-L5(end-i+1)*s12345));
   ksi_6(3,i) = -1/(norm(p6(:,i)-pob1)^2-obr^2)^2*2*((p6x(tk,i)-xob1)*(-l(2)*c12-l(3)*c123-l(4)*c1234-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob1)*(-l(2)*s12-l(3)*s123-l(4)*s1234-L5(end-i+1)*s12345))...
       - 1/(norm(p6(:,i)-pob2)^2-obr^2)^2 * 2*((p6x(tk,i)-xob2)*(-l(2)*c12-l(3)*c123-l(4)*c1234-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob2)*(-l(2)*s12-l(3)*s123-l(4)*s1234-L5(end-i+1)*s12345));
   ksi_6(4,i) = -1/(norm(p6(:,i)-pob1)^2-obr^2)^2*2*((p6x(tk,i)-xob1)*(-l(3)*c123-l(4)*c1234-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob1)*(-l(3)*s123-l(4)*s1234-L5(end-i+1)*s12345))...
       - 1/(norm(p6(:,i)-pob2)^2-obr^2)^2 * 2*((p6x(tk,i)-xob2)*(-l(3)*c123-l(4)*c1234-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob2)*(-l(3)*s123-l(4)*s1234-L5(end-i+1)*s12345));
   ksi_6(5,i) = -1/(norm(p6(:,i)-pob1)^2-obr^2)^2*2*((p6x(tk,i)-xob1)*(-l(4)*c1234-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob1)*(-l(4)*s1234-L5(end-i+1)*s12345))...
       - 1/(norm(p6(:,i)-pob2)^2-obr^2)^2 * 2*((p6x(tk,i)-xob2)*(-l(4)*c1234-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob2)*(-l(4)*s1234-L5(end-i+1)*s12345));
   ksi_6(6,i) = -1/(norm(p6(:,i)-pob1)^2-obr^2)^2*2*((p6x(tk,i)-xob1)*(-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob1)*(-L5(end-i+1)*s12345))...
       - 1/(norm(p6(:,i)-pob2)^2-obr^2)^2 * 2*((p6x(tk,i)-xob2)*(-L5(end-i+1)*c12345)+ (p6y(tk,i)-yob2)*(-L5(end-i+1)*s12345));
   ksi_6(7,i) = 0;
   end
   ksi6 = sum(ksi_6,2);
   
   %ΤΟΥ ΤΕΛΕΥΤΑΙΟΥ ΣΥΝΔΕΣΜΟΥ
   for i = 1 : size(p7,2)
   ksi_7(1,i) = -1/(norm(p7(:,i)-pob1)^2-obr^2)^2*2*(p7x(tk,i)-xob1)- 1/(norm(p7(:,i)-pob2)^2-obr^2)^2 *2*(p7x(tk,i)-xob2);
   ksi_7(2,i) = -1/(norm(p7(:,i)-pob1)^2-obr^2)^2*2*((p7x(tk,i)-xob1)*(-l(1)*c1-l(2)*c12-l(3)*c123-l(4)*c1234-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob1)*(-l(1)*s1-l(2)*s12-l(3)*s123-l(4)*s1234-l(5)*s12345-L6(end-i+1)*s123456))...
       - 1/(norm(p7(:,i)-pob2)^2-obr^2)^2 * 2*((p7x(tk,i)-xob2)*(-l(1)*c1-l(2)*c12-l(3)*c123-l(4)*c1234-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob2)*(-l(1)*s1-l(2)*s12-l(3)*s123-l(4)*s1234-l(5)*s12345-L6(end-i+1)*s123456));
   ksi_7(3,i) = -1/(norm(p7(:,i)-pob1)^2-obr^2)^2*2*((p7x(tk,i)-xob1)*(-l(2)*c12-l(3)*c123-l(4)*c1234-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob1)*(-l(2)*s12-l(3)*s123-l(4)*s1234-l(5)*s12345-L6(end-i+1)*s123456))...
       - 1/(norm(p7(:,i)-pob2)^2-obr^2)^2 * 2*((p7x(tk,i)-xob2)*(-l(2)*c12-l(3)*c123-l(4)*c1234-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob2)*(-l(2)*s12-l(3)*s123-l(4)*s1234-l(5)*s12345-L6(end-i+1)*s123456));
   ksi_7(4,i) = -1/(norm(p7(:,i)-pob1)^2-obr^2)^2*2*((p7x(tk,i)-xob1)*(-l(3)*c123-l(4)*c1234-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob1)*(-l(3)*s123-l(4)*s1234-l(5)*s12345-L6(end-i+1)*s123456))...
       - 1/(norm(p7(:,i)-pob2)^2-obr^2)^2 * 2*((p7x(tk,i)-xob2)*(-l(3)*c123-l(4)*c1234-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob2)*(-l(3)*s123-l(4)*s1234-l(5)*s12345-L6(end-i+1)*s123456));
   ksi_7(5,i) = -1/(norm(p7(:,i)-pob1)^2-obr^2)^2*2*((p7x(tk,i)-xob1)*(-l(4)*c1234-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob1)*(-l(4)*s1234-l(5)*s12345-L6(end-i+1)*s123456))...
       - 1/(norm(p7(:,i)-pob2)^2-obr^2)^2* 2*((p7x(tk,i)-xob2)*(-l(4)*c1234-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob2)*(-l(4)*s1234-l(5)*s12345-L6(end-i+1)*s123456));
   ksi_7(6,i) = -1/(norm(p7(:,i)-pob1)^2-obr^2)^2*2*((p7x(tk,i)-xob1)*(-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob1)*(-l(5)*s12345-L6(end-i+1)*s123456))...
       - 1/(norm(p7(:,i)-pob2)^2-obr^2)^2 * 2*((p7x(tk,i)-xob2)*(-l(5)*c12345-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob2)*(-l(5)*s12345-L6(end-i+1)*s123456));
   ksi_7(7,i) = -1/(norm(p7(:,i)-pob1)^2-obr^2)^2*2*((p7x(tk,i)-xob1)*(-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob1)*(-L6(end-i+1)*s123456))...
       - 1/(norm(p7(:,i)-pob2)^2-obr^2)^2 * 2*((p7x(tk,i)-xob2)*(-L6(end-i+1)*c123456)+ 2*(p7y(tk,i)-yob2)*(-L6(end-i+1)*s123456));
   end
   ksi7 = sum(ksi_7,2);
   
   
   %ΑΘΡΟΙΣΜΑ ΟΛΩΝ ΤΩΝ ξ 
   ksi = -1*(ksi_0 + ksi1 + ksi2+ ksi3+ ksi4+ ksi5+ ksi6 + ksi7); %συνολικό ξ
   H2 = eye(7)*kc; %δεν δινω βαρυτητα σε καποια αρθρωση
   task2 = (eye(7)-Jac_psinv*Jac)*H2*ksi; %2η υποεργασια
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   
   
   %ΥΠΟΛΟΓΙΣΜΟΣ ΤΑΧΥΤΗΤΩΝ ΤΩΝ ΑΡΘΡΩΣΕΩΝ
   dd_(tk)   = task1(1)+task2(1); %πρισματικη αρθρωση
   qd_(tk,:) = task1(2:end)'+task2(2:end)'; %στροφικες αρθρωσεις
   
   
   
   %ΥΠΟΛΟΓΙΣΜΟΣ ΤΩΝ ΕΠΟΜΕΝΩΝ ΕΠΙΘΥΜΗΤΩΝ ΜΕΤΑΤΟΠΙΣΕΩΝ/ΓΩΝΙΩΝ ΤΩΝ ΑΡΘΡΩΣΕΩΝ
   dd(tk+1)   = dd(tk)   + dt*dd_(tk);   %πρισματικη αρθρωση
   qd(tk+1,1) = qd(tk,1) + dt*qd_(tk,1); %στροφικες αρθρωσεις
   qd(tk+1,2) = qd(tk,2) + dt*qd_(tk,2);
   qd(tk+1,3) = qd(tk,3) + dt*qd_(tk,3);
   qd(tk+1,4) = qd(tk,4) + dt*qd_(tk,4);
   qd(tk+1,5) = qd(tk,5) + dt*qd_(tk,5);
   qd(tk+1,6) = qd(tk,6) + dt*qd_(tk,6);
   
   
   
   pause(0.001); %μικρη παυση για αποφυγη συσσωρευσης
   
   yob2 = yob1;        %ανανεωση της θέσης των δεξιων εμποδίων
   xob2 = xob1+obdist; %καθώς αλλάζουμε μονο το αριστερο απο το GUI
   
%%%%%%%%%%%%%%%%%%%  ΑΠΕΙΚΟΝΙΣΗ ΤΗΣ ΠΡΟΣΟΜΟΙΩΣΗΣ %%%%%%%%%%%%%%%%%%%%%%%%%%
      if(mod(tk,dtk)==0)          %καθε dtk δειγματα
        cla;    				  %καθαρισμα σε καθε βημα
        plot(xde, yde, 'o','MarkerSize',10,'MarkerFaceColor','g');     %επιθυμητo σημειο
        
        x1 = linspace(0,2*pi,50); y1 = x1;
        x1 = xob1 + obr*sin(x1);  x2 = x1 + obdist;
        y1 = yob1 + obr*cos(y1);  y2 = y1;
        fill(x1,y1,'r');fill(x2,y2,'r');
        plot(-1/2*(sum(l)+h),0,'m^','MarkerSize',10,'MarkerFaceColor','m');             %αρχικη θεση χρονου
        plot(-1/2*(sum(l)+h),(sum(l)+h),'mv','MarkerSize',10,'MarkerFaceColor','m'); %τελικη θεση χρονου
        plot(-1/2*(sum(l)+h),tt*(sum(l)+h)/Tf,'r*','MarkerSize',10);                       %χρονικη στιγμη

        
  		plot([p0x(tk,1), p1x(tk,1)], [p0y(tk,1), p1y(tk,1)] , 'k');	  	
  		plot([p1x(tk,1), p2x(tk,1)], [p1y(tk,1), p2y(tk,1)] , 'k');	  	
  		plot([p2x(tk,1), p3x(tk,1)], [p2y(tk,1), p3y(tk,1)] , 'k');
     	plot([p3x(tk,1), p4x(tk,1)], [p3y(tk,1), p4y(tk,1)] , 'k');
        plot([p4x(tk,1), p5x(tk,1)], [p4y(tk,1), p5y(tk,1)] , 'k');
        plot([p5x(tk,1), p6x(tk,1)], [p5y(tk,1), p6y(tk,1)] , 'k');
        plot([p6x(tk,1), p7x(tk,1)], [p6y(tk,1), p7y(tk,1)] , 'k');
        
        plot(p0x(tk,1), p0y(tk,1),'ks','LineWidth',2,'MarkerSize',15,'MarkerFaceColor','w');
        plot(p1x(tk,1), p1y(tk,1),'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
        plot(p2x(tk,1), p2y(tk,1),'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
        plot(p3x(tk,1), p3y(tk,1),'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
        plot(p4x(tk,1), p4y(tk,1),'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
        plot(p5x(tk,1), p5y(tk,1),'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
        plot(p6x(tk,1), p6y(tk,1),'ko','LineWidth',2,'MarkerSize',10,'MarkerFaceColor','w');
        plot(p7x(tk,1), p7y(tk,1),'kx','LineWidth',2,'MarkerSize',10);
        
        plot(p1x(tk,1), p1y(tk,1),'ko','MarkerSize',2,'MarkerFaceColor','k');
        plot(p2x(tk,1), p2y(tk,1),'ko','MarkerSize',2,'MarkerFaceColor','k');
        plot(p3x(tk,1), p3y(tk,1),'ko','MarkerSize',2,'MarkerFaceColor','k');
        plot(p4x(tk,1), p4y(tk,1),'ko','MarkerSize',2,'MarkerFaceColor','k');
        plot(p5x(tk,1), p5y(tk,1),'ko','MarkerSize',2,'MarkerFaceColor','k');
        plot(p6x(tk,1), p6y(tk,1),'ko','MarkerSize',2,'MarkerFaceColor','k');
        
        title('Kinematic Simulation');
      end
      
   tt=tt+dt;
   tk=tk+1;
end

 %%% ΣΧΕΔΙΑΣΗ ΤΗΣ ΑΠΟΣΤΑΣΗΣ ΤΟΥ ΤΕΛΙΚΟΥ ΣΤΟΙΧΕΙΟΥ ΔΡΑΣΗΣ ΑΠΟ ΤΟ ΣΤΟΧΟ %%%%%
 figure; hold on;
 plot(((p7x(1:tk-1,1)-xdes(1,1:tk-1)').^2 ... 
     +(p7y(1:tk-1,1)-ydes(1,1:tk-1)').^2).^(1/2));
 plot(1:tk-1,step,'g');
 ylabel('(m)');
 xlabel('steps of simulation');
 title('distance between end effector and target');
 legend('distance','step size');
 pause(0.5);
 
 
%%%%%%%%%% ΣΧΕΔΙΑΣΗ ΤΩΝ ΜΕΤΑΤΟΠΙΣΕΩΝ/ΓΩΝΙΩΝ ΤΩΝ ΑΡΘΡΩΣΕΩΝ %%%%%%%%%%%%%%%%% 
figure('Position',[100 100 scrsz(3)-200 scrsz(4)-200]); 
subplot(3,3,2);
plot(dd);
title('\fontsize{12}θεση πρισματικης αρθρωσης');
xlabel('simulation steps');

subplot(3,3,4);
plot(qd(:,1));
title('\fontsize{12}γωνια 1ης στροφικης αρθρωσης');
xlabel('simulation steps');

subplot(3,3,5);
plot(qd(:,2));
title('\fontsize{12}γωνια 2ης στροφικης αρθρωσης');
xlabel('simulation steps');

subplot(3,3,6);
plot(qd(:,3));
title('\fontsize{12}γωνια 3ης στροφικης αρθρωσης');
xlabel('simulation steps');
 
subplot(3,3,7);
plot(qd(:,4));
title('\fontsize{12}γωνια 4ης στροφικης αρθρωσης');
xlabel('simulation steps');
 
subplot(3,3,8);
plot(qd(:,5));
title('\fontsize{12}γωνια 5ης στροφικης αρθρωσης');
xlabel('simulation steps');
 
subplot(3,3,9);
plot(qd(:,6));
title('\fontsize{12}γωνια 6ης στροφικης αρθρωσης');
xlabel('simulation steps'); 
 
 

 


