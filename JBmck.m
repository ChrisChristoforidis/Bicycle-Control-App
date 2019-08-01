% JBmck.m

% Created by Arend L. Schwab and Jim Papadopoulos, 18-Jun-2003
% Copyright © 2003-2006 Schwab, Papadopoulos, Ruina, & Dressel
% Delft University of Technology & Cornell University

% This Matlab script calculates from the bicycle parameters
% the matrices of the linearized equations of motion.
% The degrees of freedom are q=[lean angle, steer angle],
% lean to the right and steer to the left is positive
% when driving forward.
% The linearized equations of motion read:
%
%       M0*qdd+(C1.*v)*qd+(K0+K2.*v^2)*q=0
%
% where v is the forward speed of the bike.
% 
% The outcome of this script is:
% the massmatrix M0
% the velocity sensetivity matrix C1 
% the stifness matrices K0 and K2

% Modification History
% 11/15/05 AED - display of M0, C1, K0, and K2 moved to here

lambda = pi/2-head_angle;

masses=[mfrontwheel
        mfork
        mbasket
        mrearwheel
        mframe
        mrack
        mrider];

Rl=[cos(lambda) -sin(lambda); sin(lambda) cos(lambda)];
Ol=[wheelbase+trail; 0];
p=Ol+Rl*[ucmfork; vcmfork];
xcmfork=p(1);
ycmfork=p(2);
p=Ol+Rl*[ucmbasket; vcmbasket];
xcmbasket=p(1);
ycmbasket=p(2);

cms=[wheelbase Dfrontwheel/2
     xcmfork   ycmfork
     xcmbasket ycmbasket
     0         Drearwheel/2
     xcmframe  ycmframe
     xcmrack   ycmrack
     xcmrider  ycmrider];
     
Icms=[Ifrontwheel
      Ifork
      Ibasket
      Irearwheel 
      Iframe
      Irack
      Irider];
    
aIcms=[alphaIfrontwheel
       alphaIfork
       alphaIbasket
       alphaIrearwheel 
       alphaIframe
       alphaIrack
       alphaIrider];
     
fa=[1 2 3];
[mf,cmf,Ifcm]=JBaddb(masses(fa),cms(fa,:),Icms(fa,:),aIcms(fa));
[mt,cmt,Itcm]=JBaddb(masses,cms,Icms,aIcms);


%ItO is total inertia at the rear contact pnt: the origin O.
cmtO=[cmt(1); cmt(2); 0];
ItO=Itcm+mt.*(cmtO.'*cmtO*eye(3)-cmtO*cmtO.');
Txx=ItO(1,1);
Txy=ItO(1,2);
Tyy=ItO(2,2);

Rl = [cos(lambda) -sin(lambda);sin(lambda) cos(lambda)];
cmfuv = Rl.'*(cmf.'-[wheelbase+trail;0]);
d = cmfuv(1);

ldir = [-sin(lambda); cos(lambda); 0];
xdir = [1;0;0];
ydir = [0;1;0];

%Fll mass moment of inertia of front assembly along steering axis
Fll =      mf*d*d+ldir.'*Ifcm*ldir;
%Flx cross product of moment inertia of front assembly at 
% the intersection of the steer-axis and the x-axis 
% about the steer-axis and the x-axis
Flx =-mf*cmf(2)*d+xdir.'*Ifcm*ldir;
%Fly cross product of moment inertia of front assembly at 
% the intersection of the steer-axis and the y-axis through
% the rear contact point about the steer-axis and the y-axis
Fly = mf*cmf(1)*d+ydir.'*Ifcm*ldir;

cw=wheelbase;
cf=trail*cos(lambda);

% Hr is the angular momentum of the rear wheel along the z-axis
% Sr = Hr/v = Jr/Rr
Sr=Irearwheel(3)/(Drearwheel/2);
% Hf is the angular momentum of the front wheel along the z-axis
% Sf = Hf/v = Jf/Rf
Sf=Ifrontwheel(3)/(Dfrontwheel/2);
St=Sr+Sf;

nu=mf*d+mt*cmt(1)*(cf/cw);

%now its four matrices
M0=zeros(2);
M0(1,1)=Txx;
M0(1,2)=Flx+(cf/cw)*Txy;
M0(2,1)=M0(1,2);
M0(2,2)=Fll+2*(cf/cw)*Fly+(cf/cw)^2*Tyy;

C1=zeros(2);
C1(1,1)=0;
C1(1,2)=-(Sf*cos(lambda)+(cf/cw)*St)+Txy*cos(lambda)/cw-(cf/cw)*mt*cmt(2);
C1(2,1)=Sf*cos(lambda)+(cf/cw)*St;
C1(2,2)=cos(lambda)/cw*Fly+(cf/cw)*(nu+cos(lambda)/cw*Tyy);

g=gravity;

K0=zeros(2);
K0(1,1)=-g*mt*cmt(2);
K0(1,2)=g*nu;
K0(2,1)=K0(1,2);
K0(2,2)=-g*nu*sin(lambda);

K2=zeros(2);
K2(1,1)=0;
K2(1,2)=-cos(lambda)/cw*(St+mt*cmt(2));
K2(2,1)=0;
K2(2,2)=cos(lambda)/cw*(sin(lambda)*Sf+nu);
% 
%   M0
%   C1
%   K0
%   K2
