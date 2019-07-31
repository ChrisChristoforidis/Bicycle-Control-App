function [mt,cmt,It]=JBaddb(m,cm,I,a)

% Created by Arend L. Schwab and Jim Papadopoulos, 18-Jun-2003
% Copyright © 2003-2006 Schwab, Papadopoulos, Ruina, & Dressel
% Delft University of Technology & Cornell University

% add the mass and inertia of rigid bodies into one body
% Modification History
%10/03/06 AED  - Better handle case of no mass at all,
%                prevent NaN in return value(s)

n=length(m);

mt=sum(m);
%centre of mass
if mt == 0            % if no mass
    cmt=(m.'*cm).*0;  % set to zeros, not NaN
else                  % otherwise
    cmt=(m.'*cm)./mt; % calculate as usual
end
%total inertia
It=zeros(3);
for i=1:n
  R=[cos(a(i)) -sin(a(i)) 0; sin(a(i)) cos(a(i)) 0; 0 0 1];
  Iuvz=diag(I(i,:));
  d=[cm(i,1); cm(i,2); 0]-[cmt(1); cmt(2); 0];
  Ixyz=R*Iuvz*R.'+m(i).*(d.'*d*eye(3)-d*d.');
  It=It+Ixyz;
end