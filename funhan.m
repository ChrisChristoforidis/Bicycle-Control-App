


function dz = funhan(t,z,v,Gp,K,w)

x=z(1:7);

NA=Gp.A;
NB=Gp.B;

u=-[K 0]*x;

dx=NA*x+NB(:,2)*u+NB(:,3)*w;
dz=[dx;v*cos(z(3));v*sin(z(3))];

end
