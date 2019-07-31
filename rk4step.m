function [t,y] = rk4step(t0,y0,h,v,Gp,K,w)
  t=t0;
  y=y0;
  k1=funhan(t,y,v,Gp,K,w);
  k2=funhan(t+h/2,y+(h/2).*k1,v,Gp,K,w);
  k3=funhan(t+h/2,y+(h/2).*k2,v,Gp,K,w);
  k4=funhan(t+h,y+h.*k3,v,Gp,K,w);
  t=t+h;
  y=y+(h/6).*(k1+2.*(k2+k3)+k4);
end