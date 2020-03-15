%Conical Helix
%conical spiral with angular frequency a on a cone of height h and radius r
%is a space curve given by the parametric equations
function [Px, Py, Pz] = get_TargetTrajectory(no_step, K)
a = 0.08  ;  % angle frequence
h = 20; %heigh
r = 10 ;% radius
vertical = linspace(20, 0 , no_step);
x = [];
y = [];
z = [];
c= 0;
%% create points of conical helix 
for t = vertical
    z = [z, t];
    x = [x, (h-t)/h*r*cos(a*c*0.2)];
    y = [y, (h-t)/h*r*sin(a*c*0.2)];
    c = c+1;
end

%% targets for last K points
for i = 1:K
    z = [z, z(end)];
    x = [x, x(end)];
    y = [y, y(end)];
end   

Px = x;
Py = y;
Pz = z;
plot3(x, y, z)
title('Conical Helix')