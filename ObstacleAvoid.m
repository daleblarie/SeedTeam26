clc

start = [2 2 -2];
goal = [20 20 20];

x = -5:.5:22;
y = -5:.5:22;
z = -5:.5:22;

% field = 

% attractive constant
attConst = 5;

% repulsive constant
repConst = 2;

%% Plot obstacles
%sphere center and radius
spC1 = [3 3 3];
r1 =2;

spC2 = [15 5 0];
r2 = 3;

spC3 = [11 15 12];
r3 = 4;

obsCenters = [spC1; spC2; spC3];
obsRadius = [r1, r2, r3];

[x, y, z] = sphere();
hold on
for i  = 1:size(obsCenters)
    cent = obsCenters(i, :);
    surf(x*obsRadius(i) + cent(1), y*obsRadius(i) + cent(2), z*obsRadius(i) + cent(3));

end

% plot start point
plot3(start(1), start(2), start(3), 'ko')

% plot goal
plot3(goal(1), goal(2), goal(3), 'R*')





%% Methods
% function to calculate distance between a sphere and a point 
function dist = dist2Sphere(pt, sphC, radius)
  dist = sqrt(sum((pt-sphC).^2))-radius;
end 

function dist = eucliDist(point, target)
    dist = sqrt(sum((point-target)^2));
end

function u_att = attractive_potetial(currPt, goal, attConst)
    u_att = 0.5 * attConst * sum((currPt - goal)^2);
end

function [x_att, y_att, z_att] = att_compontents(attConst, currPt, goal)
    x_att = -attConst * (currPt(1) - goal(1));
    y_att = -attConst * (currPt(2) - goal(2));
    z_att = -attConst * (currPt(3) - goal(3));
end

function u_rep = repulsive_potential(currPt, obs, radius, repConst)
    u_rep = 0.5 * ((repConst * radius)/sum((currPt - obs)^2));
end

