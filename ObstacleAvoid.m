clc
% clear

start = [2 2 -2];
goal = [20 20 20];

xf = linspace(-5, 22, 10);
yf = linspace(-5, 22, 10);
zf = linspace(-5, 22, 10);

% field = 

% attractive constant
attConst = 5;

% repulsive constant
repConst = 1000;

%% Plot obstacles
%sphere center and radius
spC1 = [3 3 3];
r1 =2;

spC2 = [15 5 0];
r2 = 5;

spC3 = [11 15 12];
r3 = 3;

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

pointField = [];
attPotential = [];
for i = xf
    for j = yf
       for k = zf
          pt = [i j k];
          pointField = [pointField; pt];
          attpot = att_components(attConst, pt, goal);
          attPotential = [attPotential; attpot];
       end
    end
end
% 
repPotential = zeros(size(pointField, 1), 3);
for i  = 1:size(obsCenters)
    i
    cent = obsCenters(i, :);
    r = obsRadius(i);
    repField = zeros(size(pointField, 3));
%     size(repField)
    for k = 1:size(pointField, 1)
        
        reppot = rep_components(repConst, pointField(k, :), cent, r);
        for a = 1:3
            repField(k,a) = reppot(a);
        end
    end
    repPotential = repPotential + repField;
end
% repPotential = repPotential * 10;
temp = repPotential .* 10
% repPotential = repPotential * 10
% potential = attPotential + repPotential*10;
temp = normalize(temp) * 10;
temp = temp * 10;
temp2 = normalize(attPotential);
potential = temp + attPotential;
% potential = attPotential;
% potential = repPotential;
% potential = normalize(potential, 'range');
x = pointField(:,1);
y = pointField(:,2);
z = pointField(:,3);

a = potential(:,1);
b = potential(:,2);
c = potential(:,3);
a = a * 10;
b = b * 10;
c = c * 10;
scale = 2;
quiver3(x,y, z,a, b, c, scale);
% quiver(pointField(:,1), pointField(:,2), potential(:, 1), potential(:, 2))
% 
% c = q.Color;
% q.Color = 'black';

%% Methods
% function to calculate distance between a sphere and a point 
function dist = dist2Sphere(pt, sphC, radius)
  dist = sqrt(sum((pt-sphC).^2))-radius;
end 

function dist = eucliDist(point, target)
    dist = sqrt(sum((point-target)^2));
end

function u_att = attractive_potetial(currPt, goal, attConst)
    u_att = 0.5 * attConst * sum((currPt - goal).^2);
end

function potential_components = att_components(attConst, currPt, goal)
    x_att = -attConst * (currPt(1) - goal(1));
    y_att = -attConst * (currPt(2) - goal(2));
    z_att = -attConst * (currPt(3) - goal(3));
    
    potential_components = [x_att y_att z_att];
end

function u_rep = repulsive_potential(x, y, z, obs, radius, repConst)
    u_rep = 0.5 * ((repConst * radius)/((x-obs(1))^2 + (y-obs(2))^2 + (z-obs(3)^2)));
end

% Repulsive forces
function potential_components = rep_components(repConst, currPt, obs, radius)
%    u_rep = 0.5 * ((repConst * radius)/((x-obs(1))^2 + (y-obs(2))^2 + (z-obs(3))^2));
%    u_rep = 0.5 *  ((repConst)/((x-obs(1))^2 + (y-obs(2))^2 + (z-obs(3))^2));
%    x_rep = -radius^2 *diff(u_rep, x);
%    y_rep = -radius^2 *diff(u_rep, y);
%    z_rep = -radius^2 *diff(u_rep, z);
% https://www.robotshop.com/community/forum/t/artificial-potential-field-approach-and-its-problems/11821
%    urep = 1/nu (1/distToObs - 1/obRadius)^2
   
    x_rep = -(repConst*(obs(1) - currPt(1))) * ((sqrt(obs(1) - currPt(1))^2 + (obs(2) - currPt(2))^2 + (obs(3) - currPt(3))^2) - radius);
    x_rep = x_rep / (radius*((obs(1) - currPt(1))^2 + (obs(2) - currPt(2))^2 + (obs(3) - currPt(3))^2)^2);
    y_rep = -(repConst*(obs(2) - currPt(2))) * ((sqrt(obs(1) - currPt(1))^2 + (obs(2) - currPt(2))^2 + (obs(3) - currPt(3))^2) - radius);
    y_rep = y_rep / (radius*((obs(1) - currPt(1))^2 + (obs(2) - currPt(2))^2 + (obs(3) - currPt(3))^2)^2);
    z_rep = -(repConst*(obs(3) - currPt(3))) * ((sqrt(obs(1) - currPt(1))^2 + (obs(2) - currPt(2))^2 + (obs(3) - currPt(3))^2) - radius);
    z_rep = z_rep / (radius*((obs(1) - currPt(1))^2 + (obs(2) - currPt(2))^2 + (obs(3) - currPt(3))^2)^2);
%     x_rep = radius^2 * ((2 * repConst * (currPt(1) - obs(1)))/((currPt(1) - obs(1))^2 + (currPt(2) - obs(2))^2 + (currPt(3) - obs(3))^2)^2);
%     y_rep = radius^2 * ((2 * repConst * (currPt(2) - obs(2)))/((currPt(1) - obs(1))^2 + (currPt(2) - obs(2))^2 + (currPt(3) - obs(3))^2)^2);
%     z_rep = radius^2 * ((2 * repConst * (currPt(3) - obs(3)))/((currPt(1) - obs(1))^2 + (currPt(2) - obs(2))^2 + (currPt(3) - obs(3))^2)^2);
    x = currPt(1);
    y = currPt(2);
    z = currPt(3);
   
    if currPt == obs
       potential_components = [0 0 0];
    else
       potential_components = [x_rep y_rep z_rep]; 
    end
end
