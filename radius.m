function [radius, position] = radius(p);
for i = 1:4             % find m11
    a(i,1) = p(i,1);
    a(i,2) = p(i,2);
    a(i,3) = p(i,3);
    a(i,4) = 1;
end
m11 = det(a); 

for i = 1:4             % find m12
    a(i,1) = p(i,1)*p(i,1) + p(i,2)*p(i,2) + p(i,3)*p(i,3);
    a(i,2) = p(i,2);
    a(i,3) = p(i,3);
    a(i,4) = 1;
end
m12 = det(a);

for i = 1:4             % find m13
    a(i,1) = p(i,1)*p(i,1) + p(i,2)*p(i,2) + p(i,3)*p(i,3);
    a(i,2) = p(i,1);
    a(i,3) = p(i,3);
    a(i,4) = 1;
end
m13 = det(a);

for i = 1:4             % find m14
    a(i,1) = p(i,1)*p(i,1) + p(i,2)*p(i,2) + p(i,3)*p(i,3);
    a(i,2) = p(i,1);
    a(i,3) = p(i,2);
    a(i,4) = 1;
end
m14 = det(a);

for i = 1:4             % find m15
    a(i,1) = p(i,1)*p(i,1) + p(i,2)*p(i,2) + p(i,3)*p(i,3);
    a(i,2) = p(i,1);
    a(i,3) = p(i,2);
    a(i,4) = p(i,3);
end
m15 = det(a);

if m11 == 0             % calculate the centre and radius of the sphere
    radius = 0;
else
    position(1) =  0.5 * m12 / m11;
    position(2) = -0.5 * m13 / m11;
    position(3) =  0.5 * m14 / m11;
    radius = sqrt(power(position(1),2)+power(position(2),2) + power(position(3),2) - m15 / m11);
end
end

