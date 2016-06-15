%% Main function to determinatie the orientation between the sensor and 
% the tip
% Charlotte Treffers June 2016

%% test with 10 points
clear all;
close all;
samples = 8;
position(1,1) = 0; % point 1 (0,1,0)
position(1,2) = 1;
position(1,3) = 0;

position(2,1) = 0; % point 2 (0,0,1)
position(2,2) = 0;
position(2,3) = 1;

position(3,1) = 0; % point 3 (0,-1,0)
position(3,2) = -1;
position(3,3) = 0;

position(4,1) = 0; % point 4 (0,0,-1)
position(4,2) = 0;
position(4,3) = -1;

position(5,1) = 0; % point 5 (0, sqrt(0.5), sqrt(0.5))
position(5,2) = sqrt(0.5);
position(5,3) = sqrt(0.5);

position(6,1) = 0; % point 6 (0, -sqrt(0.5), sqrt(0.5))
position(6,2) = -sqrt(0.5);
position(6,3) = sqrt(0.5);

position(7,1) = 0; % point 7 (0, -sqrt(0.5), -sqrt(0.5))
position(7,2) = -sqrt(0.5);
position(7,3) = -sqrt(0.5);

position(8,1) = 0; % point 8 (0, sqrt(0.5), -sqrt(0.5))
position(8,2) = sqrt(0.5);
position(8,3) = -sqrt(0.5);

%% Other used parameter
lengthDistance = 5;%length of distance vector between MPU9250 and probe-tip

%% Calculate different radius and positions
pos(1) = 0;
pos(2) = 0;
pos(3) = 0;
i = 1;
while i < samples - 1
    j = i;
    while j < i + 3
        k = 1;
        while k < 4
            l = j - i + 1;
            p(l,k) = position(j,k);
            k = k + 1;
        end
        j = j + 1;
    end
    [center, radius, v1n, v2n] = circlefit3d(p(1,:),p(2,:), p(3,:));
    rad(i,1) = radius;
    centercirkel(i,1) = center(1);
    centercirkel(i,2) = center(2);
    centercirkel(i,3) = center(3);
    i = i + 1;
end

finalRadius = mean(rad(1,:));
finalCenter(1) = mean(centercirkel(:,1));
finalCenter(2) = mean(centercirkel(:,2));
finalCenter(3) = mean(centercirkel(:,3));
normal(1) = v1n(2)*v2n(3) - v1n(3)*v2n(2);
normal(2) = v1n(3)*v2n(1) - v1n(1)*v2n(3);
normal(3) = v1n(1)*v2n(2) - v1n(2)*v2n(1);

lengthNormal = sqrt(power(lengthDistance,2) - power(finalRadius,2));
probeTip(1) = finalCenter(1) - lengthNormal*normal(1);
probeTip(2) = finalCenter(2) - lengthNormal*normal(2);
probeTip(3) = finalCenter(3) - lengthNormal*normal(3);
offsetVector(1) = position(8,1) - probeTip(1);
offsetVector(2) = position(8,2) - probeTip(2);
offsetVector(3) = position(8,3) - probeTip(3);

%% plot results
figure(1)
i = 1;

while (i < samples + 1) % plot results
    line([position(i,1) finalCenter(1)], [position(i,2) finalCenter(2)], [position(i,3) finalCenter(3)]);
    hold on
    grid on
    scatter3(position(i,1), position(i,2), position(i,3), 'blue')
    hold on
    i = i + 1;
end

scatter3(finalCenter(1), finalCenter(2), finalCenter(3), 'green')
scatter3(probeTip(1), probeTip(2), probeTip(3), 'red')
line([probeTip(1) finalCenter(1)], [probeTip(2) finalCenter(2)], [probeTip(3) finalCenter(3)], 'color', 'red');
line([position(8,1) probeTip(1)], [position(8,2) probeTip(2)], [position(8,3) probeTip(3)], 'color', 'green');
line([position(8,1) finalCenter(1)], [position(8,2) finalCenter(2)], [position(8,3) finalCenter(3)], 'color', 'magenta');
xlabel('x')
ylabel('y')
zlabel('z')
