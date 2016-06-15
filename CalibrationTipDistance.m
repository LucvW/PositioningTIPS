%Script to test the determination distance between sensor and tip
%31-5-2016 C.Treffers TU Delft

%% test with 10 point
clear all;
close all;
samples = 14;
position(1,1) = 0;          % point 1 (0,0,sqrt(3))
position(1,2) = 0;
position(1,3) = sqrt(3);

position(2,1) = sqrt(3);    % point 2 (sqrt(3),0,0)
position(2,2) = 0;
position(2,3) = 0;

position(3,1) = 0;          % point 3 (0,sqrt(3),0)
position(3,2) = sqrt(3);
position(3,3) = 0;

position(4,1) = 1;          % point 4 (1,1,1)
position(4,2) = 1;
position(4,3) = 1;

position(5,1) = -1;         % point 5 (-1,-1,-1)
position(5,2) = -1;
position(5,3) = -1;

position(6,1) =  1;         % point 6 (1,1,-1)
position(6,2) =  1;
position(6,3) = -1;

position(7,1) = 1;          % point 7 (1,-1,1)
position(7,2) = -1;
position(7,3) = 1;

position(8,1) = -1;         % point 8 (-1,1,1)
position(8,2) = 1;
position(8,3) = 1;

position(9,1) = -1;         % point 9 (-1,-1,1)
position(9,2) = -1;
position(9,3) =  1;

position(10,1) =  1;        % point 10 (1,-1,-1)
position(10,2) = -1;
position(10,3) = -1;

position(11,1) = -1;        % point 11 (-1,1,-1)
position(11,2) =  1;
position(11,3) = -1;

position(12,1) = 0;         % point 12 (0,0,-sqrt(3))
position(12,2) = 0;
position(12,3) = -sqrt(3);

position(13,1) = -sqrt(3);  % point 13 (-sqrt(3),0,0)
position(13,2) = 0;
position(13,3) = 0;

position(14,1) = 0;         % point 14 (0,-sqrt(3),0)
position(14,2) = -sqrt(3);
position(14,3) = 0;


%% Calculate different radius and positions
pos(1) = 0;
pos(2) = 0;
pos(3) = 0;
i = 1;
while i < samples - 3  %take different combinations op points
    j = i;
    while j < i + 4
        k = 1;
        while k < 4
            l = j - i + 1;
            p(l,k) = position(j,k);
            k = k + 1;
        end
        j = j + 1;
    end
    [r, pos] = radius(p);   %calculate the radius and centre of the sphere
    rad(i,1) = r;
    positiontip(i,1) = pos(1);
    positiontip(i,2) = pos(2);
    positiontip(i,3) = pos(3);
    i = i + 1;
end

figure(1)
i = 1;

while (i < samples + 1) % plot results
    line([position(i,1) positiontip(1,1)], [position(i,2) positiontip(1,2)],[position(i,3) positiontip(1,3)]);
    hold on
    grid on
    scatter3(position(i,1), position(i,2), position(i,3), 'blue')
    hold on
    i = i + 1;
end

scatter3(positiontip(1,1), positiontip(1,2), positiontip(1,3), 'red')
xlabel('x')
ylabel('y')
zlabel('z')

FinalRadius = mean(rad(1,:))    %calculate the average radius 



