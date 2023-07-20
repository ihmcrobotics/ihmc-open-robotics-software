clear;
clc;
directions = [1.000, 0.000; % 0
              0.866, 0.500; % 1
              0.500, 0.866; % 2
              0.000, 1.000; % 3
             -0.500, 0.866; % 4
             -0.866, 0.500; % 5
             -1.000, 0.000; % 6
             -0.866,-0.500; % 7
             -0.500,-0.866; % 8
              0.000,-1.000; % 9
              0.500,-0.866; % 10
              0.866,-0.500];% 11
directions(13,:) = directions(1,:);

% All of this is scaled by gravity.
% f = delta / duration * mass

% case 1 = no step adjustment, no swing time adjustment, no transfer time adjustment, no cross over
magnitude_case1 = [0.27; % 0
                   0.27; % 1
                   0.18; % 2
                   0.16; % 3
                   0.21;  % 4
                   0.25; % 5
                   0.22;  % 6
                   0.22;  % 7
                   0.14;  % 8
                   0.11;  % 9
                   0.12;  % 10
                   0.2]; % 11
magnitude_case1(13) = magnitude_case1(1);
% case 2 = with step adjustment, no swing time adjustment, no transfer time adjustment, no cross over
magnitude_case2 = [0.50; % 0
                   0.28; % 1
                   0.19; % 2
                   0.17; % 3
                   0.21;  % 4
                   0.27; % 5
                   0.4;  % 6
                   0.5;  % 7
                   0.37; % 8
                   0.35; % 9
                   0.39; % 10
                   0.68]; % 11
magnitude_case2(13) = magnitude_case2(1);

% case 3 = with step adjustment, with swing time adjustment, no transfer time adjustment, no cross over
magnitude_case3 = [0.64; % 0
                   0.31;  % 1
                   0.22;  % 2
                   0.20;  % 3
                   0.23;  % 4
                   0.31;  % 5
                   0.59;  % 6
                   0.57; % 7
                   0.59; % 8
                   0.76; % 9
                   0.71;  % 10
                   0.51];  % 11
magnitude_case3(13) = magnitude_case3(1);

% case 4 = with step adjustment, with swing time adjustment, with transfer time adjustment, no cross over
magnitude_case4 = [1.03; % 0
                   0.36; % 1
                   0.28; % 2
                   0.26; % 3
                   0.30;  % 4
                   0.41;  % 5
                   0.90;  % 6
                   0.95; % 7
                   0.73;  % 8
                   0.80;  % 9
                   0.83;  % 10
                   0.86];  % 11
magnitude_case4(13) = magnitude_case4(1);

% case 5 = with step adjustment, with swing time adjustment, with transfer time adjustment, with cross over
magnitude_case5 = [1.15; % 0
                   0.78; % 1
                   0.45; % 2
                   0.41; % 3
                   0.30; % 4
                   0.55; % 5
                   0.91;  % 6
                   0.95; % 7
                   0.84;  % 8
                   0.85; % 9
                   1.0;  % 10
                   1.07];% 11
magnitude_case5(13) = magnitude_case5(1);

size = 13;
values_case1 = zeros(size, 2);
values_case2 = zeros(size, 2);
values_case3 = zeros(size, 2);
values_case4 = zeros(size, 2);
values_case5 = zeros(size, 2);
for i = 1:size
    values_case1(i,:) = magnitude_case1(i) * directions(i,:);
    values_case2(i,:) = magnitude_case2(i) * directions(i,:);
    values_case3(i,:) = magnitude_case3(i) * directions(i,:);
    values_case4(i,:) = magnitude_case4(i) * directions(i,:);
    values_case5(i,:) = magnitude_case5(i) * directions(i,:);
end

plot(values_case1(:,1), values_case1(:,2),'o-',
  values_case2(:,1), values_case2(:,2),'o-',
  values_case3(:,1), values_case3(:,2),'o-',
  values_case4(:,1), values_case4(:,2),'o-',
  values_case5(:,1), values_case5(:,2),'o-');
