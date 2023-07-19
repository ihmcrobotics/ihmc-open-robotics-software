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
% case 3 = with step adjustment, with swing time adjustment, no transfer time adjustment, no cross over
magnitude_case3 = [0.64; % 0
                   0.31;  % 1
                   0.22;  % 2
                   0.20;  % 3
                   0.23;  % 4
                   0.31;  % 5
                   0.59;  % 6
                   0.55; % 7 stopped here (didn't do this one)
                   0.55; % 8
                   0.75; % 9
                   0.7;  % 10
                   0.5];  % 11

% case 4 = with step adjustment, with swing time adjustment, with transfer time adjustment, no cross over
magnitude_case4 = [1.00; % 0
                   0.35; % 1
                   0.25; % 2
                   0.25; % 3
                   0.3;  % 4
                   0.4;  % 5
                   0.9;  % 6
                   0.95; % 7
                   0.7;  % 8
                   0.8;  % 9
                   0.8;  % 10
                   0.8];  % 11

% case 5 = with step adjustment, with swing time adjustment, with transfer time adjustment, with cross over
magnitude_case5 = [1.10; % 0
                   0.75; % 1
                   0.50; % 2
                   0.40; % 3
                   0.30; % 4
                   0.55; % 5
                   0.9;  % 6
                   0.95; % 7
                   0.8;  % 8
                   0.85; % 9
                   1.0;  % 10
                   1.05];% 11

values_case1 = zeros(12, 2);
values_case2 = zeros(12, 2);
values_case3 = zeros(12, 2);
values_case4 = zeros(12, 2);
values_case5 = zeros(12, 2);
for i = 1:12
    values_case1(i,:) = magnitude_case1(i) * directions(i,:);
    values_case2(i,:) = magnitude_case2(i) * directions(i,:);
    values_case3(i,:) = magnitude_case3(i) * directions(i,:);
    values_case4(i,:) = magnitude_case4(i) * directions(i,:);
    values_case5(i,:) = magnitude_case5(i) * directions(i,:);
end

plot(values_case1(:,1), values_case1(:,2),
  values_case2(:,1), values_case2(:,2),
  values_case3(:,1), values_case3(:,2),
  values_case4(:,1), values_case4(:,2),
  values_case5(:,1), values_case5(:,2));
