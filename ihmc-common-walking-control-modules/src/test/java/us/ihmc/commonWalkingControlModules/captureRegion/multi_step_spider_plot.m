directions = [1.000, 0.000; // 0
              0.866, 0.500; // 1
              0.500, 0.866; // 2
              0.000, 1.000; // 3
             -0.500, 0.866; // 4
             -0.866, 0.500; // 5
             -1.000, 0.000; // 6
             -0.866,-0.500; // 7
             -0.500,-0.866; // 8
              0.000,-1.000; // 9
              0.500,-0.866; // 10
              0.866,-0.500];// 11

// All of this is scaled by gravity.
// f = delta / duration * mass

// case 1 = no step adjustment, no swing time adjustment, no transfer time adjustment, no cross over
magnitude_case1 = [0.25; // 0
                   0.25; // 1
                   0.15; // 2
                   0.15; // 3
                   0.2;  // 4
                   0.25; // 5
                   0.2;  // 6
                   0.2;  // 7
                   0.1;  // 8
                   0.1;  // 9
                   0.1;  // 10
                   0.2]; // 11
// case 2 = with step adjustment, no swing time adjustment, no transfer time adjustment, no cross over
magnitude_case2 = [0.50; // 0
                   0.25; // 1
                   0.15; // 2
                   0.15; // 3
                   0.2;  // 4
                   0.25; // 5
                   0.4;  // 6
                   0.5;  // 7
                   0.35; // 8
                   0.35; // 9
                   0.35; // 10
                   0.7]; // 11
// case 3 = with step adjustment, with swing time adjustment, no transfer time adjustment, no cross over TODO regenerate this, and set "icpDistanceOutsideSupportForStep" to infinity
magnitude_case3 = [0.60; // 0
                   0.3;  // 1
                   0.2;  // 2
                   0.2;  // 3
                   0.2;  // 4
                   0.3;  // 5
                   0.6; // 6
                   0.55;  // 7
                   0.55;  // 8
                   0.75;  // 9
                   0.7; // 10
                   0.5; // 11
                   ];
// case 4 = with step adjustment, with swing time adjustment, with transfer time adjustment, no cross over
magnitude_case4 = [1.00; // 0
                   0.35; // 1
                   0.25; // 2
                   0.25; // 3
                   0.3;  // 4
                   0.4;  // 5
                   0.9;  // 6
                   0.95; // 7
                   0.7;  // 8
                   0.8;  // 9
                   0.8;  // 10
                   0.8;  // 11
                   ];
// case 5 = with step adjustment, with swing time adjustment, with transfer time adjustment, with cross over
magnitude_case5 = [1.10; // 0
                   0.75; // 1
                   0.50; // 2
                   0.40; // 3
                   0.30; // 4
                   0.55; // 5
                   0.9;  // 6
                   0.95; // 7
                   0.8;  // 8
                   0.85; // 9
                   1.0;  // 10
                   1.05];// 11
