function control = generateStandardControl()
control.ld = [0.1; -0.1; 0.1];
control.kd = [0.0; 0.0; 0.0];
control.wk = 0.5;
control.epsilonf = 0.001;
end