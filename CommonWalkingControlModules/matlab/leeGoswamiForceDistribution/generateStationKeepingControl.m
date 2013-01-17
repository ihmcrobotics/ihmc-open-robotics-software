function control = generateStationKeepingControl(wk,epsilonf)
control.ld = [0.0; 0.0; 0.0];
control.kd = [0.0; 0.0; 0.0];
control.wk = wk;
control.epsilonf = epsilonf;
end