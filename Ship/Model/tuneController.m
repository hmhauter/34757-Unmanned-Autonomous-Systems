%% Estimate transfer function for base system using LINEARIZE
% Motor volatge to wheel velocity (wv)
model = 'ship_pid_SIMPLE';
load_system(model);
open_system(model);
% define points in model
ios(1) = linio(strcat(model,'/in1'),1,'openinput');
ios(2) = linio(strcat(model, '/out1'),1,'openoutput');
% attach to model
setlinio(model,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys = linearize(model,ios,op);
% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gf = minreal(tf(num, den))
pole(Gf)
zero(Gf)

%% Step and Impulse Response
subplot(2,1,1)
step(Gf)
subplot(2,1,2)
impulse(Gf)
%% plots
figure(1)
bode(Gf)
grid on
title('Transfer function Bode')




