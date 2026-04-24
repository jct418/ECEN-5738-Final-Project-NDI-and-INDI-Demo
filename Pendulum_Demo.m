%% Setup
clear; clc;
ti = 0;
tf = 20;
fq = 500;
steps = (tf-ti)*fq;
tspan = linspace(ti,tf,steps);
x0 = [0;0;pi/6;0;0];

% Noise, hard-coded signal to noise ratio of 5
noise = false;

% LQR setup, based on the plant linearized about theta=0
Q = [50 0 0 0;
     0 7 0 0;
     0 0 50 0;
     0 0 0 10];

R = 5e-4;

[A_lin,B_lin] = GetLinDyn(30,20,3.4);
K = lqr(A_lin,B_lin,Q,R);

% Nominal nonlinear plant
M_k = 30;
m_k = 20;
l_k = 3.4;

gxr = @(x2,x3,x4,M,m,l,g) [
       0;
       1/(M+m*sin(x3)^2);
       0;
       -cos(x3)/(l*M+l*m*sin(x3)^2)];

fxr = @(x2,x3,x4,M,m,l,g) [
    x2;
    (1/((M/m) + sin(x3)^2)) * (x4^2*l*sin(x3) - g*sin(x3)*cos(x3));
    x4;
    (1/(l*(M/m) + l*sin(x3)^2)) * (-x4^2*l*cos(x3)*sin(x3) + (M+m)*g*sin(x3)/m)
];

% Inaccurate nonlinear plant
% Yes this is an absurd mismodeling, but the LQR does too good of a job
% stabilizing the system.
fxc = @(x2,x3,x4,M,m,l,g) [
    x2;
    (((M*m) + sin(x3*180/pi)^2)) * (x4^2*l*sin(x3*180/pi) + g*sin(x3*180/pi)*cos(x3*180/pi));
    x4;
    ((l*(M*m) + l*sin(x3*180/pi)^2)) * (-x4^2*l*cos(x3*180/pi)*sin(x3*180/pi) + (M+m)*g*sin(x3*180/pi)/m)
];

%% (Case 1) NDI - Accurate Plant
[T1,X1] = rk4(@(t,x)InvPenNDI(t,x,K,fxr,gxr,M_k,m_k,l_k,noise),tspan,x0);

PenPlot(T1,X1,fq,1);

%% (Case 2) NDI - Inaccurate Plant
[T2,X2] = rk4(@(t,x)InvPenNDI(t,x,K,fxc,gxr,M_k,m_k,l_k,noise),tspan,x0);

PenPlot(T2,X2,fq,2);

%% (Case 3) INDI - Inaccurate Plant
[T3,X3] = rk4(@(t,x)InvPenINDI(t,x,K,fxc,gxr,M_k,m_k,l_k,noise),tspan,x0);

PenPlot(T3,X3,fq,3);