

clc; clear; close; close all;

% Alkim GOKCEN, 04.09.2022, Universit of IZMIR KATIP CELEBI
% alkim.gokcen@outlook.com

% This mfile is to solve a contunious cost (performance) functional J(V)
% to find an optimal controller signal according to given plant!

% Plant & controller are one dimensional cont. systems. (Will be
% generalized for n-dimensional plant and m-dimensional plants where m<n



% setup of initials
initial_State_Value = 1; % x(0) = 1;
final_Time = 1;          % final_T = 1; (Time span end point)
final_State_Value = 0;   % x(final) = 0

% define the plant xdot = -5x + u
A_plant = -5;
B_plant = 1;
syms t x(t) u(t);

% initial condition
initCond = x(0) == initial_State_Value;
finalCond = x(final_Time) == final_State_Value;

% def. of xdot
xdot = diff(x(t), t);

% define the function of V ( V from functional J(V) )
J = x * x + 2 * u * u; % there is an integral from 0 to FinalTime
V = (x * x + (2 * (xdot + 5 * x)^2));

% find Euler_Lagrange equation derivatives
dvdx = diff(V, x);
dvdxdot = diff(V, xdot);
dvdxdot_dt = diff(V, xdot, t);

% define Euler_Lagrange equality, EL = 0
EL_eq = dvdx - dvdxdot_dt;

% solve EL diff eq. to find optimal state
x_optimal = dsolve(EL_eq,initCond,finalCond);

% find optimal controller signal
u_optimal = (diff(x_optimal, t) + (-A_plant) * x_optimal) / B_plant;
u_optimal_fun_of_t = matlabFunction(u_optimal);















