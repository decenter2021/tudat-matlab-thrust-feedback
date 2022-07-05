%% Package: tudat-matlab-thrust-feedback
% Author: Leonardo Pedroso
%% Feedback loop implementation
% Implement the feedback-loop in this script 
% Input:   t  
%          x_t dim: 7N x 1
% Output:  u_t dim: 3 x N
% The controller parameters should be defined in matlab-app.m

%% Dummy feedback
% Thrust along velocity after t = 3000 s
u_t = zeros(3*N,1);
for i = 1:N
	if t>3e3
		u_t(3*(i-1)+1) = thrust_mag;
	end
end

