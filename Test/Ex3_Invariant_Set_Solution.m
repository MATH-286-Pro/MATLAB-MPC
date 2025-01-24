% ME-425 : Model Predictive Control
% Exercise sheet 2: Invariant sets
% 
%% Exercise 1, Solution
%
% 代码说明：
%   解决 x+ = Ax 这种 invariant set，没有输入控制 u
% 
% 代码使用方法：在终端按下回车进行一次运算

clc; close all; clear;

% 系统设置
alpha = pi/6; 
beta  = 0.8;
A = [cos(alpha) sin(alpha); 
    -sin(alpha) cos(alpha)]*beta;
Hx = [cos(pi/3) sin(pi/3); -cos(pi/3) -sin(pi/3);...
	sin(pi/3) -cos(pi/3); -sin(pi/3) cos(pi/3)];
hx = [2;1;2;5];

% 绘制约束
figure(1); clf; axis equal;
h1=plot(polytope(Hx,hx), 'b');
hold on;

% 计算 maximal invariant set
i = 1;
O = polytope(Hx,hx);
while 1
	Oprev = O;
	[F,f] = double(O);	
	% Compute the pre-set
	O = polytope([F;F*A],[f;f]);
	if O == Oprev, break; end
	
	h2=plot(O, 'y');
	fprintf('Iteration %i... not yet equal\n', i)
	% pause

	i = i + 1;
end
fprintf('Maximal invariant set computed after %i iterations\n\n', i);
h3=plot(O,'g');
legend([h3;h1;h2],{'Invariant set';'State constraints';'Iterations'});

%% Plot some trajectories
%  绘制轨迹

[F,f] = double(O);
cl_sys = ss(A,[0;0],eye(2),[0;0],1);
steps = 50;
set(gcf,'renderer','opengl');
while 1
	fprintf('\nClick on figure to plot trajectory (right click to exit)\n')
	[x,y,button] = ginput(1);
	if button > 1, break; end

	x = [x;y];

	if all(Hx*x <= hx) % x is in X
		traj = lsim(cl_sys,zeros(1,steps),[0:steps-1],x)';
		hdl = plot(traj(1,:),traj(2,:),'--k');
		if all(F*x <= f) % x is in Oinf
			set(hdl,'color','k')
			plot(x(1),x(2),'.k','markersize',5);
			fprintf('Entire trajectory is feasible\n');
		else
			set(hdl,'color','r')
			plot(x(1),x(2),'xr','markersize',5);
			fprintf('Some part of the trajectory is infeasible\n');
		end
	else
		fprintf('Selected point is not in the state constraints\n');
	end

end


%% Exercise 2, Solution
%
% 代码说明：
%   解决 x+ = Ax + Bu Control Invariant Set
%
% Define the system

alpha = pi/6; beta = 0.8;
A = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)]*beta;
B = [0.5;0.5];

Hx = [cos(pi/3) sin(pi/3); -cos(pi/3) -sin(pi/3);...
	sin(pi/3) -cos(pi/3); -sin(pi/3) cos(pi/3)];
hx = [2;1;2;5];

Hu = [1;-1]; hu = [0.5;0.5];


X = polytope(Hx,hx);
U = polytope(Hu,hu);


% Plot the constraints
figure(1); clf;
h1=plot(polytope(Hx,hx), 'b');
hold on;

% Compute the maximal invariant set
i = 1;
O = polytope(Hx,hx);
while 1
	Oprev = O;
	[F,f] = double(O);	
	% Compute the pre-set
	O = intersect(O, projection(polytope([F*A F*B; zeros(2) Hu],[f;hu]), 1:2));
	if O == Oprev, break; end
	
	h2=plot(O, 'y');
	fprintf('Iteration %i... not yet equal\n', i)
	% pause

	i = i + 1;
end
fprintf('Maximal control invariant set computed after %i iterations\n\n', i);
h3=plot(O,'g');
legend([h3;h1;h2],{'Control Invariant set';'State constraints';'Iterations'});

%% Task 2 LQR
figure(2); clf;
h1=plot(polytope(Hx,hx), 'b');
hold on;

% Compute LQR controller
K = dlqr(A,B,eye(2),1);
K = -K; % Note that matlab defines K as -K
Ak = A+B*K; % Closed-loop dynamics

HH = [Hx;Hu*K]; hh = [hx;hu]; % State and input constraints

h4=plot(polytope(HH,hh), 'c');
hold on;

% Compute the maximal invariant set
i = 1;
O = polytope(HH,hh);
while 1
	Oprev = O;
	[F,f] = double(O);	
	% Compute the pre-set
	O = polytope([F;F*Ak],[f;f]);
	if O == Oprev, break; end
	
	h2=plot(O, 'y');
	fprintf('Iteration %i... not yet equal\n', i)
	% pause

	i = i + 1;
end
fprintf('Maximal invariant set computed after %i iterations\n\n', i);
h3=plot(O,'g');
legend([h3;h1;h2;h4],{'Invariant set';'State constraints';'Iterations';'State and input constraints'});

%% Plot some trajectories
%
% 代码说明：这里只是通过 Ax + Bu 进行迭代计算轨迹，本质上就是 LQR 的轨迹，不是 MPC 的轨迹！！！！

[F,f] = double(O);                                  % invariant set 的边界
cl_sys = ss(Ak,[0;0],eye(2),[0;0],1);
steps = 50;
set(gcf,'renderer','opengl');

while 1
	fprintf('\nClick on figure to plot trajectory (right click to exit)\n')
	[x,y,button] = ginput(1);                      % 获取鼠标点击位置
	if button > 1, break; end

	x = [x;y];

	if all(HH*x <= hh) % x is in X
		traj = lsim(cl_sys,zeros(1,steps),[0:steps-1],x)';
		hdl = plot(traj(1,:),traj(2,:),'--k');
		if all(F*x <= f) % x is in Oinf            % 判断是否进入 invariant set
			set(hdl,'color','k')
			plot(x(1),x(2),'.k','markersize',5);
			fprintf('Entire trajectory is feasible\n');
		else
			set(hdl,'color','r')
			plot(x(1),x(2),'xr','markersize',5);
			fprintf('Some part of the trajectory is infeasible\n');
		end
	else
		fprintf('Selected point is not in the state constraints\n');
	end

end