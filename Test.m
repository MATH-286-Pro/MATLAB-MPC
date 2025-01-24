%% Exercise 1, Solution
%
% 代码说明：
%   解决 x+ = Ax 这种 invariant set，没有输入控制 u

clc; close all; clear;

% 系统设置
alpha = pi/6; 
beta  = 0.8;
A = [cos(alpha) sin(alpha); 
    -sin(alpha) cos(alpha)]*beta;
Ax = [cos(pi/3) sin(pi/3); -cos(pi/3) -sin(pi/3);...
	sin(pi/3) -cos(pi/3); -sin(pi/3) cos(pi/3)];
bx = [2;1;2;5];

X = polytope(Ax,bx);

% 绘制约束
figure(1); clf; axis equal; hold on;
h1=plot(X, 'b');


% 计算 maximal invariant set
i = 1;
O = X;
while i <= 50
	Oprev = O;
	[F,f] = double(O);	
	% Compute the pre-set
	O = polytope([F;F*A],[f;f]);
    test = polytope(F*A,f); % 添加测试
    plot(test,'c')
	if O == Oprev, break; end
	
	h2=plot(O, 'y');
	fprintf('Iteration %i... not yet equal\n', i)
	% pause

	i = i + 1;
end
% fprintf('Maximal invariant set computed after %i iterations\n\n', i);
h3=plot(O,'g');
legend([h3;h1;h2],{'Invariant set';'State constraints';'Iterations'});