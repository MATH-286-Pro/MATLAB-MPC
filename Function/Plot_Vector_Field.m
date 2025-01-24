function [] = Plot_Vector_Field(A, X_range, Y_range)
% 绘制矢量场

% 在指定范围内生成网格点
[x, y] = meshgrid(linspace(X_range(1), X_range(2), 20), linspace(Y_range(1), Y_range(2), 20));

% 计算 x' = A * x
% 将 (x, y) 视为向量 [x; y]，然后应用 A
I = eye(size(A));
xy = [x(:)'; 
      y(:)'];
xy_dot = (A-I) * xy;

% 将结果拆回与网格对应的形状，用于绘制矢量场
u = reshape(xy_dot(1, :), size(x));
v = reshape(xy_dot(2, :), size(y));

% 绘制矢量场
% X.plot('color','b','alpha',1.0);
% O.plot('color','g','alpha',1.0);
quiver(x, y, u, v, 'LineWidth', 1.0, 'Color', 'k');
xlabel('x_1');
ylabel('x_2');

end