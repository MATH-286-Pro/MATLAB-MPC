function [Xf] = plot_ControlInvariantSet(A,B,X,U)

% 1) 从约束构建多面体
% X, U 已经加工完成
  
% Step 3: Compute control invariant set iteratively
figure;
hold on;
axis equal;
xlabel('x_1'); ylabel('x_2');
title('Control Invariant Set with State Constraints');
grid on;

% Plot the state constraint set in green
X.plot('color', 'green', 'alpha', 0.1);

% Initialize sets for iterative computation
V_old = X; % Start with state constraint set
iteration = 1;

% Set up color gradient (green to red)
numIterations = 8; % Estimated max iterations
% colors = [linspace(0, 1, numIterations)', linspace(1, 0, numIterations)', zeros(numIterations, 1)];
colors = [zeros(numIterations, 1), linspace(1, 0, numIterations)', linspace(0, 1, numIterations)'];

while true
    % Compute pre-set
    pre_V = preSet(A, B, V_old, U); % Custom function for pre-set calculation
    
    % Intersect with state constraints
    V_new = V_old & pre_V;
    
    % Plot the intermediate invariant set with gradient color
    colorIdx = min(iteration, numIterations); % Prevent exceeding color range
    V_new.plot('alpha', 0.5, 'color', colors(colorIdx, :));
    drawnow;
    
    % Check for convergence
    if V_new == V_old
        disp(['Converged after ' num2str(iteration) ' iterations.']);
        break;
    end
    
    % Update for next iteration
    V_old = V_new;
    iteration = iteration + 1;
end

% Final invariant set
V_new.plot('color', 'c');
grid on;



% 输出终止集
Xf = V_new;


% Custom pre-set computation function
function pre_V = preSet(A, B, X, U)
    % Ensure dimensions are consistent
    nx = size(A, 1); % State dimension
    nu = size(B, 2); % Input dimension

    % Compute pre-set
    P = Polyhedron('A', [X.A * A, X.A * B; ...
                         zeros(size(U.A, 1), nx), U.A], ...
                   'b', [X.b; U.b]);
    
    % Project onto state space
    pre_V = projection(P, 1:nx);
end
end