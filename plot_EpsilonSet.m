function [E, K] = plot_EpsilonSet(A, B, Q, R, W, maxIter, PLOT)
% computeEpsilonSet_evolution 计算并绘制给定系统的 epsilon-set，
% 使用指定的终止条件以及迭代上限。
%
% 详细信息与原代码一致。

    
    % 使用 LQR 计算控制器
    K = -dlqr(A, B, Q, R);

    % 计算 M = A + B*K
    M = A + B * K;
    % M = A - B * K;

    % 初始化 epsilon-set, 用单点 {0} 开始
    E = Polyhedron('V', [0 0]);  % 只包含原点

    % M^i 用于每次更新
    Mi = eye(size(M));

    % -- 准备绘图 --
    if PLOT == 1
        figure('Name', 'Epsilon-set Evolution', 'NumberTitle', 'off');
        hold on; grid on;
        axis equal;
        xlabel('x_1'); ylabel('x_2');
        title('\epsilon Set');
        
        % 绘制干扰集合 W (黄色)
        W.plot('color', 'yellow', 'alpha', 0.3);
    end
    
    % 初始化迭代变量
    i = 0;

    % 使用 while 循环并根据终止条件退出
    while true
        if i == 0
            Wi = W;      % M^0 * W = W
        else
            % 计算 M^i * W
            Mi = Mi * M; 
            Wi = W.affineMap(Mi, []);
        end

        E_new = E + Wi;       % 闵斯基和
        E     = E_new;        % 更新 epsilon-set

        % 绘制本次 E
        if PLOT == 1, E.plot('color', 'b', 'alpha', 0.1); end
        E.minVRep();
        E.minHRep();

        % 终止条件：norm(M^i) < 1e-2 或迭代次数达到上限
        if norm(Mi, 'fro') < 1e-2
            disp(['在第 ', num2str(i), ' 次迭代时收敛。']);
            break;
        elseif i >= maxIter
            disp('迭代达到 maxIter 限制');
            break;
        end

        i = i + 1;  % 更新迭代计数
    end
    
end





