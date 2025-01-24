function [X_tilde, U_tilde] = plot_Tighten(X, U, E, K, PLOT)
%TIGHTEN_BOXAPPROACH_POLYHEDRON 
%
% 基于“最大偏移量”的盒收紧方法。
%
% 输入:
%   X:          2D Polyhedron, 描述状态约束
%   U:          1D Polyhedron, 描述输入约束
%   EdataFile:  .mat 文件路径, 内含:
%                   E: Polyhedron (2D), 关于原点对称的 epsilon-set
%                   K: 1x2 double, 用于计算 u=Kx
%
% 输出:
%   X_tilde:    收紧后的 2D Polyhedron
%   uTildeRange:[u_min, u_max], 收紧后的 1D 输入范围

  
    %% 2. 获取 X 的 bounding box
    %   outerApprox() => 返回一个外包矩形 (hyper-rectangle)
    X_box = X.outerApprox();   % => bounding box for X
    X_lb  = X_box.Internal.lb; % 2x1
    X_ub  = X_box.Internal.ub; % 2x1

    % % 小检查：X 是否真的是2D
    % if length(X_lb) ~= 2 || length(X_ub) ~= 2
    %     error('当前示例仅针对2D状态，检测到X维度不为2');
    % end
    
    %% 3. 计算 E 在各维度上的最大偏移 e_absMax
    E_box = E.outerApprox();   
    e_min = E_box.Internal.lb; % 2x1
    e_max = E_box.Internal.ub; % 2x1

    % 用最大偏移量收紧 X 的 bounding box
    x_min_tilde = X_lb - e_min; % 每维都往里加
    x_max_tilde = X_ub - e_max;
    
    % 构造新的 2D 多面体 X_tilde
    X_tilde = Polyhedron('lb', x_min_tilde, ...
                         'ub', x_max_tilde);

    %% 4. 收紧输入 U
    % 4.1 先映射 E 到输入维度 => KE
    KE = E.affineMap(K, []);   % 1 x 2 => 1D 多面体
    KE_box = KE.outerApprox(); 
    ke_min = KE_box.Internal.lb; % 标量
    ke_max = KE_box.Internal.ub; % 标量
    
    % 4.2 U 本身的 bounding box
    U_box = U.outerApprox();
    U_lb  = U_box.Internal.lb;  % 标量
    U_ub  = U_box.Internal.ub;  % 标量
    
    % 收紧 U
    u_min_tilde = U_lb - ke_min;
    u_max_tilde = U_ub - ke_max;
    U_tilde = Polyhedron('lb', u_min_tilde, ...
                         'ub', u_max_tilde);
    
    
    %% 5. (可选)绘图对比
    if PLOT == 1
        figure('Name','X_{tilde}','NumberTitle','off');
        hold on; grid on; axis equal;
        title('X_{tilde}：基于最大偏移量收紧后的 X(2D)');
    
        % 画原始 X
        % X.plot('wire',true,'linestyle','--','color','k');
        X.plot('color','g','alpha',0.4);
    
        % 画收紧后的 X_tilde
        X_tilde.plot('color','green','alpha',0.9);
        plot(X_tilde, 'color','green'); 
    
        legend('X','X_{Tighten}','Location','best');
        xlabel('x_1'); ylabel('x_2');
    end
    %% 6. 打印提示
    fprintf('X_tilde:\n');
    fprintf([' x1 in [%.2f, %.2f]\n' ...
             ' x2 in [%.2f, %.2f]\n'], ...
        x_min_tilde(1), x_max_tilde(1), x_min_tilde(2), x_max_tilde(2));
    fprintf(['U_tilde:\n' ...
             ' [%.2f, %.2f]\n'], u_min_tilde, u_max_tilde);

end
