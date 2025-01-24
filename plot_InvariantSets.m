% function [Xf] = plot_InvariantSets(A, B, Q, R, X, U, PLOT)
% % plotInvariantSets(A, B, Q, R, X, U)
% %
% % 功能：
% %   1. 给定离散系统 x+ = A x + B u； 2. 给定 LQR 权重 Q, R，用于计算反馈增益 K =
% %   -dlqr(A,B,Q,R)； 3. 给定状态可行域 X 和输入可行域 U (polytope 对象)； 4.
% %   分两种情况计算不变集合并可视化：
% %       (a) 不显式使用 LQR，只要求对每个状态都存在满足输入约束的控制量； (b) 使用 LQR
% %       反馈律，联合状态和输入约束来计算不变集合。
% %
% % 输入：
% %   A, B  : 系统矩阵 (离散) Q, R  : LQR 设计的代价函数权重 X     : 状态约束多面体 (polytope) ，即 X
% %   = { x | Hx x <= hx } U     : 输入约束多面体 (polytope) ，即 U = { u | Hu u <= hu
% %   }
% %
% % 依赖：
% %   - 需要在 MATLAB 环境中安装 MPT3 (或兼容库) 以使用 polytope 等函数。 -
% %   默认：dlqr、plot、double、projection 为 MATLAB 自带或 Toolbox 函数。
% %
% % 作者：胡嘉骏 日期：2025-01-03
% 
% % 数据类型转换
% X = polytope(X.A,X.b); % Polyhedron 类型转换为 polytope
% U = polytope(U.A,U.b); % Polyhedron 类型转换为 polytope
% 
% 
% % 计算闭环系统
% K = -dlqr(A,B,Q,R);
% M = A + B * K;
% 
% % 创建 u = Kx 导致的第一次约束收紧
% [Ax,bx] = double(X);
% [Au,bu] = double(U);
% 
% % U 的约束反作用于 X
% HH = [Ax;Au*K];
% hh = [bx;bu];
% T = polytope(HH,hh);
% 
% 
% % 初始画图
% if PLOT == 1
%     figure; hold on; grid on; %axis equal;
%     X.plot('color','g','alpha',0.4);
%     T.plot('color','b','alpha',1);
% end
% 
% % 开始迭代
% i = 1;
% O = T;
% maxIter = 20;
% while i <= maxIter
%     O_pre = O;
%     [F,f] = double(O);
%     O = polytope([F;F*M],[f;f]); 
%     if O == O_pre, fprintf('Complete'), break; end
%     if PLOT == 1, O.plot('color','y'); end
%     fprintf('Iteration %i\n',i)
%     i = i + 1;
% end
% 
% 
% fprintf('Maximal control invariant set computed after %i iterations\n\n', i);
% 
% if PLOT == 1
%     O.plot('color','c','alpha',1);
%     hold off
% end
% 
% % 数据类型转换
% % 将 polytope 转化为 Polyhedron
% [A_Xf,b_Xf] = double(O);
% Xf = Polyhedron(A_Xf,b_Xf);
% 
% end


function [Xf] = plot_InvariantSets(A, B, Q, R, X, U, PLOT)
    sys = LTISystem('A',A,'B',B);
    
    X_range = X.V;
    U_range = U.V;

    sys.x.min = [X_range(1);-inf]; sys.x.max = [+inf; +inf];
    sys.u.min = [U_range(2)];       sys.u.max = [U_range(1)];
    sys.x.penalty = QuadFunction(Q);
    sys.u.penalty = QuadFunction(R);

    Xf = sys.LQRSet;

    if PLOT == 1, Xf.plot('color','c','alpha',0.5); end
