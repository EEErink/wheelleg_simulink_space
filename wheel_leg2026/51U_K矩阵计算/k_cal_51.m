%%%%%%%%%%%%%%%%%%%%%% 求出用左右腿腿长表达的AB矩阵并拟合K系数（行优先修正版）%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
clear; clc;

%% Step 1: 符号定义与方程求解（一次性计算）
syms R_w R_l l_l l_r l_wl l_wr l_bl l_br l_c m_w m_l m_b I_w I_ll I_lr I_b I_z g
syms theta_wl theta_wr dtheta_wl dtheta_wr ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b
syms theta_ll theta_lr theta_b T_wl T_wr T_bl T_br

% 定义方程组 (3.11)-(3.15)
eqn1 = (I_w*l_l/R_w + m_w*R_w*l_l + m_l*R_w*l_bl)*ddtheta_wl + ...
       (m_l*l_wl*l_bl - I_ll)*ddtheta_ll + ...
       (m_l*l_wl + m_b*l_l/2)*g*theta_ll + T_bl - T_wl*(1 + l_l/R_w) == 0;
eqn2 = (I_w*l_r/R_w + m_w*R_w*l_r + m_l*R_w*l_br)*ddtheta_wr + ...
       (m_l*l_wr*l_br - I_lr)*ddtheta_lr + ...
       (m_l*l_wr + m_b*l_r/2)*g*theta_lr + T_br - T_wr*(1 + l_r/R_w) == 0;
eqn3 = -(m_w*R_w^2 + I_w + m_l*R_w^2 + m_b*R_w^2/2)*(ddtheta_wl + ddtheta_wr) - ...
       (m_l*R_w*l_wl + m_b*R_w*l_l/2)*ddtheta_ll - ...
       (m_l*R_w*l_wr + m_b*R_w*l_r/2)*ddtheta_lr + T_wl + T_wr == 0;
eqn4 = (m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c)*(ddtheta_wl + ddtheta_wr) + ...
       m_l*l_wl*l_c*ddtheta_ll + m_l*l_wr*l_c*ddtheta_lr - I_b*ddtheta_b + ...
       m_b*g*l_c*theta_b - (T_wl + T_wr)*l_c/R_w - (T_bl + T_br) == 0;
eqn5 = ((I_z*R_w)/(2*R_l) + I_w*R_l/R_w)*ddtheta_wl - ...
       ((I_z*R_w)/(2*R_l) + I_w*R_l/R_w)*ddtheta_wr + ...
       (I_z*l_l)/(2*R_l)*ddtheta_ll - (I_z*l_r)/(2*R_l)*ddtheta_lr - ...
       T_wl*R_l/R_w + T_wr*R_l/R_w == 0;

% 求解加速度表达式
[ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b] = ...
    solve(eqn1, eqn2, eqn3, eqn4, eqn5, ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b);

% 计算雅可比矩阵
J_A = jacobian([ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b], [theta_ll, theta_lr, theta_b]);
J_B = jacobian([ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b], [T_wl, T_wr, T_bl, T_br]);

% 构建10x10 A矩阵和10x4 B矩阵
A_sym = sym(zeros(10,10));
B_sym = sym(zeros(10,4));

% 填充A矩阵非零元素
for p = 5:2:9
    idx = (p-3)/2; % 对应theta_ll(1), theta_lr(2), theta_b(3)
    A_sym(2,p) = R_w*(J_A(1,idx) + J_A(2,idx))/2;
    A_sym(4,p) = (R_w*(-J_A(1,idx) + J_A(2,idx)))/(2*R_l) - (l_l*J_A(3,idx))/(2*R_l) + (l_r*J_A(4,idx))/(2*R_l);
    A_sym(6,p) = J_A(3,idx);
    A_sym(8,p) = J_A(4,idx);
    A_sym(10,p) = J_A(5,idx);
end
% 设置A矩阵的导数关系 (a12=a34=a56=a78=a9,10=1)
for i = 1:5
    A_sym(2*i-1, 2*i) = 1;
end

% 填充B矩阵
for h = 1:4
    B_sym(2,h) = R_w*(J_B(1,h) + J_B(2,h))/2;
    B_sym(4,h) = (R_w*(-J_B(1,h) + J_B(2,h)))/(2*R_l) - (l_l*J_B(3,h))/(2*R_l) + (l_r*J_B(4,h))/(2*R_l);
    B_sym(6,h) = J_B(3,h);
    B_sym(8,h) = J_B(4,h);
    B_sym(10,h) = J_B(5,h);
end

%% Step 2: 预编译符号函数（关键提速步骤）
fprintf('正在预编译符号函数...\n');
Af = matlabFunction(A_sym, 'Vars', {R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, ...
                                    m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g});
Bf = matlabFunction(B_sym, 'Vars', {R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, ...
                                    m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g});
fprintf('符号函数预编译完成.\n');

%% Step 3: 物理参数定义
g_ac     = 9.79;
R_w_ac   = 0.06;    % 驱动轮半径 (m)
R_l_ac   = 0.23;  % 轮距/2 (m)
l_c_ac   = 0.01863;   % 机体质心偏移 (m)
m_w_ac   = 0.65;      % 轮质量 (kg)
m_l_ac   = 1.21;      % 单腿质量 (kg)
m_b_ac   = 15.42;      % 机体质量 (kg)
I_w_ac   = 0.000922814; % 轮转动惯量
I_b_ac   = 0.2266;  % 机体俯仰惯量
I_z_ac   = 0.41;  % 绕Z轴惯量

% 腿部参数数据集 [腿长, 腿质心到轮距, 腿质心到转轴距, 腿部转动惯量]
Leg_data = [0.11,  0.4986*0.11 + 0.0553,  0.1347*0.11 + 0.0576,  0.0229*0.11 + 0.0107;
              0.12, 0.4986*0.12 + 0.0553,  0.1347*0.12 + 0.0576,  0.0229*0.12 + 0.0107;
              0.13, 0.4986*0.13 + 0.0553,  0.1347*0.13 + 0.0576,  0.0229*0.13 + 0.0107;
              0.14,  0.4986*0.14 + 0.0553,  0.1347*0.14 + 0.0576,  0.0229*0.14 + 0.0107;
              0.15,  0.4986*0.15 + 0.0553,   0.1347*0.15 + 0.0576,  0.0229*0.15 + 0.0107;
              0.16,  0.4986*0.16 + 0.0553,   0.1347*0.16 + 0.0576,  0.0229*0.16 + 0.0107;
              0.17,  0.4986*0.17 + 0.0553,   0.1347*0.17 + 0.0576,  0.0229*0.17 + 0.0107;
              0.18,  0.4986*0.18 + 0.0553,   0.1347*0.18 + 0.0576,  0.0229*0.18 + 0.0107;
              0.19,  0.4986*0.19 + 0.0553,   0.1347*0.19 + 0.0576,  0.0229*0.19 + 0.0107;
              0.20,  0.4986*0.20 + 0.0553,   0.1347*0.20 + 0.0576,  0.0229*0.20 + 0.0107;
              0.21,  0.4986*0.21 + 0.0553,   0.1347*0.21 + 0.0576,  0.0229*0.21 + 0.0107;
              0.22,  0.4986*0.22 + 0.0553,   0.1347*0.22 + 0.0576,  0.0229*0.22 + 0.0107;
              0.23,  0.4986*0.23 + 0.0553,   0.1347*0.23 + 0.0576,  0.0229*0.23 + 0.0107;
              0.24,  0.4986*0.24 + 0.0553,   0.1347*0.24 + 0.0576,  0.0229*0.24 + 0.0107;
              0.25,  0.4986*0.25 + 0.0553,   0.1347*0.25 + 0.0576,  0.0229*0.25 + 0.0107;
              0.26,  0.4986*0.26 + 0.0553,   0.1347*0.26 + 0.0576,  0.0229*0.26 + 0.0107;
              0.27,  0.4986*0.27 + 0.0553,   0.1347*0.27 + 0.0576,  0.0229*0.27 + 0.0107;
              0.28,  0.4986*0.28 + 0.0553,   0.1347*0.28 + 0.0576,  0.0229*0.28 + 0.0107;
              0.29,  0.4986*0.29 + 0.0553,   0.1347*0.29 + 0.0576,  0.0229*0.29 + 0.0107;
              0.30,  0.4986*0.30 + 0.0553,   0.1347*0.30 + 0.0576,  0.0229*0.30 + 0.0107;
              0.30,  0.4986*0.31 + 0.0553,   0.1347*0.31 + 0.0576,  0.0229*0.31 + 0.0107];

%% Step 4: LQR权重矩阵
Q = diag([700, 550, 600, 1, 500, 1, 500, 1, 20000, 1]);
R = diag([35, 35, 1, 1]);

%% Step 5: 【关键调试】展示腿长0.15m时的精确K矩阵 + 行优先展开验证
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║   腿长 = 0.15m (左右对称) 时的 LQR 增益矩阵 K (4×10)                          ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════════════════════╝\n\n');

% 获取0.15m腿长参数
l_l_ac = 0.15; l_r_ac = 0.15;
l_wl_ac = Leg_data(1,2); l_wr_ac = Leg_data(1,2);
l_bl_ac = Leg_data(1,3); l_br_ac = Leg_data(1,3);
I_ll_ac = Leg_data(1,4); I_lr_ac = Leg_data(1,4);

% 计算数值A/B矩阵
A_015 = Af(R_w_ac, R_l_ac, l_l_ac, l_r_ac, l_wl_ac, l_wr_ac, l_bl_ac, l_br_ac, l_c_ac, ...
           m_w_ac, m_l_ac, m_b_ac, I_w_ac, I_ll_ac, I_lr_ac, I_b_ac, I_z_ac, g_ac);
B_015 = Bf(R_w_ac, R_l_ac, l_l_ac, l_r_ac, l_wl_ac, l_wr_ac, l_bl_ac, l_br_ac, l_c_ac, ...
           m_w_ac, m_l_ac, m_b_ac, I_w_ac, I_ll_ac, I_lr_ac, I_b_ac, I_z_ac, g_ac);
A_015 = double(A_015);
B_015 = double(B_015);

% 求解LQR
[~, K_015, ~] = icare(A_015, B_015, Q, R);

% 美化输出K矩阵
fprintf('K 矩阵 (4行×10列，行 = 控制通道，列 = 状态变量):\n');
state_names = {'s', 'ds', 'phi', 'dphi', 'theta_ll', 'dtheta_ll', 'theta_lr', 'dtheta_lr', 'theta_b', 'dtheta_b'};
fprintf('状态: ');
for j = 1:10
    fprintf('%10s ', state_names{j});
end
fprintf('\n');
for i = 1:4
    ctrl_name = {'T_wl', 'T_wr', 'T_bl', 'T_br'};
    fprintf('%s: ', ctrl_name{i});
    for j = 1:10
        fprintf('%10.4f ', K_015(i,j));
    end
    fprintf('\n');
end

% 【关键修正】行优先展开验证 (row-major: 先第一行全部10个元素，再第二行...)
K_row_major = reshape(K_015.', 1, 40); % 转置后reshape = 行优先
fprintf('\n✅ 行优先展开顺序 (40个元素，用于拟合):\n');
fprintf('   [');
for k = 1:40
    fprintf('%.4f', K_row_major(k));
    if k < 40, fprintf(', '); end
    if mod(k,10)==0 && k<40, fprintf('...\n    '); end % 每10个元素换行便于阅读
end
fprintf(']\n');

% 物理意义注释
fprintf('\n【物理意义】\n');
fprintf('  前10个元素: T_wl 对 [s, ds, phi, ..., theta_b, dtheta_b] 的反馈增益\n');
fprintf('  中10个元素: T_wr 对所有状态的反馈增益\n');
fprintf('  后20个元素: T_bl, T_br 对所有状态的反馈增益\n');
fprintf('\n');

%% Step 6: 网格采样计算K矩阵（441个组合）- 严格行优先展开
l_vals = 0.15:0.01:0.35;
n = length(l_vals);
total = n * n;
K_sample = zeros(total, 3, 40); % [l_l, l_r, K_row_major_40]

fprintf('正在计算 %d 组腿长组合的 LQR 增益 (行优先展开)...\n', total);
progress = 0;
for i = 1:n
    for j = 1:n
        progress = progress + 1;
        if mod(progress, 50) == 0
            fprintf('  进度: %d/%d (%.1f%%)\n', progress, total, 100*progress/total);
        end
        
        % 左右腿参数
        l_l_ac = l_vals(i); l_r_ac = l_vals(j);
        l_wl_ac = Leg_data(i,2); l_wr_ac = Leg_data(j,2);
        l_bl_ac = Leg_data(i,3); l_br_ac = Leg_data(j,3);
        I_ll_ac = Leg_data(i,4); I_lr_ac = Leg_data(j,4);
        
        % 计算数值A/B矩阵
        A_num = Af(R_w_ac, R_l_ac, l_l_ac, l_r_ac, l_wl_ac, l_wr_ac, l_bl_ac, l_br_ac, l_c_ac, ...
                   m_w_ac, m_l_ac, m_b_ac, I_w_ac, I_ll_ac, I_lr_ac, I_b_ac, I_z_ac, g_ac);
        B_num = Bf(R_w_ac, R_l_ac, l_l_ac, l_r_ac, l_wl_ac, l_wr_ac, l_bl_ac, l_br_ac, l_c_ac, ...
                   m_w_ac, m_l_ac, m_b_ac, I_w_ac, I_ll_ac, I_lr_ac, I_b_ac, I_z_ac, g_ac);
        A_num = double(A_num); B_num = double(B_num);
        
        % 求解LQR
        [~, K, ~] = icare(A_num, B_num, Q, R);
        
        % 【关键修正】严格行优先展开 (先第一行10个，再第二行10个...)
        K_vec = reshape(K.', 1, 40); % 转置后reshape = 行优先顺序
        
        % 存储数据
        sample_idx = (i-1)*n + j;
        K_sample(sample_idx, 1, :) = l_l_ac;
        K_sample(sample_idx, 2, :) = l_r_ac;
        K_sample(sample_idx, 3, :) = K_vec;
    end
end

%% Step 7: 二次曲面拟合 (poly22) - 按行优先顺序拟合40个元素
fprintf('正在进行曲面拟合 (行优先顺序)...\n');
K_Fit_Coefficients = zeros(40, 6); % 40个K元素，每个6个系数

for k = 1:40
    x = K_sample(:, 1, k); % l_l
    y = K_sample(:, 2, k); % l_r
    z = K_sample(:, 3, k); % K_value (行优先第k个元素)
    f = fit([x, y], z, 'poly22');
    K_Fit_Coefficients(k, :) = coeffvalues(f); % [p00, p10, p01, p20, p11, p02]
end

%% Step 8: 输出C语言格式系数矩阵 (行优先顺序)
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║   C语言格式拟合系数 (40行×6列，严格行优先顺序)                              ║\n');
fprintf('║   顺序: K11..K1,10, K21..K2,10, K31..K3,10, K41..K4,10                       ║\n');
fprintf('║   即: [T_wl增益(10个), T_wr增益(10个), T_bl增益(10个), T_br增益(10个)]       ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════════════════════╝\n\n');
fprintf('float K_fit_coeffs[40][6] = {\n');
for k = 1:40
    coeffs = K_Fit_Coefficients(k, :);
    line = sprintf('  {%.5g, %.5g, %.5g, %.5g, %.5g, %.5g}', ...
                   coeffs(1), coeffs(2), coeffs(3), coeffs(4), coeffs(5), coeffs(6));
    if k < 40, line = [line ',']; end
    fprintf('%s\n', line);
    
    % 每10个元素添加注释分隔（便于阅读）
    % if mod(k,10)==0 && k<40
    %     ctrl_name = {'T_wl', 'T_wr', 'T_bl', 'T_br'};
    %     fprintf('  // --- %s 增益结束，%s 增益开始 ---\n', ctrl_name{k/10}, ctrl_name{k/10+1});
    %end
end
fprintf('};\n');

% C代码使用示例
fprintf('\n💡 C代码使用示例 (行优先重建K矩阵):\n');
fprintf('   float K[4][10];\n');
fprintf('   for (int i = 0; i < 40; i++) {\n');
fprintf('       int row = i / 10;  // 0=T_wl, 1=T_wr, 2=T_bl, 3=T_br\n');
fprintf('       int col = i %% 10; // 0=s, 1=ds, ..., 9=dtheta_b\n');
fprintf('       K[row][col] = eval_poly(K_fit_coeffs[i], l_left, l_right);\n');
fprintf('   }\n');

% 验证：用拟合系数重建0.15m腿长的K矩阵
fprintf('\n🔍 验证: 用拟合系数重建0.15m腿长K矩阵 (最大绝对误差):\n');
l_test = 0.15;
K_fitted = zeros(4,10);
for i = 1:40
    c = K_Fit_Coefficients(i,:);
    K_fitted_val = c(1) + c(2)*l_test + c(3)*l_test + c(4)*l_test^2 + c(5)*l_test*l_test + c(6)*l_test^2;
    row = floor((i-1)/10) + 1;
    col = mod(i-1,10) + 1;
    K_fitted(row,col) = K_fitted_val;
end
max_err = max(abs(K_fitted(:) - K_015(:)));
fprintf('   max|K_fitted - K_exact| = %.4e\n', max_err);
if max_err < 1e-3
    fprintf('   ✅ 拟合精度良好 (误差 < 0.1%%)\n');
else
    fprintf('   ⚠️  拟合误差较大，请检查拟合质量\n');
end

fprintf('\n✅ 全流程完成! 总耗时: %.2f 秒\n', toc);
fprintf('💡 提示: 拟合系数严格按控制通道分组（行优先），符合控制工程实践。\n');