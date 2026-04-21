% v6：全面优化计算效率，使用 matlabFunction 加速符号代入（2026/01/26）
% 融合HKUv4 SZUpublic等上交建模
tic
clear; clc;

%% Step 0: 加载符号模型并预编译为数值函数
[A_sym, B_sym, ~, ~] = Func_Cal_ABCD_Array();

% 定义符号变量（顺序必须与 Func_Cal_ABCD_Array 内部一致）
syms rw Rl l_l l_r lw_l lw_r lb_l lb_r lc mw ml mb Iw Il_l Il_r Ib Izz g

% 将符号 A、B 转为快速数值函数（关键提速步骤！）
Af = matlabFunction(A_sym, 'Vars', {rw, Rl, l_l, l_r, lw_l, lw_r, lb_l, lb_r, lc, ...
                                    mw, ml, mb, Iw, Il_l, Il_r, Ib, Izz, g});
Bf = matlabFunction(B_sym, 'Vars', {rw, Rl, l_l, l_r, lw_l, lw_r, lb_l, lb_r, lc, ...
                                    mw, ml, mb, Iw, Il_l, Il_r, Ib, Izz, g});

%% Step 1: 物理参数（固定部分）
g_ac     = 9.79849;
R_w_ac   = 0.06;        % 驱动轮半径 (m)
R_l_ac   = 0.23;        % 轮距/2 (m)
l_c_ac   = 0.004;         % 机体质心偏移 (m)
m_w_ac   = 0.632;       % 轮质量 (kg)
m_l_ac   = 0.996;       % 单腿质量 (kg)
m_b_ac   = 17.42;       % 机体质量 (kg)
I_w_ac   = 0.00049222;  % 轮转动惯量
I_b_ac   = 0.2266;      % 机体俯仰惯量
I_z_ac   = 0.41;        % 绕Z轴惯量

%% Step 2: 腿部参数数据集（l, lw, lb, I）
leg_lengths = (0.11:0.01:0.30)';  % 20个长度
a1 = 0.4986; b1 = 0.0553;  % lw = a1*l + b1
a2 = 0.1347; b2 = 0.0576;  % lb = a2*l + b2
a3 = 0.0229; b3 = 0.0107;  % I  = a3*l + b3

Leg_data = [leg_lengths, ...
            a1*leg_lengths + b1, ...
            a2*leg_lengths + b2, ...
            a3*leg_lengths + b3];

%% Step 3: LQR 权重矩阵
lqr_Q = diag([50, 100, 100, 10, 400,20, 400,20, 2000, 100]);
lqr_R = diag([15, 15, 1, 1]);


%% Step 4: 【可选】定腿长调试（快速验证）
if true  % 改为 false 可跳过
    idx = find(leg_lengths == 0.15);
    if isempty(idx), idx = 1; end
    l_l_ac = 0.15; l_r_ac = 0.15;
    lw_l_ac = Leg_data(idx,2); lw_r_ac = lw_l_ac;
    lb_l_ac = Leg_data(idx,3); lb_r_ac = lb_l_ac;
    I_ll_ac = Leg_data(idx,4); I_lr_ac = I_ll_ac;

    K_debug = get_K_fast(...
        R_w_ac, R_l_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, lb_l_ac, lb_r_ac, ...
        l_c_ac, m_w_ac, m_l_ac, m_b_ac, I_w_ac, I_ll_ac, I_lr_ac, I_b_ac, I_z_ac, g_ac, ...
        Af, Bf, lqr_Q, lqr_R);

    disp('✅ 定腿长 (0.15m) LQR 增益 K:');
    disp(K_debug);
end

%% Step 5: 【主流程】生成 K 表格并拟合（耗时但只需运行一次）
generate_fit = true;  % 设为 false 可跳过拟合

if generate_fit
    n = size(Leg_data,1);
    total = n * n;
    K_sample = zeros(total, 3, 40);  % [l_l, l_r, K_ij]

    fprintf('正在计算 %d 组腿长组合的 LQR 增益...\n', total);
    for i = 1:n
        for j = 1:n
            idx = (i-1)*n + j;
            
            % 左腿
            l_l_ac = Leg_data(i,1); lw_l_ac = Leg_data(i,2);
            lb_l_ac = Leg_data(i,3); I_ll_ac = Leg_data(i,4);
            % 右腿
            l_r_ac = Leg_data(j,1); lw_r_ac = Leg_data(j,2);
            lb_r_ac = Leg_data(j,3); I_lr_ac = Leg_data(j,4);
            
            K_sample(idx,1,:) = l_l_ac;
            K_sample(idx,2,:) = l_r_ac;
            
            K = get_K_fast(...
                R_w_ac, R_l_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, lb_l_ac, lb_r_ac, ...
                l_c_ac, m_w_ac, m_l_ac, m_b_ac, I_w_ac, I_ll_ac, I_lr_ac, I_b_ac, I_z_ac, g_ac, ...
                Af, Bf, lqr_Q, lqr_R);
            
            % 展平 K (4x10 → 40)
            K_vec = reshape(K.', [1, 40]);  % 注意转置+reshape顺序
            K_sample(idx,3,:) = K_vec;
        end
        if mod(i,5)==0, fprintf('  进度: %d/%d\n', i, n); end
    end

    % 拟合每个 K_ij 为 l_l, l_r 的二次多项式: p = c00 + c10*x + c01*y + c20*x^2 + c11*x*y + c02*y^2
    fprintf('正在进行曲面拟合...\n');
    K_Fit_Coefficients = zeros(40, 6);
    for k = 1:40
        x = K_sample(:,1,k);
        y = K_sample(:,2,k);
        z = K_sample(:,3,k);
        f = fit([x,y], z, 'poly22');
        K_Fit_Coefficients(k,:) = coeffvalues(f);
    end

    % 输出 C 语言格式
    coeffs_str = sprintf(['{' strjoin(repmat({'%.5g'},1,6), ', ') '},\n'], K_Fit_Coefficients.');
    fprintf('\n✅ C语言格式拟合系数（共40行，每行6个系数）:\n\n');
    fprintf('float K_fit_coeffs[40][6] = {\n');
    fprintf('%s', coeffs_str);
    fprintf('};\n');
end

toc


%% ==================== 辅助函数 ====================
function K = get_K_fast(...
    rw_ac, Rl_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, lb_l_ac, lb_r_ac, ...
    lc_ac, mw_ac, ml_ac, mb_ac, Iw_ac, Il_l_ac, Il_r_ac, Ib_ac, Izz_ac, g_ac, ...
    Af, Bf, Q, R)

    % 快速计算数值 A, B 矩阵
    A_num = Af(rw_ac, Rl_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, lb_l_ac, lb_r_ac, ...
               lc_ac, mw_ac, ml_ac, mb_ac, Iw_ac, Il_l_ac, Il_r_ac, Ib_ac, Izz_ac, g_ac);
    B_num = Bf(rw_ac, Rl_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, lb_l_ac, lb_r_ac, ...
               lc_ac, mw_ac, ml_ac, mb_ac, Iw_ac, Il_l_ac, Il_r_ac, Ib_ac, Izz_ac, g_ac);

    A_num = double(A_num);
    B_num = double(B_num);

    % 求解 LQR
    [~, K, ~] = icare(A_num, B_num, Q, R);
end