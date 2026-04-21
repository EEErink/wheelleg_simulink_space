% 仅生成 0.15m 腿长下的 K，并拆分为 k11~k410 变量（2026/02/05）
clear; clc; tic;

%% 加载符号模型
[A_sym, B_sym, ~, ~] = Func_Cal_ABCD_Array();
syms rw Rl l_l l_r lw_l lw_r lb_l lb_r lc mw ml mb Iw Il_l Il_r Ib Izz g
Af = matlabFunction(A_sym, 'Vars', {rw, Rl, l_l, l_r, lw_l, lw_r, lb_l, lb_r, lc, ...
                                    mw, ml, mb, Iw, Il_l, Il_r, Ib, Izz, g});
Bf = matlabFunction(B_sym, 'Vars', {rw, Rl, l_l, l_r, lw_l, lw_r, lb_l, lb_r, lc, ...
                                    mw, ml, mb, Iw, Il_l, Il_r, Ib, Izz, g});

%% 参数
g_ac = 9.79849; R_w_ac = 0.06; R_l_ac = 0.23; l_c_ac = 0.04;
m_w_ac = 0.632; m_l_ac = 0.996; m_b_ac = 15.42;
I_w_ac = 0.0004822; I_b_ac = 0.2266; I_z_ac = 0.41;

% 固定腿长 0.15m
l_val = 0.15;
a1=0.4986; b1=0.0553; a2=0.1347; b2=0.0576; a3=0.0229; b3=0.0107;
l_l_ac =  l_val; l_r_ac = l_val;
lw_l_ac = 0.1;
lw_r_ac = 0.1;
lb_l_ac= 0.05;
lb_r_ac= 0.05;
I_ll_ac = a3*l_val + b3; I_lr_ac = a3*l_val + b3;

% LQR 权重
lqr_Q = diag([5, 100, 60, 20, 400,5, 400,5, 4000, 5]);
lqr_R = diag([10, 10, 1, 1]);

%% 计算 K
K = get_K_fast(R_w_ac, R_l_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, ...
               lb_l_ac, lb_r_ac, l_c_ac, m_w_ac, m_l_ac, m_b_ac, ...
               I_w_ac, I_ll_ac, I_lr_ac, I_b_ac, I_z_ac, g_ac, ...
               Af, Bf, lqr_Q, lqr_R);

%% 拆分为 k11, k12, ..., k410
for i = 1:4
    for j = 1:10
        varName = sprintf('k%d%d', i, j);
        assignin('base', varName, -K(i, j));
    end
end

disp('✅ 已生成以下变量到工作区:');
disp('k11 k12 ... k110');
disp('k21 k22 ... k210');
disp('k31 k32 ... k310');
disp('k41 k42 ... k410');
toc;

%% 辅助函数
function K = get_K_fast(rw_ac, Rl_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, ...
                        lb_l_ac, lb_r_ac, lc_ac, mw_ac, ml_ac, mb_ac, ...
                        Iw_ac, Il_l_ac, Il_r_ac, Ib_ac, Izz_ac, g_ac, Af, Bf, Q, R)
    A_num = double(Af(rw_ac, Rl_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, ...
                      lb_l_ac, lb_r_ac, lc_ac, mw_ac, ml_ac, mb_ac, ...
                      Iw_ac, Il_l_ac, Il_r_ac, Ib_ac, Izz_ac, g_ac));
    B_num = double(Bf(rw_ac, Rl_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, ...
                      lb_l_ac, lb_r_ac, lc_ac, mw_ac, ml_ac, mb_ac, ...
                      Iw_ac, Il_l_ac, Il_r_ac, Ib_ac, Izz_ac, g_ac));
    [~, K, ~] = icare(A_num, B_num, Q, R);
end