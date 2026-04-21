function [F_thrust, Tp, L0, phi0] = calculateThrustJacobian(phi1, phi4, T1, T2, l1,l2,l3,l4,hip_width)
% calculateThrustJacobian - 基于雅可比矩阵计算五连杆推力
% 输入:
%   phi1, phi4 - 两个大腿角度 (rad)
%   T1, T2 - 两个关节力矩 (Nm)
%   param - 结构体包含机构参数
% 输出:
%   F_thrust - 沿虚拟腿方向的推力 (N)
%   Tp - 摆力矩 (Nm)
%   L0 - 虚拟腿长 (m)
%   phi0 - 虚拟腿角度 (rad)

    % 提取机构参数
  %  l1 = param.leg_1;      % AB
  %  l2 = param.leg_2;      % BC  
  %  l3 = param.leg_3;      % CD
   % l4 = param.leg_4;      % DE
  %  hip_width = param.hip_length;  % AE
    
    % 1. 计算五连杆几何关系
    % 点A(0,0), E(hip_width,0)
    % 点B: 从A出发
    xB = l1 * cos(phi1);
    yB = l1 * sin(phi1);
    
    % 点D: 从E出发
    xD = hip_width - l4 * cos(phi4);
    yD = l4 * sin(phi4);
    
    % 计算点C位置（通过B和D的交点）
    % 使用几何约束: |BC| = l2, |CD| = l3
    d = sqrt((xD - xB)^2 + (yD - yB)^2);  % BD距离
    
    if d > (l2 + l3) || d < abs(l2 - l3)
        error('五连杆机构无法闭合，请检查角度参数');
    end
    
    % 计算中间角度
    a = (l2^2 - l3^2 + d^2) / (2*d);
    h = sqrt(l2^2 - a^2);
    
    % 点C的两个可能位置
    xC1 = xB + a*(xD - xB)/d - h*(yD - yB)/d;
    yC1 = yB + a*(yD - yB)/d + h*(xD - xB)/d;
    
    xC2 = xB + a*(xD - xB)/d + h*(yD - yB)/d;
    yC2 = yB + a*(yD - yB)/d - h*(xD - xB)/d;
    
    % 选择合理的解（通常yC > 0）
    if yC1 > 0
        xC = xC1;
        yC = yC1;
    else
        xC = xC2;
        yC = yC2;
    end
    
    % 计算虚拟腿参数
    hip_center_x = hip_width / 2;
    L0 = sqrt((xC - hip_center_x)^2 + yC^2);
    phi0 = atan2(yC, xC - hip_center_x);
    
    % 2. 计算雅可比矩阵
    % 基于虚功原理: τ = J^T * F
    % 其中 F = [F_thrust; Tp] 是末端力/力矩
    
    % 计算各连杆角度
    phi2 = atan2(yC - yB, xC - xB);  % BC与x轴夹角
    phi3 = atan2(yD - yC, xD - xC);  % CD与x轴夹角
    
    % 雅可比矩阵元素（根据五连杆几何推导）
    j11 = l1 * sin(phi1 - phi2) * sin(phi0 - phi3) / sin(phi3 - phi2);
    j12 = l1 * cos(phi0 - phi3) * sin(phi1 - phi2) / (L0 * sin(phi3 - phi2));
    j21 = l4 * sin(phi3 - phi4) * sin(phi0 - phi2) / sin(phi3 - phi2);
    j22 = l4 * cos(phi0 - phi2) * sin(phi3 - phi4) / (L0 * sin(phi3 - phi2));
    
    % 构建雅可比矩阵转置
    J_T = [j11, j21;
           j12, j22];
    
    % 3. 计算末端力
    tau = [T1; T2];
    
    if abs(det(J_T)) < 1e-10
        warning('雅可比矩阵奇异，机构处于特殊位置');
        F_thrust = 0;
        Tp = 0;
    else
        F_end = J_T \ tau;
        F_thrust = F_end(1);
        Tp = F_end(2);
    end
    
    % 输出调试信息
    fprintf('五连杆几何计算:\n');
    fprintf('点B: (%.3f, %.3f)\n', xB, yB);
    fprintf('点C: (%.3f, %.3f)\n', xC, yC);
    fprintf('点D: (%.3f, %.3f)\n', xD, yD);
    fprintf('虚拟腿: L0=%.3fm, phi0=%.1f°\n', L0, rad2deg(phi0));
    fprintf('雅可比矩阵转置:\n');
    fprintf('  [%.3f, %.3f]\n', J_T(1,1), J_T(1,2));
    fprintf('  [%.3f, %.3f]\n', J_T(2,1), J_T(2,2));
    fprintf('行列式: %.6f\n', det(J_T));
    fprintf('计算结果: F_thrust=%.2fN, Tp=%.2fNm\n', F_thrust, Tp);
end