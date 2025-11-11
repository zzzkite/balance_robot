% 参数定义
L5 = 80;   % mm
L2 = 160;  % mm
L1 = 84;   % mm

% 电机角度范围（单位：度）
phi1_deg = 75:0.1:225;

% 转换为弧度用于计算
phi1_rad = deg2rad(phi1_deg);

% 计算 L0，根据修正后的公式
L0 = sqrt(L2^2 - (L5/2 - L1 .* cos(phi1_rad)).^2) + L1 .* sin(phi1_rad);

% 绘图
figure;
plot(phi1_deg, L0, 'r', 'LineWidth', 2);
hold on;  % 保持图形

% 计算并标记特定角度
mark_deg = [120, 210];
mark_rad = deg2rad(mark_deg);
mark_L0 = sqrt(L2^2 - (L5/2 - L1 .* cos(mark_rad)).^2) + L1 .* sin(mark_rad);

% 使用星号标记点（蓝色五角星，大小15）
scatter(mark_deg, mark_L0, 100, 'b', 'O', 'filled'); 

xlabel('\phi_1 (度)');
ylabel('L_0 (mm)');
title('腿长L_0 与电机角度 \phi_1 的关系图');
grid on;
legend('L0曲线', '特殊角度', 'Location', 'best');  % 添加图例
hold off;