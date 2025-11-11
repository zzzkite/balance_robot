%计算不同腿长下适合的K矩阵，再进行多项式拟合，得到2*6矩阵每个参数对应的多项式参数
tic
j=1;
leg=0.1:0.01:0.21                                                                            ;
for i=leg
    k=get_k_length(i);
    k11(j) = k(1,1);
    k12(j) = k(1,2);
    k13(j) = k(1,3);
    k14(j) = k(1,4);
    k15(j) = k(1,5);
    k16(j) = k(1,6);

    k21(j) = k(2,1);
    k22(j) = k(2,2);
    k23(j) = k(2,3);
    k24(j) = k(2,4);
    k25(j) = k(2,5);
    k26(j) = k(2,6);
    j=j+1;
end
a11=polyfit(leg,k11,3);
a12=polyfit(leg,k12,3);
a13=polyfit(leg,k13,3);
a14=polyfit(leg,k14,3);
a15=polyfit(leg,k15,3);
a16=polyfit(leg,k16,3);

a21=polyfit(leg,k21,3);
a22=polyfit(leg,k22,3);
a23=polyfit(leg,k23,3);
a24=polyfit(leg,k24,3);
a25=polyfit(leg,k25,3);
a26=polyfit(leg,k26,3);

%% 绘图测试
figure(1)
leg_fit = linspace(min(leg), max(leg), 100);

k11_fit = polyval(a11, leg_fit);
k12_fit = polyval(a12, leg_fit);
k13_fit = polyval(a13, leg_fit);
k14_fit = polyval(a14, leg_fit);
k15_fit = polyval(a15, leg_fit);
k16_fit = polyval(a16, leg_fit);

k21_fit = polyval(a21, leg_fit);
k22_fit = polyval(a22, leg_fit);
k23_fit = polyval(a23, leg_fit);
k24_fit = polyval(a24, leg_fit);
k25_fit = polyval(a25, leg_fit);
k26_fit = polyval(a26, leg_fit);

subplot(2,6,1);
plot(leg, k11, 'o', leg_fit, k11_fit, '-');
xlabel('LegLength');ylabel('K');title('k11');

subplot(2,6,2);
plot(leg, k12, 'o', leg_fit, k12_fit, '-');
xlabel('LegLength');ylabel('K');title('k12');

subplot(2,6,3);
plot(leg, k13, 'o', leg_fit, k13_fit, '-');
xlabel('LegLength');ylabel('K');title('k13');

subplot(2,6,4);
plot(leg, k14, 'o', leg_fit, k14_fit, '-');
xlabel('LegLength');ylabel('K');title('k14');

subplot(2,6,5);
plot(leg, k15, 'o', leg_fit, k15_fit, '-');
xlabel('LegLength');ylabel('K');title('k15');

subplot(2,6,6);
plot(leg, k16, 'o', leg_fit, k16_fit, '-');
xlabel('LegLength');ylabel('K');title('k16');

subplot(2,6,7);
plot(leg, k21, 'o', leg_fit, k21_fit, '-');
xlabel('LegLength');ylabel('K');title('k21');

subplot(2,6,8);
plot(leg, k22, 'o', leg_fit, k22_fit, '-');
xlabel('LegLength');ylabel('K');title('k22');

subplot(2,6,9);
plot(leg, k23, 'o', leg_fit, k23_fit, '-');
xlabel('LegLength');ylabel('K');title('k23');

subplot(2,6,10);
plot(leg, k24, 'o', leg_fit, k24_fit, '-');
xlabel('LegLength');ylabel('K');title('k24');

subplot(2,6,11);
plot(leg, k25, 'o', leg_fit, k25_fit, '-');
xlabel('LegLength');ylabel('K');title('k25');

subplot(2,6,12);
plot(leg, k26, 'o', leg_fit, k26_fit, '-');
xlabel('LegLength');ylabel('K');title('k26');

%% stm32版类代码测试
figure(2)
leg_fit = linspace(min(leg), max(leg), 100);

k11_fit_32 = a11(1).*leg_fit.*leg_fit.*leg_fit + a11(2).*leg_fit.*leg_fit + a11(3).*leg_fit + a11(4); %matlab里面的索引是从1开始
k12_fit_32 = a12(1).*leg_fit.*leg_fit.*leg_fit + a12(2).*leg_fit.*leg_fit + a12(3).*leg_fit + a12(4);
k13_fit_32 = a13(1).*leg_fit.*leg_fit.*leg_fit + a13(2).*leg_fit.*leg_fit + a13(3).*leg_fit + a13(4);
k14_fit_32 = a14(1).*leg_fit.*leg_fit.*leg_fit + a14(2).*leg_fit.*leg_fit + a14(3).*leg_fit + a14(4);
k15_fit_32 = a15(1).*leg_fit.*leg_fit.*leg_fit + a15(2).*leg_fit.*leg_fit + a15(3).*leg_fit + a15(4);
k16_fit_32 = a16(1).*leg_fit.*leg_fit.*leg_fit + a16(2).*leg_fit.*leg_fit + a16(3).*leg_fit + a16(4);

k21_fit_32 = a21(1).*leg_fit.*leg_fit.*leg_fit + a21(2).*leg_fit.*leg_fit + a21(3).*leg_fit + a21(4); 
k22_fit_32 = a22(1).*leg_fit.*leg_fit.*leg_fit + a22(2).*leg_fit.*leg_fit + a22(3).*leg_fit + a22(4);
k23_fit_32 = a23(1).*leg_fit.*leg_fit.*leg_fit + a23(2).*leg_fit.*leg_fit + a23(3).*leg_fit + a23(4);
k24_fit_32 = a24(1).*leg_fit.*leg_fit.*leg_fit + a24(2).*leg_fit.*leg_fit + a24(3).*leg_fit + a24(4);
k25_fit_32 = a25(1).*leg_fit.*leg_fit.*leg_fit + a25(2).*leg_fit.*leg_fit + a25(3).*leg_fit + a25(4);
k26_fit_32 = a26(1).*leg_fit.*leg_fit.*leg_fit + a26(2).*leg_fit.*leg_fit + a26(3).*leg_fit + a26(4);

subplot(2,6,1);
plot(leg, k11, 'o', leg_fit, k11_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k11 stm32');

subplot(2,6,2);
plot(leg, k12, 'o', leg_fit, k12_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k12 stm32');

subplot(2,6,3);
plot(leg, k13, 'o', leg_fit, k13_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k13 stm32');

subplot(2,6,4);
plot(leg, k14, 'o', leg_fit, k14_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k14 stm32');

subplot(2,6,5);
plot(leg, k15, 'o', leg_fit, k15_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k15 stm32');

subplot(2,6,6);
plot(leg, k16, 'o', leg_fit, k16_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k16 stm32');

subplot(2,6,7);
plot(leg, k21, 'o', leg_fit, k21_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k21 stm32');

subplot(2,6,8);
plot(leg, k22, 'o', leg_fit, k22_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k22 stm32');

subplot(2,6,9);
plot(leg, k23, 'o', leg_fit, k23_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k23 stm32');

subplot(2,6,10);
plot(leg, k24, 'o', leg_fit, k24_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k24 stm32');

subplot(2,6,11);
plot(leg, k25, 'o', leg_fit, k25_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k25 stm32');

subplot(2,6,12);
plot(leg, k26, 'o', leg_fit, k26_fit_32, '-');
xlabel('LegLength');ylabel('K');title('k26 stm32');
%% 产生可以直接复制到stm32中的矩阵
% 合并所有系数到矩阵（按行顺序排列）
Poly_Coefficient = [a11; a12; a13; a14; a15; a16; ...
                    a21; a22; a23; a24; a25; a26];
% 生成C语言二维数组代码
fprintf('float Poly_Coefficient[12][4] = {\n');
for i = 1:size(Poly_Coefficient, 1)
    fprintf('\t{');
    for j = 1:size(Poly_Coefficient, 2)
        % 格式化数值（保留6位小数，避免科学计数法）
        fprintf('%.6ff', Poly_Coefficient(i, j));
        if j < size(Poly_Coefficient, 2)
            fprintf(',\t');
        end
    end
    fprintf('}');
    if i < size(Poly_Coefficient, 1)
        fprintf(',');
    end
    fprintf('\n');
end
fprintf('};\n');
