clc;
clear;
close all;

%% =========================
% 杆长参数
%% =========================

L_AB = 0.0945;
L_BC = 0.1125;
L_CD = 0.116;

L_AD = 0.090;
L_AG = 0.210;

L_FG = 0.060;

L_GH = 0.250;

%% =========================
% 创建图窗
%% =========================

fig = figure('Name','Closed Chain IK',...
             'NumberTitle','off');

ax = axes('Parent',fig);

axis equal;
grid on;
hold on;

xlim([-0.5 0.5]);
ylim([-0.5 0.5]);

set(ax,'YDir','reverse');

xlabel('X');
ylabel('Y');

%% =========================
% H点目标滑条
%% =========================

sliderX = uicontrol( ...
    'Style','slider',...
    'Min',-0.3,...
    'Max',0.3,...
    'Value',0.15,...
    'Position',[100 10 300 20]);

sliderY = uicontrol( ...
    'Style','slider',...
    'Min',-0.3,...
    'Max',0.3,...
    'Value',0.10,...
    'Position',[500 10 300 20]);

txt1 = uicontrol( ...
    'Style','text',...
    'Position',[150 35 200 20],...
    'String','Hx');

txt2 = uicontrol( ...
    'Style','text',...
    'Position',[550 35 200 20],...
    'String','Hy');

%% =========================
% 初值
%% =========================

phi_init = deg2rad([70 140]);

%% =========================
% 主循环
%% =========================

while ishandle(fig)

    %% =========================
    % 多解逆运动学（升级版）
    %% =========================
    
    H_target = [
        sliderX.Value,...
        sliderY.Value
    ];
    phi_init_set = [
        phi_init;
        deg2rad([60 120]);
        deg2rad([-60 120]);
        deg2rad([60 -120]);
        deg2rad([-60 -120]);
    ];
    
    solutions = [];
    
    for branch = 1:2   % C1 / C2
    
        for k = 1:size(phi_init_set,1)
    
            phi_init_k = phi_init_set(k,:);
    
            fun = @(x) IK_Error_branch( ...
                x,...
                H_target,...
                L_AB,L_BC,L_CD,L_AD,L_AG,L_FG,L_GH,...
                branch);
    
            options = optimoptions( ...
                'fsolve',...
                'Display','off',...
                'FunctionTolerance',1e-8,...
                'StepTolerance',1e-8);
    
            [sol,~,exitflag] = fsolve(fun,phi_init_k,options);
    
            if exitflag > 0
                solutions(end+1,:) = sol; %#ok<AGROW>
            end
    
        end
    end
    
    %% =========================
    % 无解处理
    %% =========================
    if isempty(solutions)
    
        cla(ax);
        text(0,0,'逆解失败','FontSize',20);
        drawnow;
        continue;
    
    end
    
    %% =========================
    % 选择“最连续”的解（关键）
    %% =========================
    
    dist = vecnorm(solutions - phi_init, 2, 2);
    [~,idx] = min(dist);
    
    sol = solutions(idx,:);
    
    phi_init = sol;
    
    phi1 = sol(1);
    phi2 = sol(2);

    %% =========================
    % 正运动学绘图
    %% =========================

    [A,B,C,D,E,F,G,H] = FK_branch( ...
        phi1,...
        phi2,...
        L_AB,...
        L_BC,...
        L_CD,...
        L_AD,...
        L_AG,...
        L_FG,...
        L_GH,...
        1);  % 使用第一支路

    %% =========================
    % 绘图
    %% =========================

    cla(ax);

    hold on;

    axis equal;
    grid on;

    xlim([-0.5 0.5]);
    ylim([-0.5 0.5]);

    set(ax,'YDir','reverse');

    %% 杆件

    plot([A(1) B(1)], [A(2) B(2)],...
        'r','LineWidth',3);

    plot([B(1) C(1)], [B(2) C(2)],...
        'g','LineWidth',3);

    plot([C(1) D(1)], [C(2) D(2)],...
        'b','LineWidth',3);

    plot([A(1) D(1)], [A(2) D(2)],...
        'm','LineWidth',3);

    plot([A(1) G(1)], [A(2) G(2)],...
        'k','LineWidth',3);

    plot([F(1) G(1)], [F(2) G(2)],...
        'c','LineWidth',3);

    plot([G(1) H(1)], [G(2) H(2)],...
        '--','LineWidth',3);

    plot([D(1) E(1)], [D(2) E(2)],...
        'Color',[0.8 0.2 0.8],...
        'LineWidth',3);

    plot([E(1) F(1)], [E(2) F(2)],...
        'Color',[0.2 0.8 0.8],...
        'LineWidth',3);

    %% 点

    pts = [A;B;C;D;E;F;G;H];

    plot(pts(:,1),pts(:,2),...
        'ko',...
        'MarkerSize',8,...
        'MarkerFaceColor','y');

    %% 标签

    names = {'A','B','C','D','E','F','G','H'};

    for i = 1:length(names)

        text(pts(i,1)+0.01,...
             pts(i,2)+0.01,...
             names{i},...
             'FontSize',12);

    end

    %% 目标点

    plot(H_target(1),...
         H_target(2),...
         'rx',...
         'MarkerSize',15,...
         'LineWidth',3);

    %% 标题

    title(sprintf(...
        'phi1 = %.2f deg    phi2 = %.2f deg',...
        rad2deg(phi1),...
        rad2deg(phi2)));

    drawnow;

end

%% =========================================================
% 逆运动学误差函数
%% =========================================================
function err = IK_Error_branch( ...
    x,...
    H_target,...
    L_AB,...
    L_BC,...
    L_CD,...
    L_AD,...
    L_AG,...
    L_FG,...
    L_GH,...
    branch)

phi1 = x(1);
phi2 = x(2);

try

    [~,~,~,~,~,~,~,H] = FK_branch( ...
        phi1,...
        phi2,...
        L_AB,...
        L_BC,...
        L_CD,...
        L_AD,...
        L_AG,...
        L_FG,...
        L_GH,...
        branch);

    err = H - H_target;

catch

    err = [1e3;1e3];

end
end

%% =========================================================
% 正运动学
%% =========================================================

function [A,B,C,D,E,F,G,H] = FK_branch( ...
    phi1,...
    phi2,...
    L_AB,...
    L_BC,...
    L_CD,...
    L_AD,...
    L_AG,...
    L_FG,...
    L_GH,...
    branch)

A = [0 0];

B = [L_AB*cos(phi2), L_AB*sin(phi2)];
D = [L_AD*cos(phi1), L_AD*sin(phi1)];
G = [L_AG*cos(phi1), L_AG*sin(phi1)];

dx = D(1)-B(1);
dy = D(2)-B(2);
d = sqrt(dx^2 + dy^2);

if d > (L_BC + L_CD)
    error('不可达');
end

a = (L_BC^2 - L_CD^2 + d^2)/(2*d);
h = sqrt(max(L_BC^2 - a^2,0));

P = B + a*[dx dy]/d;

C1 = [P(1)-h*dy/d, P(2)+h*dx/d];
C2 = [P(1)+h*dy/d, P(2)-h*dx/d];

%% =========================
% ⭐ 双支路
%% =========================
if branch == 1
    C = C1;
else
    C = C2;
end

theta = atan2(C(2)-D(2), C(1)-D(1));
FG_dir = [cos(theta), sin(theta)];

F = G - L_FG*FG_dir;
E = F - (G - D);
H = G + L_GH*FG_dir;

end