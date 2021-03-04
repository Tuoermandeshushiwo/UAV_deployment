
clc;clear all;close all;
%% 初始化种群

%f= Flow_BS(X); % 函数表达式
fym = -1;
%ITER=1;
for ITER = 1:1
k_center = 3;
N = 200;                         % 初始种群个数
d = k_center*3;                  % 空间维数
ger = 50;                        % 最大迭代次数    

limit = [0, 500;0,500;10,100;
         0, 500;0,500;10,100;
         0, 500;0,500;10,100;
         0, 500;0,500;10,100;
         0, 500;0,500;10,100];                % 设置位置参数限制


vlimit = [-15, 15];             % 设置速度限制
w = 0.8;                        % 惯性权重
c1 = 0.6;                       % 自我学习因子
c2 = 0.9;                       % 群体学习因子 
for users_sets = 40   
load('alluser.mat', 'all_users');
C(1,:) = zeros(1,k_center*2);
for i = 1:N
    [A_label,B] = kmeans(all_users,k_center);
    C(i+1,:) = B(:);
    if C(i+1) == C(i)
        [A_label,B] = kmeans(all_users,k_center);
    end
    for j = 1:k_center
        h(j,1) = 50;
    end
    B = [B h]';
    X(i,:) = B(:); %初始种群的位置
end
C(1,:) = [];

%     for i = 1:d
%       X(:,i) = limit(i, 1) + (limit(i, 2) - limit(i, 1)) * rand(N, 1);%初始种群的位置
%     end

v = 15*rand(N, 3*k_center);      % 初始种群的速度
xm = X;                          % 每个个体的历史最佳位置
ym = zeros(1, d);                % 种群的历史最佳位置
fxm = zeros(N, 1);               % 每个个体的历史最佳适应度
fym = -inf;                      % 种群历史最佳适应度
hold on
%% 群体更新
iter = 1;

record = zeros(ger, 1);          % 记录器
while iter <= ger 
 parfor i = 1:N
     [fx(i),p] = Flow_UAV(X(i,:),users_sets) ; % 个体当前适应度 
 end
     for i = 1:N  
        if fxm(i) < fx(i)
            fxm(i) = fx(i);     % 更新个体历史最佳适应度
            xm(i,:) = X(i,:);   % 更新个体历史最佳位置
        end 
     end
     
if fym < max(fxm)
        [fym, nmax] = max(fxm);   % 更新群体历史最佳适应度
        ym = xm(nmax, :);      % 更新群体历史最佳位置
end
w= 0.8-0.6*iter/ger;
best(iter,:)= ym;
    v = v * w + c1 * rand * (xm - X) + c2 * rand * (repmat(ym, N, 1) - X);% 速度更新
   
    %边界速度处理
    v(v > vlimit(2)) = vlimit(2);
    v(v < vlimit(1)) = vlimit(1);
    X = X + v;% 位置更新
    % 边界位置处理
    for i = 1:N
        for j = 1:d
            if X(i,[j]) > 500
             X(i,[j]) = 500;
            end
            if X(i,[j]) < 0
             X(i,[j]) = 0;
            end
        end
        for j = 3:3:d
            if X(i,[j]) > 100
             X(i,[j]) = 100;
            end
            if X(i,[j]) < 30
             X(i,[j]) = 30;
            end
        end
    end   
    record(iter) = fym;%最大值记录 real value
iter = iter+1;
  
%SINR(users_sets)  = record(end);
 
end


% SINR(ITER,users_sets) = record(end);
% if SINR(users_sets)<1
%     k_center = k_center+1;
%    users_sets=users_sets;
% end
[none,p]=Flow_UAV(best(end,:),users_sets);
for i = 1:users_sets
    P(i,users_sets) = p(i);
end
UAV_num(ITER,users_sets) = k_center;
end
C=[];X = [];v=[];xm=[];B = [];%best = [];
end
% plot3(best(:,1),best(:,2),best(:,3),'*',best(:,4),best(:,5),best(:,6),'^',best(:,7),best(:,8),best(:,9),'+',best(:,10),best(:,11),best(:,12),'+',best(:,13),best(:,14),best(:,15),'+');
%  title(' Convergence of PSO Algorithm');
%  xlabel('Horizontal Location(m)');
%   ylabel('Horizontal Location(m)');
%  zlabel('Attitude(m)');


 

 