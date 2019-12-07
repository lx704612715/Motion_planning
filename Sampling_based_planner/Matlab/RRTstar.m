%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 流程初始化
clear all; close all;
x_I=1; y_I=1;           % 设置初始点
x_G=700; y_G=700;       % 设置目标点
Thr=50;                 % 设置目标点阈值
Delta=30;              % 设置扩展步长
Thr_near=50;           % Threshold for choosing parents points
%% 建树初始化
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I;
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;  
T.v(1).cost = 0;%
%% 开始构建树——作业部分
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%地图x轴长度
yL=size(Imp,2);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
count=1;
x_rands=int16(800*rand(10000,2));
plt = [];
plt_path = [];
find_path = false;
path_cost = [];
for iter = 1:10000
    x_rand=[];
    %Step 1: 在地图中随机采样一个点x_rand
    %提示：用（x_rand(1),x_rand(2)）表示环境中采样点的坐标
    x_rand(1) = x_rands(iter,1);
    x_rand(2) = x_rands(iter,2);

    x_near=[];
    %Step 2: 遍历树，从树中找到最近邻近点x_near
    %提示：x_near已经在树T里
    num_nodes = size(T.v,2);
    %num_nodes = count;
    dist=zeros(num_nodes,1);
    for i=1:num_nodes
        dist(i) = sqrt((T.v(i).x-x_rand(1))^2+(T.v(i).y-x_rand(2))^2);
    end
    x_near_idx = find(dist==min(dist));
    if (min(dist)<=2)
        continue
    end
    x_near(1) = T.v(x_near_idx).x;
    x_near(2) = T.v(x_near_idx).y;
    x_new=[];
    %Step 3: 扩展得到x_new节点
    %提示：注意使用扩展步长Delta
    if (min(dist)>=Delta)
        scale = Delta/min(dist);
        x_new(1) = int16(x_near(1) + scale*(x_rand(1)-x_near(1)));
        x_new(2) = int16(x_near(2) + scale*(x_rand(2)-x_near(2)));
    else
        x_new(1) = x_rand(1);
        x_new(2) = x_rand(2);
    end
    
    dist_choosePar=zeros(num_nodes,1);
    for i=1:num_nodes
        dist_choosePar(i) = sqrt((T.v(i).x-x_new(1))^2+(T.v(i).y-x_new(2))^2);
    end
    
    %检查节点是否是collision-free
    if or(~collisionChecking(x_near,x_new,Imp),min(dist_choosePar)<=0.05)
        continue;
    end
    count=count+1;
    
    X_nears_idx = find(dist_choosePar<=Thr_near);
    num_X_nears = size(X_nears_idx, 1);
    
    costs=zeros(num_X_nears,1);
    for i=1:num_X_nears
        idx = X_nears_idx(i);
        costs(i) = T.v(idx).cost + dist_choosePar(idx);
    end
    min_cost_idx = find(costs==min(costs));
    par_idx = X_nears_idx(min_cost_idx);
    if size(par_idx,1)>1
        par_idx=par_idx(1);
    end
    %Step 4: 将x_new插入树T
    %提示：新节点x_new的父节点是x_near
    T.v(count).x = x_new(1);      
    T.v(count).y = x_new(2);
    T.v(count).xPrev = T.v(par_idx).x;   
    T.v(count).yPrev = T.v(par_idx).y;
    T.v(count).dist=sqrt((T.v(count).xPrev-x_new(1))^2+(T.v(count).yPrev-x_new(2))^2);
    T.v(count).indPrev = par_idx;
    T.v(count).cost = T.v(par_idx).cost + T.v(count).dist;
    plot([T.v(count).x,T.v(count).xPrev], [T.v(count).y, T.v(count).yPrev]);
    hold on
    
    for i=1:num_X_nears
        idx = X_nears_idx(i);
        revier_cost = T.v(count).cost + sqrt((T.v(count).x-T.v(idx).x)^2+(T.v(count).y-T.v(idx).y)^2);
        if (and(T.v(idx).cost > revier_cost, collisionChecking([T.v(count).x, T.v(count).y],[T.v(idx).x, T.v(idx).y],Imp)))
       %     delete(plt(idx));
        %    hold on
            T.v(idx).xPrev = T.v(count).x;
            T.v(idx).yPrev = T.v(count).y;
            T.v(idx).dist = dist_choosePar(idx);
            T.v(idx).indPrev = count;
            T.v(idx).cost = revier_cost;
            plot([T.v(count).x,T.v(idx).x], [T.v(count).y, T.v(idx).y]);
            hold on
        end
    end
    
    %Step 5:检查是否到达目标点附近
    %提示：注意使用目标点阈值Thr，若当前节点和终点的欧式距离小于Thr，则跳出当前for循环
    dist2go = sqrt((x_G-x_new(1))^2+(y_G-x_new(2))^2);
    if (and(dist2go <= Thr, ~find_path))
        find_path = true;
        disp('Path found!');
        disp(T.v(end).cost);
    end
    if (and(find_path, mod(iter,50)==0))
        if (size(plt_path,2) > 1)
            delete(plt_path)
            hold on
        end
        dist2gos=zeros(count,1);
        for i=1:count
            dist_choosePar(i) = sqrt((T.v(i).x-x_G)^2+(T.v(i).y-y_G)^2);
        end
        goal_idxs = find(dist_choosePar<=Thr);
        num_goal_nodes = size(goal_idxs,1);
        
        path_costs=zeros(num_goal_nodes,1);
        
        for k=1:num_goal_nodes
            goal_idx = goal_idxs(k);
            path_costs(k)=T.v(goal_idx).cost;
        end
        
        min_cost_idx = find(path_costs==min(path_costs));
        path_cost(end+1)= min(path_costs);
        min_goal_idx = goal_idxs(min_cost_idx);
        
        path.pos(1).x = x_G; path.pos(1).y = y_G;
        path.pos(2).x = T.v(min_goal_idx).x; path.pos(2).y = T.v(min_goal_idx).y;
        pathIndex = T.v(min_goal_idx).indPrev; % 终点加入路径
        j=0;
        while 1
            path.pos(j+3).x = T.v(pathIndex).x;
            path.pos(j+3).y = T.v(pathIndex).y;
            pathIndex = T.v(pathIndex).indPrev;
            if pathIndex == 1
                break
            end
            j=j+1;
        end  % 沿终点回溯到起点
        path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
        for j = 2:length(path.pos)
            plt_path(j)=plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
            hold on
        end
    end
    %plot(x_new(1), x_new(2), 'go', 'MarkerSize',2, 'MarkerFaceColor','b');
   %Step 6:将x_near和x_new之间的路径画出来
   %提示 1：使用plot绘制，因为要多次在同一张图上绘制线段，所以每次使用plot后需要接上hold on命令
   %提示 2：在判断终点条件弹出for循环前，记得把x_near和x_new之间的路径画出来

   pause(0.0001); %暂停0.1s，使得RRT扩展过程容易观察
end
%% 路径已经找到，反向查询
if iter < 5000
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % 终点加入路径
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 沿终点回溯到起点
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 起点加入路径
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y;
path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end
