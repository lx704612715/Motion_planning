%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ���̳�ʼ��
clear all; close all;
x_I=1; y_I=1;           % ���ó�ʼ��
x_G=700; y_G=700;       % ����Ŀ���
Thr=50;                 % ����Ŀ�����ֵ
Delta=30;              % ������չ����
Thr_near=50;           % Threshold for choosing parents points
%% ������ʼ��
T.v(1).x = x_I;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T.v(1).y = y_I;
T.v(1).xPrev = x_I;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T.v(1).indPrev = 0;  
T.v(1).cost = 0;%
%% ��ʼ������������ҵ����
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%��ͼx�᳤��
yL=size(Imp,2);%��ͼy�᳤��
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% ��������Ŀ���
count=1;
x_rands=int16(800*rand(10000,2));
plt = [];
plt_path = [];
find_path = false;
path_cost = [];
for iter = 1:10000
    x_rand=[];
    %Step 1: �ڵ�ͼ���������һ����x_rand
    %��ʾ���ã�x_rand(1),x_rand(2)����ʾ�����в����������
    x_rand(1) = x_rands(iter,1);
    x_rand(2) = x_rands(iter,2);

    x_near=[];
    %Step 2: ���������������ҵ�����ڽ���x_near
    %��ʾ��x_near�Ѿ�����T��
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
    %Step 3: ��չ�õ�x_new�ڵ�
    %��ʾ��ע��ʹ����չ����Delta
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
    
    %���ڵ��Ƿ���collision-free
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
    %Step 4: ��x_new������T
    %��ʾ���½ڵ�x_new�ĸ��ڵ���x_near
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
    
    %Step 5:����Ƿ񵽴�Ŀ��㸽��
    %��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
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
        pathIndex = T.v(min_goal_idx).indPrev; % �յ����·��
        j=0;
        while 1
            path.pos(j+3).x = T.v(pathIndex).x;
            path.pos(j+3).y = T.v(pathIndex).y;
            pathIndex = T.v(pathIndex).indPrev;
            if pathIndex == 1
                break
            end
            j=j+1;
        end  % ���յ���ݵ����
        path.pos(end+1).x = x_I; path.pos(end).y = y_I; % ������·��
        for j = 2:length(path.pos)
            plt_path(j)=plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
            hold on
        end
    end
    %plot(x_new(1), x_new(2), 'go', 'MarkerSize',2, 'MarkerFaceColor','b');
   %Step 6:��x_near��x_new֮���·��������
   %��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
   %��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�x_near��x_new֮���·��������

   pause(0.0001); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
end
%% ·���Ѿ��ҵ��������ѯ
if iter < 5000
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % �յ����·��
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % ���յ���ݵ����
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % ������·��
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y;
path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end
