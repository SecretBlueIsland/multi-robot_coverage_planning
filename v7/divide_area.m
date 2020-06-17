function [ Area_rst ] = divide_area_v6( Core_flag, rolesArray )

robot_num = length(rolesArray);

% Area_flag:L*W (L:length of SOI, W:width of SOI, Area_flag(i,j)=1 means free space/0 means obstacles)
% init_pos:robot_num*2
% Area_rst:L*W*robot_num (Area_rst(:,:,i) means the area for robot i)
%% idea
% add a connectivity judge function
% original method: DARP-103行
%% prepare control points
Core_flag = Core_flag-1; %Area_flag(i,j)=0 means free space/-1 means obstacles
L = size(Core_flag,2); %length = cols = x
W = size(Core_flag,1); %width = rols = y

initK = zeros(W,L);
%display_area(Area_flag,1);
%绘制初始任务分配情况
ob=0;
for i=1:L
    for j=1:W
        if Core_flag(j,i)==-1
            ob = ob + 1;
            continue;
        end

        E = zeros(1,robot_num);
        for k=1:robot_num
            E(k) = norm([i,j] - rolesArray(k).coord);
        end
        [~,index] = min(E);
        initK(j,i) = index;  % 在地图上标记机器人编号       
    end
end
%figure(2);
%display_area(initK,robot_num,2);
%pause();

%% main loop

F = L*W;
alpha = ones(1,robot_num);
E = zeros(W,L,robot_num);
K = zeros(W,L);
C = ones(W,L,robot_num);

stop = 0;
iteration_count = 0;
while stop==0
    iteration_count = iteration_count + 1;
    for each= 1:robot_num
        rolesArray(each).taskAmount = 0; % 任务清零
    end
    ob = 0;
    count = 0;
        
    %% check every grid
    for i=1:L
        for j=1:W
            if Core_flag(j,i)==-1
                ob = ob + 1;
                continue;
            end
           
            for k=1:robot_num
                if alpha(k)<0
                    fprintf('error');
                    return
                end
                E(j,i,k) = C(j,i,k) * alpha(k) * norm([i,j] - rolesArray(k).coord);
                
            end
            [~,index] = min(E(j,i,:));
            
            K(j,i) = index; % 在地图上标记机器人编号
            rolesArray(index).taskAmount = rolesArray(index).taskAmount +1; % 任务量加1
        end
    end
    
    %% show image
    Area_rst = K;
    %figure(2);
    %handle2 = display_area(Area_rst,robot_num,2);
    
    %% update c
    % obtain the assignment matrix for every robot    
    for k=1:robot_num
       K_r = zeros(W,L);
       K_r(K==k)=1;
       rolesArray(k).taskDetail = K_r; % 每个机器人的区域

    end
    % obtain the connected sets for every robot
    for k=1:robot_num
        CON_t = [];        
        DCON_t = [];
        
        %K_t = rolesArray(k).taskDetail;
        for i=1:L
            for j=1:W
                if K(j,i)~=k
                   continue; 
                end
                if isconnect([j;i],rolesArray(k))==1
                   if size(CON_t)==0
                       CON_t = [j;i];
                   else
                       CON_t = [CON_t,[j;i]]; 
                   end
                else
                   if size(DCON_t)==0
                       DCON_t = [j;i];
                   else
                       DCON_t = [DCON_t,[j;i]]; 
                   end
                end
            end
        end
        rolesArray(k).CON = CON_t;
        rolesArray(k).DCON = DCON_t;
        
    end
    
    for k=1:robot_num
       CON_t = rolesArray(k).CON; 
       DCON_t = rolesArray(k).DCON; 
       
       C0 = zeros(W,L);
       if size(DCON_t,1)==0 || size(CON_t,1)==0
           C(:,:,k) = 1;
       else
           for i=1:L
               for j=1:W
                  
                   dist_con = sqrt(sum((CON_t-[j;i]).^2));
                   dist_dcon = sqrt(sum((DCON_t-[j;i]).^2));
                   [mdist_con,IR] = min(dist_con);   % 方格离R的距离
                   [mdist_dcon,IQ] = min(dist_dcon); % 方格离Q的距离
                   
                   %disp(CON_t(:,IR))
                   %disp(DCON_t(:,IQ))
                   
                   
                   
                   if mdist_con==0
                       C(j,i,k) = 1;            % 奖励离R近的方格，让他更近
                   elseif mdist_dcon==0
                       C(j,i,k) = 1.3;          % 惩罚离Q近的方格，让他更远
                   else
                       C(j,i,k) = 0.3*mdist_con/(mdist_con+mdist_dcon)+1;
                   end                   
               end
           end
       %C(:,:,k) = C0;
       end
    end
    %% recalcuate S
    for k=1:robot_num
        DCON_t = rolesArray(k).DCON; 
        rolesArray(k).taskAmount = rolesArray(k).taskAmount - size(DCON_t,1); % 减去非连通区域的数量
    end
    
    
    %% update alpha
    
    dm = [rolesArray.taskAmount] - (F-ob)/robot_num; % 任务量与平均任务量的偏差
    threshold = W*L/30;                              % 判断阈值
    if iteration_count>100
        threshold = threshold*(iteration_count/50)^2; % 阈值增大
    end
    for k=1:robot_num
       if abs(dm(k)) > threshold    % 偏差超过阈值
          alpha(k) = alpha(k)+0.002*dm(k); % 这个0.002就是论文里公式（11）的小写c
       else
           count = count + 1; % 这个机器人的任务区域满足要求
       end
    end
    
    stop1 = 0;
    if count == robot_num  % 所有机器人的任务区域都满足了要求
        stop1 = 1;
    end
    stop2 = 1;
    for k=1:robot_num
        DCON_t = rolesArray(k).DCON; 
        if size(DCON_t,1)~=0 % 还有非连通区域没搞完
            stop2 = 0;
            continue;
        end
    end
    if (stop1==1 && stop2==1) || iteration_count==300
        stop = 1;
    end
    
  if mod(iteration_count,50) == 0
      disp(iteration_count);
  end
    %K
    %dm
    %count

end
%K = K/10;
%colormap([0 0 0;rand(robot_num,3)]);
%pcolor(1:L,1:W,K);
figure(2)
handle2 = display_area(Area_rst,robot_num,2);

end

