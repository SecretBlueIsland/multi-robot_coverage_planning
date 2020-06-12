function [ Area_rst ] = divide_area_v6( Core_flag, rolesArray )

robot_num = length(rolesArray);

% Area_flag:L*W (L:length of SOI, W:width of SOI, Area_flag(i,j)=1 means free space/0 means obstacles)
% init_pos:robot_num*2
% Area_rst:L*W*robot_num (Area_rst(:,:,i) means the area for robot i)
%% idea
% add a connectivity judge function
% original method: DARP-103��
%% prepare control points
Core_flag = Core_flag-1; %Area_flag(i,j)=0 means free space/-1 means obstacles
L = size(Core_flag,2); %length = cols = x
W = size(Core_flag,1); %width = rols = y

initK = zeros(W,L);
%display_area(Area_flag,1);
%���Ƴ�ʼ����������
for i=1:L
    for j=1:W
        E = zeros(1,robot_num);
        for k=1:robot_num
            E(k) = norm([i,j] - rolesArray(k).coord);
        end
        [~,index] = min(E);
        initK(j,i) = index;  % �ڵ�ͼ�ϱ�ǻ����˱��       
    end
end
figure(2);
display_area(initK,robot_num,2);
pause();

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
        rolesArray(each).taskAmount = 0; % ��������
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
            
            K(j,i) = index; % �ڵ�ͼ�ϱ�ǻ����˱��
            rolesArray(index).taskAmount = rolesArray(index).taskAmount +1; % ��������1
        end
    end
    
    %% show image
    Area_rst = K;
    figure(2);
    handle2 = display_area(Area_rst,robot_num,2);
    
    %% update c
    % obtain the assignment matrix for every robot    
    for k=1:robot_num
       K_r = zeros(W,L);
       K_t = K - k;
       for i=1:L
           for j=1:W
               if K_t(j,i)==0
                   K_r(j,i) = 1;
               else
                   K_r(j,i) = 0;
               end
           end
       end
       eval(['K',num2str(k),'=','K_r',';']); % ÿ�������˵�����
    end
    % obtain the connected sets for every robot
    for k=1:robot_num
        CON_t = [];        
        DCON_t = [];
        eval(['K_t','=','K',num2str(k),';']);
        for i=1:L
            for j=1:W
                if K(j,i)~=k
                   continue; 
                end
                if isconnect(j,i,K_t,(rolesArray(k).coord)')==1
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
        eval(['CON',num2str(k),'=','CON_t',';']);
        eval(['DCON',num2str(k),'=','DCON_t',';']);
    end
    
    for k=1:robot_num
       eval(['CON_t','=','CON',num2str(k),';']);
       eval(['DCON_t','=','DCON',num2str(k),';']);
       C0 = zeros(W,L);
       if size(DCON_t,1)==0 || size(CON_t,1)==0
           C(:,:,k) = 1;
       else
           for i=1:L
               for j=1:W
                  
                   dist_con = sqrt(sum((CON_t-[j;i]).^2));
                   dist_dcon = sqrt(sum((DCON_t-[j;i]).^2));
                   mdist_con = min(dist_con);
                   mdist_dcon = min(dist_dcon);
                   if mdist_con==0
                       C(j,i,k) = 1;
                   elseif mdist_dcon==0
                       C(j,i,k) = 1.3;
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
        eval(['DCON_t','=','DCON',num2str(k),';']);
        rolesArray(k).taskAmount = rolesArray(k).taskAmount - size(DCON_t,1);
    end
    
    
    %% update alpha
    %dm = zeros(1,robot_num);
    dm = [rolesArray.taskAmount] - (F-ob)/robot_num;
    threshold = W*L/30;
    if iteration_count>100
        threshold = threshold*(iteration_count/50)^2;
    end
    for k=1:robot_num
       if dm(k) > threshold || dm(k) < -threshold
          alpha(k) = alpha(k)+0.002*dm(k);
       else
           count = count + 1;
       end
    end
    
    stop1 = 0;
    if count == robot_num
        stop1 = 1;
    end
    stop2 = 1;
    for k=1:robot_num
        eval(['DCON_t','=','DCON',num2str(k),';']);
        if size(DCON_t,1)~=0
            stop2 = 0;
            continue;
        end
    end
    if (stop1==1 && stop2==1) || iteration_count==300
        stop = 1;
    end
    
    
    %K
    %dm
    %count

end
%K = K/10;
%colormap([0 0 0;rand(robot_num,3)]);
%pcolor(1:L,1:W,K);


end

