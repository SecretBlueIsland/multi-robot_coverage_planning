 function [ bool ] = isconnect_v6( j0,i0,Robot )

% j-W,i-L
% K=1 represents assigned,0 represents unassigned
% remeber the iteration journey and increase the cost of turning back
pace = 0;
coordinate = [0 1 0 -1;-1 0 1 0];% 1-up 2-right 3-down 4-left
curr_p = [j0;i0];
count = 0;
while 1
   %% see if the condition were met
   dir = [Robot.coord(2);Robot.coord(1)] - curr_p; % 机器人位置和目标位置的向量差
   if dir(1)==0 && dir(2)==0  % 同一位置
       bool = 1; % 连通
       return % 不用继续计算了
   end
   dir = dir/sqrt(sum(dir.^2)); % 归一化
   
   dir_cost = sqrt(sum((coordinate - dir).^2));
   if pace~=0
      dir_cost(pace) = dir_cost(pace) + 2; % 上次移动了的，优先级放后
   end
   [~,dir_i] = sort(dir_cost); % 由小到大排列
   sorted_cd = zeros(2,4);
   for i=1:4
       sorted_cd(:,i) = coordinate(:,dir_i(i)); % 优先级
   end
   %% choose the nearest approachable way
   move = 0;
   for sd=sorted_cd
       next_p = curr_p + sd; % 靠近
       if next_p(1)<1||next_p(1)>size(Robot.taskDetail,1)||next_p(2)<1||next_p(2)>size(Robot.taskDetail,2)
           continue; % 超出地图范围
       end
       if Robot.taskDetail(next_p(1),next_p(2))==1 % 是自己的地盘
           curr_p = next_p; % 移动到该点
           move = 1;
           if sd(1)==1
               pace = 4;  % 记录移动方向
           elseif sd(1)==-1
               pace = 2;
           elseif sd(2)==-1
               pace = 3;
           else
               pace = 1;
           end
           break;
       end
   end
   if move==0 % 无法行动
       bool = 0;
       return
   else
       count = count+1;
       if count==size(Robot.taskDetail,1)*size(Robot.taskDetail,2)
           bool = 0;
           return
       end
   end
end

end
