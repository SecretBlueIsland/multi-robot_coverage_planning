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
   dir = [Robot.coord(2);Robot.coord(1)] - curr_p; % ������λ�ú�Ŀ��λ�õ�������
   if dir(1)==0 && dir(2)==0  % ͬһλ��
       bool = 1; % ��ͨ
       return % ���ü���������
   end
   dir = dir/sqrt(sum(dir.^2)); % ��һ��
   
   dir_cost = sqrt(sum((coordinate - dir).^2));
   if pace~=0
      dir_cost(pace) = dir_cost(pace) + 2; % �ϴ��ƶ��˵ģ����ȼ��ź�
   end
   [~,dir_i] = sort(dir_cost); % ��С��������
   sorted_cd = zeros(2,4);
   for i=1:4
       sorted_cd(:,i) = coordinate(:,dir_i(i)); % ���ȼ�
   end
   %% choose the nearest approachable way
   move = 0;
   for sd=sorted_cd
       next_p = curr_p + sd; % ����
       if next_p(1)<1||next_p(1)>size(Robot.taskDetail,1)||next_p(2)<1||next_p(2)>size(Robot.taskDetail,2)
           continue; % ������ͼ��Χ
       end
       if Robot.taskDetail(next_p(1),next_p(2))==1 % ���Լ��ĵ���
           curr_p = next_p; % �ƶ����õ�
           move = 1;
           if sd(1)==1
               pace = 4;  % ��¼�ƶ�����
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
   if move==0 % �޷��ж�
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
