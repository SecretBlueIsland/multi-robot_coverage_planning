classdef Robot
    properties
        name         % 机器人名称
        coord        % 机器人位置
        taskAmount   % 机器人的任务量（大栅格数量）
    end
    
    methods
        function obj = Robot(name, coord)
            obj.name = name;
            obj.coord = coord;
            obj.taskAmount = 0; % 任务量初始化为0
        end
                
        function say(obj)
            fprintf('机器人%s就位\n',obj.name);
        end
    end
end

