classdef Robot
    properties
        name         % ����������
        coord        % ������λ��
        taskAmount   % �����˵�����������դ��������
    end
    
    methods
        function obj = Robot(name, coord)
            obj.name = name;
            obj.coord = coord;
            obj.taskAmount = 0; % ��������ʼ��Ϊ0
        end
                
        function say(obj)
            fprintf('������%s��λ\n',obj.name);
        end
    end
end

