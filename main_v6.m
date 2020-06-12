ROBOT_NUM = 4;
LENGTH = 20; %must be even

rng(40 ,'twister');%24 17
%4-20: 36 32 21 20? 18? 16 15 9? 8?
%4-40: 8? 9? 15? 16 20? 21? 32? 36

%% generate matrix
%core = unifrnd(0,1,[LENGTH/2,LENGTH/2]);
%den = 0.05;
%core(core>den) = 1;
%core(core<den) = 0;
core = ones(LENGTH/2,LENGTH/2);
kn = ones(2,2);
area = kron(core,kn);

%% generate swarm
init_pos = unifrnd(1,LENGTH,[ROBOT_NUM,2]);
init_grid = round(init_pos/2);
%figure(1);
%handle1 = display_area(area,1,1);
%pause()
rolesArray = [
              Robot('Jenny',init_grid(1,:)), ...
              Robot('Storm',init_grid(2,:)), ...
              Robot('Tom',init_grid(3,:)), ...
              Robot('Sky',init_grid(4,:))
             ];

%% divide area
A_rst = divide_area_v6(core, rolesArray);
Area_rst = kron(A_rst,kn);
figure(2);
handle2 = display_area(Area_rst,ROBOT_NUM,2);
for i=1:ROBOT_NUM
    rectangle(gca,'Position',[init_pos(i,1)-0.25,init_pos(i,2)-0.25,0.5,0.5],'Curvature',1,'EdgeColor','k');
end
