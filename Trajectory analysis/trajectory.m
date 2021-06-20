%D-H parameter of UR10 
%Creat The Arm  Using Peter Corke robotics toolbox

a = [0, -0.612, -0.5723, 0, 0, 0];
d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922];
alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0];
offset= [0, -pi/2, 0,-pi/2, 0, 0];

for i= 1:6
    L(i) = Link([ 0 d(i) a(i) alpha(i) 0 offset(i)], 'standard');    
end

UR = SerialLink(L);
UR.name = 'UR10';

%trajectory 
q0 = [0 0 0 0 0 0];                                     % initial joint value
q1 = [1.2331 1.0943 -1.0968 0.0025 -1.2331 -1.5708];    % pick item 1 from rack
q2 = [0.7952 1.0807 -0.9202 -0.1605 -0.7952 -1.5708];   % pick item 2 from rack
q3 = [-0.1371 0.1179 1.4212 0.0317 -1.5708 -1.7079];    % leave item 1 or 2 to conveyor
q4 = [1.3795 0.1451 1.4426 -0.0169 -1.5708 -1.7079];     % pick item 1 or 2 from conveyor 
q5 = [-0.2204 0.2280 0.7602 0.5827 -1.5708 -1.7912];    % leave item1 or 2 to table 
t = 0:0.15:3;
Q = jtraj(q0,q5,t);
Tr = fkine(UR,Q);

for i =1:1:length(t)
    T = Tr(i);
    trs = transl(T);
    xx(i) = trs(1);
    yy(i) = trs(2);
    zz(i) = trs(3);
end

plot(UR,Q);
hold on
plot3(xx,yy,zz,'Color',[1 0 0],'LineWidth',2);

