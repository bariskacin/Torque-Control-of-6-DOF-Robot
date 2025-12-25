tool_rpy_deg  = [180 0 0];
tool_offset_m = [0 0 0.10];     

tree = rigidBodyTree('DataFormat','column','MaxNumBodies',8);
tree.Gravity = [0 0 -9.81];

dh = [0.150   pi/2    0.445   0;
      0.760   0       0       0;
      0.150  -pi/2    0       0;
      0       pi/2    0.855   0;
      0      -pi/2    0       0;
      0       0       0.100   0];

m = [25 18 12 8 4 2];

r = 0.05;

for i = 1:6
    L = max(abs(dh(i,1)), abs(dh(i,3)));
    if L < 0.10, L = 0.10; end

    Ixx = (1/12)*m(i)*(3*r^2 + L^2);
    Iyy = Ixx;
    Izz = 0.5*m(i)*r^2;

    body = rigidBody(sprintf('body%d',i));
    jnt  = rigidBodyJoint(sprintf('jnt%d',i),'revolute');
    setFixedTransform(jnt, dh(i,:), 'dh');

    jnt.PositionLimits = deg2rad([-180 180]);

    body.Joint = jnt;

    body.Mass = m(i);

    body.CenterOfMass = [dh(i,1)/2  0  dh(i,3)/2];

    body.Inertia = [Ixx Iyy Izz 0 0 0];

    if i == 1
        addBody(tree, body, tree.BaseName);
    else
        addBody(tree, body, sprintf('body%d',i-1));
    end
end

tool = rigidBody('tool');
fixJ = rigidBodyJoint('fixTool','fixed');

rpy = deg2rad(tool_rpy_deg);
Rtool = eul2rotm([rpy(3) rpy(2) rpy(1)], 'ZYX');  % yaw,pitch,roll
Ttool = rotm2tform(Rtool) * trvec2tform(tool_offset_m);

setFixedTransform(fixJ, Ttool);
tool.Joint = fixJ;
addBody(tree, tool, 'body6');

assignin('base','robot_model',tree);
