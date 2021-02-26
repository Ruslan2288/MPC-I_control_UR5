clear;
cla;
%% main parametrs
p = 5;
m = 5;
n = 6;

I = eye(n);
O = zeros(n,n);

t = 1;
h = 0.06;
dt = h;
joints_vel = zeros(1,6);
joints_pos = zeros(1,6);
joints_pos_last = zeros(1,6);
Integral = zeros(1,p*n);


%% Ie calculate
Ie = I;
for i = 2:m 
    Ie = [Ie O];
end

%% Ia, Ib calculate
Ia = I;
Ib = Ia;
for i = 2:p  
   Ia = [Ia I]; 
   Ib = [Ib I.*i];   
end
Ib = Ib.*dt;

Ia = Ia';
Ib = Ib';
%% A calculate
A = zeros(n*p,n*m);
for i = 1:p
    for j = 1:m
        if(i<j)
            A(i*n-n+1:i*n, j*n+1-n:j*n) = O;       
        else
            A(i*n-n+1:i*n, j*n+1-n:j*n) = I.*(2.*i-2.*j+1)./2;
        end
    end
end
A = A.*dt^2;
%% Gw, Gq, Ki
one = ones(1, n);
Gq = eye(5*n).*[one.*150 one.*120 one.*90 one.*60 one.*30];
Gw = eye(5*n).*[one.*0.5 one.*0.4 one.*0.3 one.*0.2 one.*0.1];
Ki = eye(n*5).*0.003;

%%

H = A'*Gq'*Gq*A + Gw'*Gw;
Kmpc = Ie*inv(H)*A'*Gq'*Gq;
%% 
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


while clientID <=-1
    disp('Failed connecting to remote API server');
    pause(1);
    clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
end
% vrep.simxSynchronous(clientID, true);
disp('Connected to remote API server');
res = vrep.simxStartSimulation(clientID,  vrep.simx_opmode_oneshot);

joints=zeros(1,6);
for i = 1:6
    [res joints(i)] = vrep.simxGetObjectHandle(clientID,strcat('R',int2str(i)),vrep.simx_opmode_oneshot_wait);
     res = vrep.simxSetJointPosition(clientID, joints(i), 0, vrep.simx_opmode_oneshot);
    [res joints_pos(i)] = vrep.simxGetJointPosition(clientID, joints(i), vrep.simx_opmode_streaming);
     
end



%% trajectory generation
start_pos = [-0.385 0 0.9865];
button_pos = [0.62 0 0.5];
button_pos_prev = [0.5 0 0.5];
button_R = ROTZ(0)*ROTY(pi./2)*ROTX(0);

trajectory_x = linspace(button_pos_prev(1),button_pos(1),50);
trajectory_y = linspace(button_pos_prev(2),button_pos(2),50);
trajectory_z = linspace(button_pos_prev(3),button_pos(3),50);
trajectory_twist = linspace(0, pi/2, 50);


trajectory_1 = ones(30,3).*button_pos_prev;
trajectory_2 = [trajectory_x' trajectory_y' trajectory_z'];
trajectory_3 = ones(30,3).*button_pos;
trajectory_4 = flipud(trajectory_2);
trajectory_5 = trajectory_1;
trajectory = [trajectory_1; trajectory_2; trajectory_3; trajectory_4; trajectory_5];

%% Qd calculate
Qd = [];
Qd_1 = [];
len = length(trajectory);
for k = 1:len
    G = zeros(4,4);
    G(1:3, 1:3) = button_R;
    G(1:3, 4) = trajectory(k,:)';
    G(4, 4) = 1;
    Qd_1 = [Qd_1 calculate_pos(G)];
    Qd = [Qd calculate_pos(G)'];
end


% calculate_dynamic();
%% main loop
s = 1;
while t < (len-p)
        
    timeStart = vrep.simxGetLastCmdTime(clientID);
    for i = 1:n
        res = vrep.simxSetJointPosition(clientID, joints(i), Qd(i,t), vrep.simx_opmode_oneshot);          
        [res joints_pos(i)] = vrep.simxGetJointPosition(clientID, joints(i), vrep.simx_opmode_buffer);   
    end
    
    %calculate velocity
    joints_vel = (joints_pos - joints_pos_last)./dt;
    joints_pos_last = joints_pos;
    
    dQk = joints_vel';
    Qk = joints_pos';
    
    Qk_p = [];
    for a = 1:p
        Qk_p = [Qk_p Qk'];
    end
    Integral = Integral + (Qd_1(:,s:s+p*n-1) - Qk_p).*dt;
    Ek_p = Qd_1(:,s:s+p*n-1)' - Ia*Qk - Ib*dQk + (Integral*Ki)';
    w = Kmpc*Ek_p;
    
%     tau = get_M(Qk)*w + get_C(Qk,dQk)*dQk + get_G(Qk)';
%     for i = 1:n
%         res = vrep.simxSetJointForce(clientID, joints(i), tau(i), vrep.simx_opmode_oneshot);           
%     end

    s = s + n;
    t = t + 1;
    
   
    pause(h);
    dt = (vrep.simxGetLastCmdTime(clientID)-timeStart)./1000;

  
end
pause(3);
res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);

 function  Qd = calculate_pos(G)
    Qd = zeros(1,6);
    startingJoints = [pi/2, pi/2, 0, pi/2, 0, 0];   %UR5
    theta = ur5inv(G);
    for ti = 1:8
        Qd = theta(:,ti)'+startingJoints;
        for tj = 1:6
            if Qd(tj) > pi
                Qd(tj) = Qd(tj) - 2.*pi;
            elseif Qd(tj) < -pi
                Qd(tj) = Qd(tj) + 2.*pi;
            end
        end
    end
 end