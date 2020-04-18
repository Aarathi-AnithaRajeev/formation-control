clear all
clc

%Dynamics of leader
x0=[0;0];
v0=[1;1];
%Dynamics of agents
xi=randi([-2,2],1,8);
vi=randi([-2,2],1,8);

time=[0 10];
init=[x0;v0;xi';vi'];
[t,z]=ode23('form_control',time,init);

plot(z(:,5),z(:,6));
hold on

plot(z(:,7),z(:,8));
hold on

plot(z(:,9),z(:,10));
hold on

plot(z(:,11),z(:,12));
hold on

plot(z(:,1),z(:,2));
hold on

title('FORMATION CONTROL with desired position offsets')
xlabel('x-axis')
ylabel('y-axis')
legend('1','2','3','4','5')

function [z_dot]=form_control(t,z1)

c=2;            %Coupling Gain
Kp=eye(2);      %Proportional Gain
gamma=100;
Kd=gamma*eye(2);%Derivative Gain
K=[Kp;Kd];

A=[0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0];
B=[0 0;0 0;1 0;0 1]';
G=0.5*[0 0 0 0;0 1 0 0;0 0 0 0;0 0 0 0];

%Position Offsets as the 4 corners of a square around the leader
delta=[1 0;1 0;1 0;-1 0;-1 0;-1 0;-1 0;1 0];
%Adjacency Matrix
A1=0.5*[0 0 1 0;1 0 0 0;1 1 0 0;0 1 0 0];  
%Diagonal Matrix
D=0.5*[1 0 0 0;0 1 0 0;0 0 2 0;0 0 0 1];
%Laplacian Matrix
L=D-A1;   

z1_dot=((kron(eye(2),A)-(c*(kron((L+G),B*K))))*[z1(5:12,:) z1(13:20,:)])+((c*(kron((L+G),B*K)))*[z1(1:2,:) z1(3:4,:); z1(1:2,:) z1(3:4,:); z1(1:2,:) z1(3:4,:); z1(1:2,:) z1(3:4,:)])+((c*(kron((L+G),B*K)))*delta);
z_tilde_dot=A*z1(1:4,:);
z_dot=[z_tilde_dot;z1_dot(:,1);z1_dot(:,2)];

end
