%--------------------Pendulum parameters----------
L=(72/2)/1000;      %Length to Pendulum's Center of Mass (m)
m=11/1000;          %Mass of Pendulum Arm                (kg)
r=50/1000;          %Rotating Arm Length                 (m)
g=9.81;             %gravity acceleration    (m/s^2)

%-----------------Motor parameters-----------
K_t=0.114;          %Motor Torque Constant 
K_m= 0.116;         %Back EMF Constant 
R_m=7.5;            %Armature Resistance 
K_g=110 ;           %motor system gear ratio (motor->load) 
n_m=0.69;           %Motor efficiency 
n_g=0.9;            %Gearbox efficiency 
B_eq =6.3568e-5;    %Equivalent viscous damping coefficient 
J_eq=0.2760;        %Equivalent moment of inertia at the load

%---------------------some dummy variables ----------------------
a=J_eq+m*r*r;
b=m*L*r;
c=(4/3)*m*L*L;
d=m*g*L;
E=a*c-b*b;
H=(n_m*n_g*K_t*K_g)/(R_m*E);
G=((n_m*n_g*K_t*K_m*K_g*K_g)-B_eq*R_m)/R_m;

%-----------------------State space of system-----------
A=[0 0 1 0;0 0 0 1;0 (b*d)/E -(c*G)/E 0;0 (a*d)/E -(b*G)/E 0];   %system matrix
B=[0;0;c*H;b*H];                                                 %input  matrix
C=[1 0 0 0;0 1 0 0];                                             %output matrix
D=[0;0];                                                         %feedforward matrix
ss_model=ss(A,B,C,D);                                            %state-space model

%----------------------LQR parameters-----------------
Q=diag([0.1 800 700 400]);       % state matrix [theta alpha thetadot alphadot]
R = 1;                           %control effort weights
k=lqr(ss_model,Q,R);             %optimum pole placement gain;
k_theta=k(1);
k_alpha=k(2);
k_thetadot=k(3);
k_alphadot=k(4);
syslqr=ss(A-B*k,B,C,0);

%Step response
TF = tf(syslqr)
step(TF)