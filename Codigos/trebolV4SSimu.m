clear
clc
load("trebolV4.mat");
plot(Posiciones(:,1),Posiciones(:,2))


l1=25;l2=25; %longitud de brazos en cm
% inputs
escalado=1.33; %1-1.33
rotacion=45;    %en ° +/- 45°

%escalamiento y rotacion-- devuelve angulos de articualciones q[rad]
escala=Posiciones*escalado;
rotado=rot(rotacion*pi/180)*escala';
traslado=rotado+[18;15];            % distancias (puntos) en cm recorridas en plano cartesiano
q=zeros(size(Posiciones));
for i=1:size(traslado,2)
    ang=inv2(traslado(1,i),traslado(2,i));
    q(i,1)=ang(1);
    q(i,2)=ang(2);
end
t=60;% tiempo estimado total para dibujo 
tr=t/size(Posiciones,1);
%%  Animacion Brazo robot
tvec=[0,tr];
P(1)=Link('revolute','d',0,'alpha',0,'a',l1,'qlim',pi/180*[-180 180]);
P(2)=Link('revolute','d',0,'alpha',0,'a',l2,'qlim',pi/180*[-180 180]);
rr=SerialLink(P,'name','RR');

for i=1:size(q,1)-1
    jtray1=jtraj(q(i,:),q(i+1,:),tvec);
    rr.plot(jtray1,'workspace',[-20 29 -1 25 -1 5],'view',[0,90],'trail','-')
end
%% 
angulos=zeros(size(Posiciones));
for i =1:size(q)
    angulos(i,1)=rad2deg(q(i,1));
    angulos(i,2)=rad2deg(q(i,2));
end
giro2brazo=180+angulos(:,2);
MinAng=min(giro2brazo)          %angulo minimo que debe poder girar brazo 2 respecto  abrazo 1

%%  Velocidades lineales y angulares de motores
qang=q;  %angulos en rad
q=traslado';  %traslado =posiciones en cm
rangos=size(Posiciones);
qdot=zeros(rangos(1)-1,rangos(2)); %velocidades 2-lo que de m/s

velang=zeros(rangos(1)-1,rangos(2));
for i=1:size(qang,1)-1
    vel1=(qang(i+1,1)-qang(i,1))/tr;      %rad/s
    vel2=(qang(i+1,2)-qang(i,2))/tr;      %rad/s
    velang(i,1)=vel1/(2*pi)*60;             %rpm
    velang(i,2)=vel2/(2*pi)*60;             %rpm
end
plot(velang)        %velocidades motores rad/s
for i=1:size(q,1)-1
    vel1=(q(i+1,1)-q(i,1))/tr;      %cm/s
    vel2=(q(i+1,2)-q(i,2))/tr;      %cm/s
    qdot(i,1)=vel1/100;             %m/s
    qdot(i,2)=vel2/100;             %m/s
end
%plot(qdot)         %velocidades 
qdot2=zeros(rangos(1)-2,rangos(2));
for i=1:size(qdot,1)-1 %accel  3-149  m/s^2
    acc1=(qdot(i+1,1)-qdot(i,1))/tr;
    acc2=(qdot(i+1,2)-qdot(i,2))/tr;
    qdot2(i,1)=acc1;
    qdot2(i,2)=acc2;
end 

%syms lc1 lc2 m1 m2 i1 i2
g=9.8;% m/s^2
m1=0.2; m2=0.085;   %kg
l1=0.25;l2=0.2; %m
lc1=l1/2;lc2=l2/2;

ibrazo1=210.076*10^-6; %kg m^2
isop1=0.243*10^-6; %kg m^2
isop2=13.515*10^-6; %kg m^2
imotor=1.879*10^-6; %kg m^2
ieje1=0.487*10^-6; %kg m^2
i1=ibrazo1*2+isop1*2+isop2+imotor+ieje1; %kg m^2

ibrazo2=161.402*10^-6; %kg m^2
isopbrazo2=0.085*10^-6; %kg m^2
ieje2=0.325*10^-6; %kg m^2
i2=ibrazo2*2+isopbrazo2*2+ieje2;    %kg m^2


function mat=rot(theta)
    mat=[cos(theta) -sin(theta);
        sin(theta) cos(theta)];
end 

function thetas=inv2(x,y)
    l1=25;l2=25;
    r=sqrt(x^2+y^2);
    theta=atan2(y,x);
    beta=acos((l1^2+r^2-l2^2)/(2*l1*r));
    qu1=theta+beta;
    qu2=acos((l1^2+l2^2-r^2)/(2*l1*l2))-pi;
    thetas=[qu1, qu2];
end
