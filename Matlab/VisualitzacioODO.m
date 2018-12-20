close all
clear all

DistanciaTOT = 9.801;  %distància total
f_valueX = 0;   %valors finals de l'odometria X i Y
f_valueY = 0;
vFrames=[250 500 550 20 100];    %frames els quals resetejarem l'odometria a [0,0]
%%%%%%%%%%%%%%%%%%%%%%%
Odo=load('Metros/Cercle3/A2B1C6D1E80.txt');
Odo=Odo';

Time = Odo(1,1)/1000  %treim el temps de l'arxiu de sortida
Odo(1,1) = 0;

% agafam el darrer valor de l'array i si el valor inicial hagues d'esser 0,0
% es lerror directament
[rowmax,colmax]= size(Odo);
ValX = Odo(1,colmax);
ValY = Odo(2,colmax);

%% error en X i Y
ErrorX = abs(f_valueX - ValX)
ErrorY = abs(f_valueY - ValY)

%% Error en distancia final de l'odometria
Dfinal = sqrt((f_valueX-ValX)^2+(f_valueY-ValY)^2)


%% Error en percentatge
EPercent = (100*Dfinal)/DistanciaTOT


%% Mostrar per pantalla l'odometria
Odo(3,:) =Odo(3,:)+pi/2;

if EPercent>0
aux_plot_trajectory(Odo,20);

axis equal
x = plot(Odo(1,1), Odo(2,1),'ro','MarkerFaceColor','g');
%% imprimir les posicions del robot durant el recorregut
for i=1:length(Odo)
    h = plot(Odo(1,i), Odo(2,i),'o','MarkerFaceColor','r'); 
    pause(0.03);
    delete(h);
end
x = plot(Odo(1,colmax), Odo(2,colmax),'ro','MarkerFaceColor','r');
end

%% dist'ancia total de l'odometria
SUMA = 0;
for i=1:length(Odo)-1
    a = i+1;
  DIS = sqrt((Odo(1,i)-Odo(1,a))^2+(Odo(2,i)-Odo(2,a))^2);  
  SUMA = SUMA + DIS;
end
SUMA
