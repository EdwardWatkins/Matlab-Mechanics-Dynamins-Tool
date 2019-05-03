%% Setup %%
clear; clc; close all; addpath Functions
global posi currentPointPosi posj currentPointPosj posk currentPointPosk angveli currentPointVeli angvelj currentPointVelj angvelk currentPointVelk angacci currentPointAcci angaccj currentPointAccj angacck currentPointAcck
%% User Inputs %%
gridScale = 3; % Scale of the axis to animate. The scale of shape generation may differ
bigDiskSpeed = -rad2deg(1); % Big Disk angular rotation in degrees per second
Tmax = abs(360/bigDiskSpeed); % Animate for 1 rotation
pointX = 0; pointY = 1; pointZ = 0.25; % Starting co-ordinates of the point to measure
smallDiskRadius = 0.25; % Dimension of the small disk
smallDiskMountPointRadius = 1; % Used to link the speed of the two disks
smallDiskSpeed = -rad2deg(3);%(smallDiskMountPointRadius/smallDiskRadius)*bigDiskSpeed; % Small Disk angular rotation in degrees per second
smallDiskMass = 2.5; % Mass of the small disk. Used when calculating inertia
animationSpeed = 0.05; % Controls the speed of animation. Useful with high or low speeds
framerateLimit = 8; % FPS - Lower this number if the graphs have added noise
scale = 2.1; % The scale of the static shape. The shape can move outside of these limits but can't be generated outisde of them. This value should be kept as low as possible to increase the resolution of the shape
resolution = 101; % Number of points along each axis between -scale and +scale. Total number of points is resolution^3 so slows down computation with large increases
%% Shape Generation %%
[x,y,z] = defineAxis(scale,resolution);
bigDisk = cylinder(2,[0,0,-0.05],0.1,x,y,z);
polyhedronCoords = [0.9,0.3,0.1;0.9,0.3,-0.1;0.9,-0.3,0.1;0.9,-0.3,-0.1;1.1,0.3,0.1;1.1,0.3,-0.1;1.1,-0.3,0.1;1.1,-0.3,-0.1;];
polyhedronCoords = [polyhedronCoords(:,2),polyhedronCoords(:,1),polyhedronCoords(:,3)];
cutOut = polyhedron(polyhedronCoords,x,y,z);
bigDisk = addShapes(bigDisk,cutOut,-1);
[xxx1,yyy1,zzz1,f1] = generateShape(bigDisk,x,y,z);
smallDisk = vectorCylinder(smallDiskRadius,[0,0.95,0],[0,1.05,0],x,y,z);
[xxx2,yyy2,zzz2,f2] = generateShape(smallDisk,x,y,z);
%% Shape Info %%
inertiaBigDisk = momentOfInertia(bigDisk,1,x,y,z);
inertiaSmolDisk = momentOfInertia(smallDisk,1,x,y,z);
[X1,Y1,Z1] = massCentre(bigDisk,x,y,z);
[X2,Y2,Z2] = massCentre(smallDisk,x,y,z);
[Matrix,Mass] = inertiaSetup(smallDisk,x,y,z);
%% Plotting Figures And Formatting %%
f = figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,3,1); hold on
patch('Faces',f1,'Vertices',[xxx1 yyy1 zzz1],'EdgeColor','none','FaceColor','blue');
patch('Faces',f2,'Vertices',[xxx2 yyy2 zzz2],'EdgeColor','none','FaceColor','red');
scatter3(pointX,pointY,pointZ,200,'go','filled')
camlight; axis vis3d; grid on; daspect([1 1 1]); view([1 1 1])
axis([-1*gridScale 1*gridScale -1*gridScale 1*gridScale -1*gridScale 1*gridScale])
subplot(2,3,3);alpha(0.3)
patch1 = patch('Faces',f1,'Vertices',[xxx1 yyy1 zzz1],'EdgeColor','none','FaceColor','blue','HandleVisibility','off');
points = animatedline(pointX,pointY,pointZ,'Color','g','LineWidth',3);
patch2 = patch('Faces',f2,'Vertices',[xxx2 yyy2 zzz2],'EdgeColor','none','FaceColor','red','HandleVisibility','off');
inertiax = animatedline(0,0,0,'Color','r','LineWidth',3);
inertiay = animatedline(0,0,0,'Color','g','LineWidth',3);
inertiaz = animatedline(0,0,0,'Color','b','LineWidth',3);
inertiaxm = animatedline(X2,Y2,Z2,'Color','r','LineWidth',3);
inertiaym = animatedline(X2,Y2,Z2,'Color','g','LineWidth',3);
inertiazm = animatedline(X2,Y2,Z2,'Color','b','LineWidth',3);
camlight; axis vis3d; grid on; daspect([1 1 1]); view([1 1 1])
axis([-1*gridScale 1*gridScale -1*gridScale 1*gridScale -1*gridScale 1*gridScale])
subplot(2,3,2); xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
patch3 = patch('Faces',f1,'Vertices',[xxx1 yyy1 zzz1],'EdgeColor','none','FaceColor','blue');
patch4 = patch('Faces',f2,'Vertices',[xxx2 yyy2 zzz2],'EdgeColor','none','FaceColor','red');
camlight; axis vis3d; grid on; daspect([1 1 1]); view([1 1 1])
axis([-1*gridScale 1*gridScale -1*gridScale 1*gridScale -1*gridScale 1*gridScale])
s3 = animatedline(pointX,pointY,pointZ,'Color','g','Marker','o','MarkerFaceColor','g','MarkerSize',12);
subplot(2,3,4); title('Point Position'); xlabel('Time (s)')
posi = animatedline('Color','b'); posj = animatedline('Color','r'); posk = animatedline('Color','k');
axis([0 Tmax -inf inf]); grid on
subplot(2,3,5); title('Angular Velocity'); xlabel('Time (s)'); ylabel('rad/s')
angveli = animatedline('Color','b'); angvelj = animatedline('Color','r'); angvelk = animatedline('Color','k');
axis([0 Tmax -inf inf]); grid on
subplot(2,3,6); title('Angular Acceleration'); xlabel('Time (s)'); ylabel('rad/s^2')
angacci=animatedline('Color','b'); angaccj = animatedline('Color','r'); angacck = animatedline('Color','k');
axis([0 Tmax -inf inf]); grid on
%% Initial Inputs %%
smallDiskAngVel(1,:) = [0,0,0];
smolDiskAngVelRad(1,:) = [0,0,0];
point = zeros(3,1);
angacc = zeros(1,3);
i=1; currentAngle = 0;
t = 0; dt = 0;
%% Loop Setup %%
pausePlay = uicontrol; pausePlay.String = 'Pause'; pausePlay.Style = 'togglebutton';
pausePlay.Position = [20 100 150 50]; pausePlay.Value = 1;
quit = uicontrol; quit.String = 'Quit'; quit.Style = 'togglebutton';
quit.Position = [20 40 150 50]; quit.Value = 1;
maxFrames = ceil((Tmax/animationSpeed)*framerateLimit) + 1;
xxx1i = zeros(numel(xxx1),maxFrames); yyy1i = zeros(numel(yyy1),maxFrames);
zzz1i = zeros(numel(zzz1),maxFrames); xxx2i = zeros(numel(xxx2),maxFrames);
yyy2i = zeros(numel(yyy2),maxFrames); zzz2i = zeros(numel(zzz2),maxFrames);
ti = zeros(1,maxFrames); X2i = zeros(numel(X2),maxFrames);
Y2i = zeros(numel(Y2),maxFrames); Z2i = zeros(numel(Z2),maxFrames);
rI = zeros(maxFrames,6); rIXYZ = zeros(maxFrames,6);
%% Loop %%%
while t <= Tmax
    tic;
    %% Plotting %%
    subplot(2,3,3); delete(patch1); delete(patch2); hold on
    patch1 = patch('Faces',f1,'Vertices',[xxx1 yyy1 zzz1],'EdgeColor','none','FaceColor','blue');
    addpoints(points,pointX,pointY,pointZ);
    if i > 1
        clearpoints(inertiax);clearpoints(inertiay);clearpoints(inertiaz);
        clearpoints(inertiaxm);clearpoints(inertiaym);clearpoints(inertiazm);
        addpoints(inertiax,0,0,0);addpoints(inertiay,0,0,0);addpoints(inertiaz,0,0,0);
        addpoints(inertiaxm,X2,Y2,Z2);addpoints(inertiaym,X2,Y2,Z2);addpoints(inertiazm,X2,Y2,Z2);
        addpoints(inertiax,rI(i-1,1),0,0);addpoints(inertiay,0,rI(i-1,2),0);addpoints(inertiaz,0,0,rI(i-1,3));
        addpoints(inertiaxm,X2+rIXYZ(i-1,1),Y2,Z2);addpoints(inertiaym,X2,Y2+rIXYZ(i-1,2),Z2);addpoints(inertiazm,X2,Y2,Z2+rIXYZ(i-1,3));
    end
    patch2 = patch('Faces',f2,'Vertices',[xxx2 yyy2 zzz2],'EdgeColor','none','FaceColor','red');
    alpha(0.3)
    subplot(2,3,2); delete(patch3); delete(patch4); hold on
    clearpoints(s3); addpoints(s3,pointX,pointY,pointZ)
    patch3 = patch('Faces',f1,'Vertices',[xxx1 yyy1 zzz1],'EdgeColor','none','FaceColor','blue');
    patch4 = patch('Faces',f2,'Vertices',[xxx2 yyy2 zzz2],'EdgeColor','none','FaceColor','red');
    drawnow limitrate
    %% Motion %%
    bigDiskSpin = (dt*bigDiskSpeed);
    smallDiskSpin = (-dt*smallDiskSpeed);
    [xxx1,yyy1,zzz1] = rotateAboutAxis([xxx1,yyy1,zzz1],0,0,bigDiskSpin,[0,0,0]);
    xxx1i(:,i) = xxx1; yyy1i(:,i) = yyy1; zzz1i(:,i) = zzz1; ti(i) = t;
    [xxx2,yyy2,zzz2] = rotateAboutAxis([xxx2,yyy2,zzz2],0,0,bigDiskSpin,[0,0,0]);
    [X2,Y2,Z2] = rotateAboutAxis([X2,Y2,Z2],0,0,bigDiskSpin,[0,0,0]);
    [pointX,pointY,pointZ] = rotateAboutAxis([pointX,pointY,pointZ],0,0,bigDiskSpin,[0,0,0]);
    [xxx2,yyy2,zzz2] = rotateAboutVector([xxx2,yyy2,zzz2],smallDiskSpin,[X2,Y2,Z2],[0,0,0]);
    xxx2i(:,i) = xxx2; yyy2i(:,i) = yyy2; zzz2i(:,i) = zzz2;
    [pointX,pointY,pointZ] = rotateAboutVector([pointX,pointY,pointZ],smallDiskSpin,[X2,Y2,Z2],[0,0,0]);
    %% Inertia %%
    currentAngle = currentAngle + bigDiskSpin;
    rI(i,:) = rotatedInertia(Matrix,0,0,currentAngle,[0,0,0],[0,0,0],smallDiskMass,Mass);
    rIXYZ(i,:) = rotatedInertia(Matrix,0,0,currentAngle,[0,0,0],[X2,Y2,Z2],smallDiskMass,Mass);
    X2i(i) = X2; Y2i(i) = Y2; Z2i(i) = Z2;
    %% Further Data Calculation %%
    addpoints(posi,t,pointX); addpoints(posj,t,pointY); addpoints(posk,t,pointZ);
    point(1,i) = pointX; point(2,i) = pointY; point(3,i) = pointZ;
    pVel = smallDiskAngVel(i,:);
    smallDiskAngVel(i+1,:) = (bigDiskSpin*[0,0,1]+smallDiskSpin*[X2,Y2,Z2]/norm([X2,Y2,Z2]))/dt;
    smolDiskAngVelRad(i+1,:) = deg2rad(smallDiskAngVel(i+1,:));
    addpoints(angveli,t,smolDiskAngVelRad(i+1,1));addpoints(angvelj,t,smolDiskAngVelRad(i+1,2));addpoints(angvelk,t,smolDiskAngVelRad(i+1,3));
    if i > 1
        angacc(i,:) = (smolDiskAngVelRad(i+1,:)-deg2rad(pVel))*(1/dt);
        addpoints(angacci,t,angacc(i,1));addpoints(angaccj,t,angacc(i,2));addpoints(angacck,t,angacc(i,3));
    end
    %% Buttons %%
    if get(pausePlay,'Value') ~= 1
        pausePlay.String = 'Play';
        while true
            uicontrol(pausePlay);
            if get(pausePlay,'Value') == 1
                break
            end
            if get(quit,'Value') ~= 1
                break
            end
        end
    end
    pausePlay.String = 'Pause';
    uicontrol(pausePlay);
    if get(quit,'Value') ~= 1
        break
    end
    %% Time Control %%
    to=toc;
    if to < (1/framerateLimit)
        pause((1/framerateLimit)-to);
    end
    to = (1/framerateLimit);
    if i == 1
        to = 0;
    end
    dt = (to*animationSpeed);
    t = t + dt;
    i = i + 1;
end
%% Clickable Legend %%
subplot(2,3,4); l1 = legend('i','i','k'); l1.ItemHitFcn = @clickableLegend;
max1 = max(point(:)); min1 = min(point(:)); axis([0 Tmax min1 max1]);
subplot(2,3,5); l2 = legend('i','j','k'); l2.ItemHitFcn = @clickableLegend;
max2 = max(smolDiskAngVelRad(:)); min2 = min(smolDiskAngVelRad(:)); axis([0 Tmax min2 max2]);
subplot(2,3,6); l3 = legend('i','j','k'); l3.ItemHitFcn = @clickableLegend;
max3 = max(angacc(:)); min3 = min(angacc(:)); axis([0 Tmax min3 max3]);
%% Plotting Animated Points %%
subplot(2,3,4)
currentPointPosi = animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');
currentPointPosj = animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
currentPointPosk = animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
subplot(2,3,5)
currentPointVeli = animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');
currentPointVelj = animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
currentPointVelk = animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
subplot(2,3,6)
currentPointAcci = animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');
currentPointAccj = animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
currentPointAcck = animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
%% Loop Setup %%
for i = 3:numel(ti)
    if ti(i) == 0
        actualFrames = i-1;
        break
    end
    if i == numel(ti)
        actualFrames = maxFrames;
    end
end
i = 1;
while true
    tic;
    %% Animation %%
    subplot(2,3,3); delete(patch1); delete(patch2); hold on
    patch1 = patch('Faces',f1,'Vertices',[xxx1i(:,i) yyy1i(:,i) zzz1i(:,i)],'EdgeColor','none','FaceColor','blue');
    patch2 = patch('Faces',f2,'Vertices',[xxx2i(:,i) yyy2i(:,i) zzz2i(:,i)],'EdgeColor','none','FaceColor','red');
    alpha(0.3)
    subplot(2,3,2); delete(patch3); delete(patch4); hold on
    clearpoints(s3); addpoints(s3,point(1,i),point(2,i),point(3,i))
    patch3 = patch('Faces',f1,'Vertices',[xxx1i(:,i) yyy1i(:,i) zzz1i(:,i)],'EdgeColor','none','FaceColor','blue');
    patch4 = patch('Faces',f2,'Vertices',[xxx2i(:,i) yyy2i(:,i) zzz2i(:,i)],'EdgeColor','none','FaceColor','red');
    %% Move Points %%
    if i > 1
        clearpoints(inertiax);clearpoints(inertiay);clearpoints(inertiaz);
        clearpoints(inertiaxm);clearpoints(inertiaym);clearpoints(inertiazm);
        clearpoints(currentPointPosi);clearpoints(currentPointPosj);clearpoints(currentPointPosk);
        clearpoints(currentPointVeli);clearpoints(currentPointVelj);clearpoints(currentPointVelk);
        clearpoints(currentPointAcci);clearpoints(currentPointAccj);clearpoints(currentPointAcck);
        addpoints(inertiax,0,0,0);addpoints(inertiay,0,0,0);addpoints(inertiaz,0,0,0);
        addpoints(inertiaxm,X2i(i),Y2i(i),Z2i(i));addpoints(inertiaym,X2i(i),Y2i(i),Z2i(i));addpoints(inertiazm,X2i(i),Y2i(i),Z2i(i));
        addpoints(inertiax,rI(i-1,1),0,0);addpoints(inertiay,0,rI(i-1,2),0);addpoints(inertiaz,0,0,rI(i-1,3));
        addpoints(inertiaxm,X2i(i)+rIXYZ(i-1,1),Y2i(i),Z2i(i));addpoints(inertiaym,X2i(i),Y2i(i)+rIXYZ(i-1,2),Z2i(i));addpoints(inertiazm,X2i(i),Y2i(i),Z2i(i)+rIXYZ(i-1,3));
        addpoints(currentPointPosi,ti(i),point(1,i));addpoints(currentPointPosj,ti(i),point(2,i));addpoints(currentPointPosk,ti(i),point(3,i));
        addpoints(currentPointVeli,ti(i+1),smolDiskAngVelRad(i+2,1));addpoints(currentPointVelj,ti(i+1),smolDiskAngVelRad(i+2,2));addpoints(currentPointVelk,ti(i+1),smolDiskAngVelRad(i+2,3));
        addpoints(currentPointAcci,ti(i+2),angacc(i+2,1));addpoints(currentPointAccj,ti(i+2),angacc(i+2,2));addpoints(currentPointAcck,ti(i+2),angacc(i+2,3));
    end
    drawnow
    %% Buttons %%
    if get(pausePlay,'Value') ~= 1
        pausePlay.String = 'Play';
        while true
            uicontrol(pausePlay);
            if get(pausePlay,'Value') == 1
                break
            end
            if get(quit,'Value') ~= 1
                break
            end
        end
    end
    if get(quit,'Value') ~= 1
        break
    end
    pausePlay.String = 'Pause';
    uicontrol(pausePlay);
    i = i + 1;
    if i == actualFrames - 1
        i = 1;
    end
    %% Time Control %%
    to = toc;
    if to > (ti(i+1)-ti(i))
        pause((ti(i+1)-ti(i))-to)
    end
end
close all
pointVelocity=(point(:,3)-point(:,2))/dt;
pointAcceleration=(((point(:,3)-point(:,2))/dt)-((point(:,4)-point(:,3))/dt))/dt;
disp(pointVelocity)
disp(pointAcceleration)
%% Clickable Legend Function %%
function clickableLegend(~,evnt)
    global posi currentPointPosi posj currentPointPosj posk currentPointPosk angveli currentPointVeli angvelj currentPointVelj angvelk currentPointVelk angacci currentPointAcci angaccj currentPointAccj angacck currentPointAcck
    if strcmp(evnt.Peer.Visible,'on')
        evnt.Peer.Visible = 'off';
    else 
        evnt.Peer.Visible = 'on';
    end
    currentPointPosi.Visible = posi.Visible; currentPointPosj.Visible = posj.Visible; currentPointPosk.Visible = posk.Visible;
    currentPointVeli.Visible = angveli.Visible; currentPointVelj.Visible = angvelj.Visible; currentPointVelk.Visible = angvelk.Visible;
    currentPointAcci.Visible = angacci.Visible; currentPointAccj.Visible = angaccj.Visible; currentPointAcck.Visible = angacck.Visible;
end