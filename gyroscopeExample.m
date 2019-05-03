%% Setup %%
clear; clc; close all; addpath Functions
global posi currentPointPosi posj currentPointPosj posk currentPointPosk angveli currentPointVeli angvelj currentPointVelj angvelk currentPointVelk angacci currentPointAcci angaccj currentPointAccj angacck currentPointAcck
%% User Inputs %%
gridScale = 3.1; % Scale of the axis to animate. The scale of shape generation may differ
precession = 10; % Period of one precession rotation
nutationMidangle = 60; % Starting nutation angle
nutationMaxvariation = 10; % Variance of the nutation angle
nutationPeriod = 5; % Period of a sin wave nutation cycle
spin = 4; % Period of a spin rotation
Tmax = precession; % Animate for 1 rotation
pointX = 0; pointY = 1.01; pointZ = 1; % Starting co-ordinates of the point to measure
spinningTopMass = 2.5; % Mass of the spinning top. Used when calculating inertia
animationSpeed = 1; % Controls the speed of animation. Useful with high or low speed motion
framerateLimit = 8; % FPS - Lower this number if the graphs have added noise
scale = 2.1; % The scale of the static shape. The shape can move outside of these limits but can't be generated outisde of them. This value should be kept as low as possible to increase the resolution of the shape
resolution = 101; % Number of points along each axis between -scale and +scale. Total number of points is resolution^3 so slows down computation with large increases
%% Shape Generation %%
[x,y,z]=defineAxis(scale,resolution);
bottom1 = cone(0.3,[0,0,0],[0,0,0.9],x,y,z);
bottom2 = cone(0.5,[0,0,0.3],[0,0,0.9],x,y,z);
bottom3 = cone(0.7,[0,0,0.5],[0,0,0.9],x,y,z);
bottom4 = cone(0.9,[0,0,0.7],[0,0,0.9],x,y,z);
top1 = cone(0.9,[0,0,1.5],[0,0,1],x,y,z);
top2 = cone(0.7,[0,0,1.75],[0,0,1],x,y,z);
top3 = cone(0.5,[0,0,2],[0,0,1],x,y,z);
top4 = cone(0.3,[0,0,2.5],[0,0,1],x,y,z);
top5 = cone(0.1,[0,0,3],[0,0,1],x,y,z);
topsum1=addShapes(top1,top2,1);
topsum2=addShapes(topsum1,top3,1);
topsum3=addShapes(topsum2,top4,1);
topsum4=addShapes(topsum3,top5,1);
bottomsum1=addShapes(bottom1,bottom2,1);
bottomsum2=addShapes(bottomsum1,bottom3,1);
bottomsum3=addShapes(bottomsum2,bottom4,1);
shape=addShapes(topsum4,bottomsum3,1);
disk = cylinder(1,[0,0,0.9],0.1,x,y,z);
disk2 = cylinder(0.2,[0,0,0],3,x,y,z);
shape=addShapes(shape,disk,1);
% shape = shape + vectorCylinder(0.5,[0.5,0,0.9],[0.5,0,1.1],x,y,z);
[xxx,yyy,zzz,f] = generateShape(shape,x,y,z);
xxxstatic=xxx;yyystatic=yyy;zzzstatic=zzz;
pointXstatic=pointX;pointYstatic=pointY;pointZstatic=pointZ;
%% Shape Info %%
[X,Y,Z]=massCentre(shape,x,y,z);
[Matrix,Mass] = inertiaSetup(shape,x,y,z);
%% Position Components %%
[xxx,yyy,zzz]=rotateAboutVector([xxx,yyy,zzz],(90-nutationMidangle),[1,0,0],[0,0,0]);
[X,Y,Z]=rotateAboutVector([X,Y,Z],(90-nutationMidangle),[1,0,0],[0,0,0]);
[pointX,pointY,pointZ]=rotateAboutVector([pointX,pointY,pointZ],(90-nutationMidangle),[1,0,0],[0,0,0]);
[Matrix(:,1),Matrix(:,2),Matrix(:,3)]=rotateAboutVector([Matrix(:,1),Matrix(:,2),Matrix(:,3)],(90-nutationMidangle),[1,0,0],[0,0,0]);
%% Plotting Figures And Formatting %%
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,3,1); xlabel('x axis'); ylabel('y axis'); zlabel('z axis'); hold on
patch('Faces',f,'Vertices',[xxxstatic,yyystatic,zzzstatic],'EdgeColor','none','FaceColor','blue');
scatter3(pointXstatic,pointYstatic,pointZstatic,200,'go','filled')
camlight; axis vis3d; grid on; daspect([1 1 1]); view([1 1 1])
axis([-1*gridScale 1*gridScale -1*gridScale 1*gridScale -0.5*gridScale 1.5*gridScale])
subplot(2,3,3);alpha(0.3); xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
patch1 = patch('Faces',f,'Vertices',[xxx yyy zzz],'EdgeColor','none','FaceColor','blue','HandleVisibility','off');
points = animatedline(pointX,pointY,pointZ,'Color','g','LineWidth',3);
inertiax = animatedline(0,0,0,'Color','r','LineWidth',3);
inertiay = animatedline(0,0,0,'Color','g','LineWidth',3);
inertiaz = animatedline(0,0,0,'Color','b','LineWidth',3);
inertiaxm = animatedline(X,Y,Z,'Color','r','LineWidth',3);
inertiaym = animatedline(X,Y,Z,'Color','g','LineWidth',3);
inertiazm = animatedline(X,Y,Z,'Color','b','LineWidth',3);
camlight; axis vis3d; grid on; daspect([1 1 1]); view([1 1 1])
axis([-1*gridScale 1*gridScale -1*gridScale 1*gridScale -0.5*gridScale 1.5*gridScale])
subplot(2,3,2); xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
patch3 = patch('Faces',f,'Vertices',[xxx yyy zzz],'EdgeColor','none','FaceColor','blue');
camlight; axis vis3d; grid on; daspect([1 1 1]); view([1 1 1])
axis([-1*gridScale 1*gridScale -1*gridScale 1*gridScale -0.5*gridScale 1.5*gridScale])
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
smolDiskAngVel(1,:) = [0,0,0];
smolDiskAngVelRad(1,:) = [0,0,0];
point = zeros(3,1);
angacc = zeros(1,3);
i=1; currentAngle = 0;
t = 0; dt = 0; wn=0;
a=1; b=0; c=0;
%% Loop Setup %%
pausePlay = uicontrol; pausePlay.String = 'Pause'; pausePlay.Style = 'togglebutton';
pausePlay.Position = [20 100 150 50]; pausePlay.Value = 1;
quit = uicontrol; quit.String = 'Quit'; quit.Style = 'togglebutton';
quit.Position = [20 40 150 50]; quit.Value = 1;
maxFrames = ceil((Tmax/animationSpeed)*framerateLimit) + 1;
xxxi = zeros(numel(xxx),maxFrames); yyyi = zeros(numel(yyy),maxFrames);
zzzi = zeros(numel(zzz),maxFrames);
ti = zeros(1,maxFrames); Xi = zeros(numel(X),maxFrames);
Yi = zeros(numel(Y),maxFrames); Zi = zeros(numel(Z),maxFrames);
rI = zeros(maxFrames,6); rIXYZ = zeros(maxFrames,6);
angularAcceleration = zeros(3,maxFrames); angularVelocity = zeros(3,maxFrames);
while t <= Tmax
    tic;
    %% Plotting %%
    subplot(2,3,3); delete(patch1); hold on
    patch1 = patch('Faces',f,'Vertices',[xxx yyy zzz],'EdgeColor','none','FaceColor','blue');
    %addpoints(points,pointX,pointY,pointZ);
    if i > 1
        clearpoints(inertiax);clearpoints(inertiay);clearpoints(inertiaz);
        clearpoints(inertiaxm);clearpoints(inertiaym);clearpoints(inertiazm);
        addpoints(inertiax,0,0,0);addpoints(inertiay,0,0,0);addpoints(inertiaz,0,0,0);
        addpoints(inertiaxm,X,Y,Z);addpoints(inertiaym,X,Y,Z);addpoints(inertiazm,X,Y,Z);
        addpoints(inertiax,rI(i-1,1),0,0);addpoints(inertiay,0,rI(i-1,2),0);addpoints(inertiaz,0,0,rI(i-1,3));
        addpoints(inertiaxm,X+rIXYZ(i-1,1),Y,Z);addpoints(inertiaym,X,Y+rIXYZ(i-1,2),Z);addpoints(inertiazm,X,Y,Z+rIXYZ(i-1,3));
    end
    alpha(0.3)
    subplot(2,3,2); delete(patch3); hold on
    clearpoints(s3); addpoints(s3,pointX,pointY,pointZ)
    patch3 = patch('Faces',f,'Vertices',[xxx yyy zzz],'EdgeColor','none','FaceColor','blue');
    drawnow limitrate
    %% Motion %%
    Precession=(360*(dt/precession)); Spin=(360*(dt/spin)); Nutation=((nutationMaxvariation*sind(360*(t/nutationPeriod)))-(nutationMaxvariation*sind(360*((t-dt)/nutationPeriod))));
    % Precession
    [xxx,yyy,zzz]=rotateAboutVector([xxx,yyy,zzz],Precession,[0,0,1],[0,0,0]);
    [X,Y,Z]=rotateAboutVector([X,Y,Z],Precession,[0,0,1],[0,0,0]);
    [a,b,c]=rotateAboutVector([a,b,c],Precession,[0,0,1],[0,0,0]);
    [pointX,pointY,pointZ]=rotateAboutVector([pointX pointY pointZ],Precession,[0,0,1],[0,0,0]);
    [Matrix(:,1),Matrix(:,2),Matrix(:,3)]=rotateAboutVector([Matrix(:,1),Matrix(:,2),Matrix(:,3)],Precession,[0,0,1],[0,0,0]);
    % Nutation
    [xxx,yyy,zzz]=rotateAboutVector([xxx,yyy,zzz],Nutation,[a,b,c],[0,0,0]);
    [X,Y,Z]=rotateAboutVector([X,Y,Z],Nutation,[a,b,c],[0,0,0]);
    [pointX,pointY,pointZ]=rotateAboutVector([pointX pointY pointZ],Nutation,[a,b,c],[0,0,0]);
    [Matrix(:,1),Matrix(:,2),Matrix(:,3)]=rotateAboutVector([Matrix(:,1),Matrix(:,2),Matrix(:,3)],Nutation,[a,b,c],[0,0,0]);
    % Spin
    [xxx,yyy,zzz]=rotateAboutVector([xxx,yyy,zzz],Spin,[X,Y,Z],[0,0,0]);
    [pointX,pointY,pointZ]=rotateAboutVector([pointX pointY pointZ],Spin,[X,Y,Z],[0,0,0]);
    [Matrix(:,1),Matrix(:,2),Matrix(:,3)]=rotateAboutVector([Matrix(:,1),Matrix(:,2),Matrix(:,3)],Spin,[X,Y,Z],[0,0,0]);
    %% Inertia %%
    rI(i,:) = rotatedInertia(Matrix,0,0,0,[0,0,0],[0,0,0],spinningTopMass,Mass);
    rIXYZ(i,:) = rotatedInertia(Matrix,0,0,0,[0,0,0],[X,Y,Z],spinningTopMass,Mass);
        %% Further Data Calculation %%
    addpoints(posi,t,pointX); addpoints(posj,t,pointY); addpoints(posk,t,pointZ);
    point(1,i) = pointX; point(2,i) = pointY; point(3,i) = pointZ;
    xxxi(:,i) = xxx; yyyi(:,i) = yyy; zzzi(:,i) = zzz;
    Xi(i) = X; Yi(i) = Y; Zi(i) = Z; ti(i) = t;
    pwn=wn; wp=deg2rad(Precession); wn=deg2rad(Nutation); ws=deg2rad(Spin);
    vp=[0,0,1]; vn=[a,b,c]; vs=[X,Y,Z];
    nvp=vp/norm(vp); nvn=vn/norm(vn); nvs=vs/norm(vs);
    angularVelocity(:,i)=(wp*nvp+wn*nvn+ws*nvs)/dt;
    addpoints(angveli,t,angularVelocity(1,i)); addpoints(angvelj,t,angularVelocity(2,i)); addpoints(angvelk,t,angularVelocity(3,i));
    if i > 1
        angularAcceleration(:,i)=(angularVelocity(:,i)-angularVelocity(:,i-1))/dt;
        addpoints(angacci,t,angularAcceleration(1,i)); addpoints(angaccj,t,angularAcceleration(2,i)); addpoints(angacck,t,angularAcceleration(3,i));
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
max2 = max(angularVelocity(:)); min2 = min(angularVelocity(:)); axis([0 Tmax min2 max2]);
subplot(2,3,6); l3 = legend('i','j','k'); l3.ItemHitFcn = @clickableLegend;
max3 = max(angularAcceleration(:)); min3 = min(angularAcceleration(:)); axis([0 Tmax min3 max3]);
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
    subplot(2,3,3); delete(patch1);% delete(patch2); hold on
    patch1 = patch('Faces',f,'Vertices',[xxxi(:,i) yyyi(:,i) zzzi(:,i)],'EdgeColor','none','FaceColor','blue');
    %patch2 = patch('Faces',f2,'Vertices',[xxx2i(:,i) yyy2i(:,i) zzz2i(:,i)],'EdgeColor','none','FaceColor','red');
    alpha(0.3)
    subplot(2,3,2); delete(patch3);% delete(patch4); hold on
    clearpoints(s3); addpoints(s3,point(1,i),point(2,i),point(3,i))
    patch3 = patch('Faces',f,'Vertices',[xxxi(:,i) yyyi(:,i) zzzi(:,i)],'EdgeColor','none','FaceColor','blue');
    %patch4 = patch('Faces',f2,'Vertices',[xxx2i(:,i) yyy2i(:,i) zzz2i(:,i)],'EdgeColor','none','FaceColor','red');
    %% Move Points %%
    if i > 1
        clearpoints(inertiax);clearpoints(inertiay);clearpoints(inertiaz);
        clearpoints(inertiaxm);clearpoints(inertiaym);clearpoints(inertiazm);
        clearpoints(currentPointPosi);clearpoints(currentPointPosj);clearpoints(currentPointPosk);
        clearpoints(currentPointVeli);clearpoints(currentPointVelj);clearpoints(currentPointVelk);
        clearpoints(currentPointAcci);clearpoints(currentPointAccj);clearpoints(currentPointAcck);
        addpoints(inertiax,0,0,0);addpoints(inertiay,0,0,0);addpoints(inertiaz,0,0,0);
        addpoints(inertiaxm,Xi(i),Yi(i),Zi(i));addpoints(inertiaym,Xi(i),Yi(i),Zi(i));addpoints(inertiazm,Xi(i),Yi(i),Zi(i));
        addpoints(inertiax,rI(i-1,1),0,0);addpoints(inertiay,0,rI(i-1,2),0);addpoints(inertiaz,0,0,rI(i-1,3));
        addpoints(inertiaxm,Xi(i)+rIXYZ(i-1,1),Yi(i),Zi(i));addpoints(inertiaym,Xi(i),Yi(i)+rIXYZ(i-1,2),Zi(i));addpoints(inertiazm,Xi(i),Yi(i),Zi(i)+rIXYZ(i-1,3));
        addpoints(currentPointPosi,ti(i),point(1,i));addpoints(currentPointPosj,ti(i),point(2,i));addpoints(currentPointPosk,ti(i),point(3,i));
        addpoints(currentPointVeli,ti(i+1),angularVelocity(1,i+2));addpoints(currentPointVelj,ti(i+1),angularVelocity(2,i+2));addpoints(currentPointVelk,ti(i+1),angularVelocity(3,i+2));
        addpoints(currentPointAcci,ti(i+2),angularAcceleration(1,i+2));addpoints(currentPointAccj,ti(i+2),angularAcceleration(2,i+2));addpoints(currentPointAcck,ti(i+2),angularAcceleration(3,i+2));
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
        pause(((ti(i+1)-ti(i)))-to)
    end
end
close all
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