%% Setup %%
clear; clc; close all; addpath Functions;
p=1.5; n=1.5;
global aniline1vel aniline2vel aniline1velpoint aniline2velpoint aniline1acc aniline2acc aniline1accpoint aniline2accpoint
%% User Inputs %%
animationSpeed = 0.5;
framerate = 8;
% Shaft 1 %
v1=[0,1,0];
p1=[0,0,0];
velocity1temp=3; % Times v1
% Shaft2 %
v2=[0,0,1];
p2=[1,3,-0.5];
% Rod length %
L=0.5*sqrt(21);
%% Calculate info about motion %%
v1=v1/norm(v1);
v2=v2/norm(v2);
c1=v1(1).^2+v1(2).^2+v1(3).^2;
c2=2*p1(1)*v1(1)-2*p2(1)*v1(1)+2*p1(2)*v1(2)-2*p2(2)*v1(2)+2*p1(3)*v1(3)-2*p2(3)*v1(3);
c3=-2*v2(1)*v1(1)-2*v2(2)*v1(2)-2*v2(3)*v1(3);
c4=-2*p1(1)*v2(1)+2*p2(1)*v2(1)-2*p1(2)*v2(2)+2*p2(2)*v2(2)-2*p1(3)*v2(3)+2*p2(3)*v2(3);
c5=v2(1).^2+v2(2).^2+v2(3).^2;
c6=p1(1).^2+2*p2(1)*p1(1)+p2(1).^2+p1(2).^2-2*p2(2)*p1(2)+p2(2).^2+p1(3).^2-2*p2(3)*p1(3)+p2(3).^2;
xm=((-2*c4*c3+4*c5*c2)+(sqrt((2*c4*c3-4*c5*c2)^2-(4*(c3^2-4*c5*c1)*(c4^2-4*c5*c6+4*c5*L^2)))))/(2*(c3^2-4*c5*c1));
Ox=(-2*c4*c3+4*c5*c2)/(2*c3^2-8*c5*c1);
Oy=(-2*c2*c3+4*c1*c4)/(2*c3^2-8*c1*c5);
R=sqrt((xm-Ox)^2);
[rmin,rmax,alphaMax,alphaMin,betaMax,betaMin,startAlpha,startBeta,perimiter,Ox,Oy,R]=linkInfo(v1,p1,v2,p2,L);
%% Make components %%
dBeta=betaMax-betaMin;
dAlpha=alphaMax-alphaMin;
if L > dBeta && L > dAlpha
    l=L;
elseif dAlpha > dBeta && dAlpha > L
    l=dAlpha;
else
    l=dBeta;
end
s=0.55*l;
mings=(0.6*l)/2.5;
if mings > 201
    gs=mings;
else
    gs=201;
end
[x,y,z]=defineAxis(s,gs);
shaft=cylinder(0.25,[0,0,-1.25*R],2.5*R,x,y,z);
[shaftxxxog,shaftyyyog,shaftzzzog,shaft1f] = generateShape(shaft,x,y,z);
links=3.6; linkgs=101;
[linkx,linky,linkz]=defineAxis(links,linkgs);
linkshape1=cylinder(0.35,[0,0,0],1,linkx,linky,linkz);
linkshape2=cylinder(0.25,[0,0,0],1,linkx,linky,linkz);
linkshape=addShapes(linkshape1,linkshape2,-1);
[link1xxx,link1yyy,link1zzz,link1f] = generateShape(linkshape,linkx,linky,linkz);
rodshape=cylinder(0.25,[0,0,0],L,x,y,z);
[rodxxx,rodyyy,rodzzz,rodf] = generateShape(rodshape,x,y,z);
%% Position components %%
link1=v1*startAlpha+p1;
link2=v2*startBeta+p2;
link1pos=link1;
link2pos=link2;
[shaft1xxx,shaft1yyy,shaft1zzz]=positionAlongVector([shaftxxxog,shaftyyyog,shaftzzzog],(Ox*v1+p1),v1);
[shaft2xxx,shaft2yyy,shaft2zzz]=positionAlongVector([shaftxxxog,shaftyyyog,shaftzzzog],(Oy*v2+p2),v2);
[link1xxxnew,link1yyynew,link1zzznew]=positionAlongVector([link1xxx,link1yyy,link1zzz],(startAlpha*v1+p1),v1);
[link2xxxnew,link2yyynew,link2zzznew]=positionAlongVector([link1xxx,link1yyy,link1zzz],(startBeta*v2+p2),v2);
[rodxxxtemp,rodyyytemp,rodzzztemp]=positionAlongVector([rodxxx,rodyyy,rodzzz],link1,(link2-link1));
%% Plot %%
f = figure('units','normalized','outerposition',[0 0 1 1]);%figure;
hold on
subplot(2,3,1);
xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
patch('Faces',shaft1f,'Vertices',[shaft1xxx,shaft1yyy,shaft1zzz],'EdgeColor','none','FaceColor','blue');
patch('Faces',shaft1f,'Vertices',[shaft2xxx,shaft2yyy,shaft2zzz],'EdgeColor','none','FaceColor','red');
l1=patch('Faces',link1f,'Vertices',[link1xxxnew,link1yyynew,link1zzznew],'EdgeColor','none','FaceColor','green');
l2=patch('Faces',link1f,'Vertices',[link2xxxnew,link2yyynew,link2zzznew],'EdgeColor','none','FaceColor','green');
r=patch('Faces',rodf,'Vertices',[rodxxxtemp,rodyyytemp,rodzzztemp],'EdgeColor','none','FaceColor','yellow');
camlight; axis vis3d; grid on
daspect([1 1 1]); view([1 1 1])
subplot(2,3,2); axis([alphaMin*1.2 alphaMax*1.2 betaMin*1.2 betaMax*1.2]); grid on; title('Link position'); xlabel('Position along shaft 1'); ylabel('Position along shaft 2')
aniline1pos=animatedline('Color','k');aniline1pospoint=animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
subplot(2,3,3); axis([0 360 -inf inf]); grid on; title('Relative velocity of link ends'); xlabel('Angle of the rod'); ylabel('Distance traveled along the shaft per degree of motion')
aniline1vel=animatedline('Color','b');aniline2vel=animatedline('Color','r');
aniline1velpoint=animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');aniline2velpoint=animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
subplot(2,3,4); axis([0 360 -inf inf]); grid on; title('Relative acceleration of link ends'); xlabel('Angle of the rod'); ylabel('Acceleration of link motion per degree of motion')
aniline1acc=animatedline('Color','b');aniline2acc=animatedline('Color','r');
aniline1accpoint=animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');aniline2accpoint=animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
subplot(2,3,5); axis([0 360 -20 20]); grid on; title('Angular velocity of rod'); xlabel('Angle of the rod'); ylabel('Angular velocity of rod')
wxplot=animatedline('Color','b');wxpoint=animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');
wyplot=animatedline('Color','r');wypoint=animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
wzplot=animatedline('Color','k');wzpoint=animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
subplot(2,3,6); axis([0 360 -10 10]); grid on; title('Angular Acceleration of rod'); xlabel('Angle of the rod'); ylabel('Angular velocity of rod')
axplot=animatedline('Color','b');axpoint=animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');
ayplot=animatedline('Color','r');aypoint=animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
azplot=animatedline('Color','k');azpoint=animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
%% Buttons %%
pausePlay = uicontrol; pausePlay.String = 'Pause'; pausePlay.Style = 'togglebutton';
pausePlay.Position = [20 100 150 50]; pausePlay.Value = 1;
quit = uicontrol; quit.String = 'Quit'; quit.Style = 'togglebutton';
quit.Position = [20 40 150 50]; quit.Value = 1;
%% Clickable Legend %%
subplot(2,3,2); legend1 = legend(); legend1.String='Position';legend1.ItemHitFcn = @clickableLegend;
subplot(2,3,3); legend2 = legend('Link 1','Link 2'); legend2.ItemHitFcn = @clickableLegend;
subplot(2,3,4); legend3 = legend('Link 1','Link 2'); legend3.ItemHitFcn = @clickableLegend;
%% Loop %%
theta=180+atand(0.5/2);
i = 1; q = 1;
velocity = zeros(2,200); acceleration = zeros(2,200);
for theta=(180+atand(0.5/2));
    tic;
    %% Motion %%
    if i > 1
        plink1 = v1*alpha+p1;
        plink2 = v2*beta+p2;
    end
    [alpha,beta,deltaTheta]=linkMotion(v1,p1,v2,p2,L,theta,[rmin,rmax],abs(perimiter)/((0.030)*(animationSpeed.^0.5)));
    theta=theta+deltaTheta;
    if theta >= 360
        theta = theta - 360;
        q = 0;
    end
    link1=v1*alpha+p1;
    link2=v2*beta+p2;
    [rodxxxtemp,rodyyytemp,rodzzztemp]=positionAlongVector([rodxxx,rodyyy,rodzzz],link1,(link2-link1));
    if i > 1
        plink1vel=(link1delta/(v1/norm(v1)));
        plink2vel=(link2delta/(v2/norm(v2)));
    end
    link1delta=link1-link1pos-0.5*v1;
    link2delta=link2-link2pos-0.5*v2;
    link1pos=link1pos+link1delta;
    link2pos=link2pos+link2delta;
    [link1xxxnew,link1yyynew,link1zzznew]=translateMatrix([link1xxxnew,link1yyynew,link1zzznew],link1delta(1),link1delta(2),link1delta(3));
    [link2xxxnew,link2yyynew,link2zzznew]=translateMatrix([link2xxxnew,link2yyynew,link2zzznew],link2delta(1),link2delta(2),link2delta(3));
    %% Velocity calculations %%
    if i > 1
        pwx=wx;pwy=wy;pwz=wz;
    end
    if link1delta < 0
        velocity1 = -velocity1temp;
    else
        velocity1 = velocity1temp;
    end
    [wx,wy,wz,velocity2]=linkSpeed(v1,v2,link1,link2,velocity1);
%     rdc = link2-link1;
%     A = [0,rdc(3),-rdc(2),-v2(1);
%         -rdc(3),0,rdc(1),-v2(2);
%         rdc(2),-rdc(1),0,-v2(3);
%         rdc(1),rdc(2),rdc(3),0;];
%     B = velocity1*[-v1(1);-v1(2);-v1(3);0;];
%     C = A\B;
%     wx = C(1);
%     wy = C(2);
%     wz = C(3);
%     velocity2 = C(4)*v2;
    %% Plotting %%
    delete(l1); delete(l2); delete(r)
    subplot(2,3,1)
    l1=patch('Faces',link1f,'Vertices',[link1xxxnew,link1yyynew,link1zzznew],'EdgeColor','none','FaceColor','green');
    l2=patch('Faces',link1f,'Vertices',[link2xxxnew,link2yyynew,link2zzznew],'EdgeColor','none','FaceColor','green');
    r=patch('Faces',rodf,'Vertices',[rodxxxtemp,rodyyytemp,rodzzztemp],'EdgeColor','none','FaceColor','yellow');
    subplot(2,3,2); clearpoints(aniline1pospoint);
    addpoints(aniline1pospoint,(link1pos/(v1/norm(v1))),(link2pos/(v2/norm(v2))));
    addpoints(aniline1pos,(link1pos/(v1/norm(v1))),(link2pos/(v2/norm(v2))));
    if i > 1
        subplot(2,3,3); clearpoints(aniline1velpoint); clearpoints(aniline2velpoint);
        addpoints(aniline1velpoint,theta,(link1delta/(v1/norm(v1))));addpoints(aniline2velpoint,theta,(link2delta/(v2/norm(v2))));
        link1vel=(link1delta/(v1/norm(v1)));
        link2vel=(link2delta/(v2/norm(v2)));
        if i > 2
            subplot(2,3,4); clearpoints(aniline1accpoint); clearpoints(aniline2accpoint);
            link1acc=(link1vel-plink1vel);
            link2acc=(link2vel-plink2vel);
            addpoints(aniline1accpoint,theta,link1acc);addpoints(aniline2accpoint,theta,link2acc);
        end
    end
    if q == 1
        if i > 1
            subplot(2,3,3);addpoints(aniline1vel,theta,(link1delta/(v1/norm(v1))));addpoints(aniline2vel,theta,(link2delta/(v2/norm(v2))));
            if i > 2
                subplot(2,3,4);addpoints(aniline1acc,theta,link1acc);addpoints(aniline2acc,theta,link2acc);
            end
        end
    end
    subplot(2,3,5)
    clearpoints(wxpoint);clearpoints(wypoint);clearpoints(wzpoint);
    addpoints(wxpoint,theta,wx);
    addpoints(wypoint,theta,wy);
    addpoints(wzpoint,theta,wz);
    if q == 1
        addpoints(wxplot,theta,wx);
        addpoints(wyplot,theta,wy);
        addpoints(wzplot,theta,wz);
    end
    if i > 1
        ax=pwx-wx;ay=pwy-wy;az=pwz-wz;
        subplot(2,3,6)
        clearpoints(axpoint);clearpoints(aypoint);clearpoints(azpoint);
        addpoints(axpoint,theta,ax);
        addpoints(aypoint,theta,ay);
        addpoints(azpoint,theta,az);
        if q == 1
            addpoints(axplot,theta,ax);
            addpoints(ayplot,theta,ay);
            addpoints(azplot,theta,az);
        end
    end
    drawnow
    if q == 0
        %minvel=min(velocity(:));minacc=min(angularAcceleration(:));
        subplot(2,3,3);axis([0 360 min(velocity(:)) max(velocity(:))])
        subplot(2,3,4);axis([0 360 min(acceleration(:)) max(acceleration(:))])
    elseif i > 3
        velocity(1,i) = (link1delta/(v1/norm(v1))); velocity(2,i) = (link2delta/(v2/norm(v2)));
        acceleration(1,i) = (link1vel-plink1vel); acceleration(2,i) = (link2vel-plink2vel);
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
    if to < (1/framerate)
        pause((1/framerate)-to);
    end
    dt = (1/framerate);
    i = i + 1;
end
% link1speed=link1-plink1;
% link2speed=link2-plink2;
% disp(link2speed(3)/link1speed(2))
% vector1 = plink2-plink1;
% vector2 = link2-link1;
disp(wx)
disp(wy)
disp(wz)
disp(velocity2)
close all
%% Clickable Legend Function %%
function clickableLegend(~,evnt)
    global aniline1vel aniline2vel aniline1velpoint aniline2velpoint aniline1acc aniline2acc aniline1accpoint aniline2accpoint
    if strcmp(evnt.Peer.Visible,'on')
        evnt.Peer.Visible = 'off';
    else 
        evnt.Peer.Visible = 'on';
    end
    aniline1velpoint.Visible = aniline1vel.Visible;
    aniline2velpoint.Visible = aniline2vel.Visible;
    aniline1accpoint.Visible = aniline1acc.Visible;
    aniline2accpoint.Visible = aniline2acc.Visible;
end