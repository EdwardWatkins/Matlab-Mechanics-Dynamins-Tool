clear; clc;
Load=1;
if Load == 1
    load 'Saved Animations/diskExample.mat'
elseif Load == 2
    load 'Saved Animations/gyroscopeExample.mat'
end
if ~exist('f1')
    f1=f;xxx1=xxx;yyy1=yyy;zzz1=zzz;xxx1i=xxxi;yyy1i=yyyi;zzz1i=zzzi;X2=X;Y2=Y;Z2=Z;X2i=Xi;Y2i=Yi;Z2i=Zi;smolDiskAngVelRad=angularVelocity';angacc=angularAcceleration';%ti=ti(1:(numel(ti)-1));
else
    smolDiskAngVelRad=smolDiskAngVelRad(2:end,:);
end
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,3,1); hold on
patch('Faces',f1,'Vertices',[xxx1 yyy1 zzz1],'EdgeColor','none','FaceColor','blue');
if exist('f2')
    patch('Faces',f2,'Vertices',[xxx2 yyy2 zzz2],'EdgeColor','none','FaceColor','red');
end
scatter3(pointX,pointY,pointZ,200,'go','filled')
camlight; axis vis3d; grid on; daspect([1 1 1]); view([1 1 1])
axis([-1*gridScale 1*gridScale -1*gridScale 1*gridScale -1*gridScale 1*gridScale])
subplot(2,3,3);alpha(0.3)
patch1 = patch('Faces',f1,'Vertices',[xxx1 yyy1 zzz1],'EdgeColor','none','FaceColor','blue','HandleVisibility','off');
points = animatedline(pointX,pointY,pointZ,'Color','g','LineWidth',3);
if exist('f2')
    patch2 = patch('Faces',f2,'Vertices',[xxx2 yyy2 zzz2],'EdgeColor','none','FaceColor','red','HandleVisibility','off');
end
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
if exist('f2')
    patch4 = patch('Faces',f2,'Vertices',[xxx2 yyy2 zzz2],'EdgeColor','none','FaceColor','red');
end
camlight; axis vis3d; grid on; daspect([1 1 1]); view([1 1 1])
axis([-1*gridScale 1*gridScale -1*gridScale 1*gridScale -1*gridScale 1*gridScale])
s3 = animatedline(pointX,pointY,pointZ,'Color','g','Marker','o','MarkerFaceColor','g','MarkerSize',12);
subplot(2,3,4); title('Point Position'); xlabel('Time (s)')
% posi = animatedline('Color','b'); posj = animatedline('Color','r'); posk = animatedline('Color','k');
axis([0 Tmax -inf inf]); grid on
subplot(2,3,5); title('Angular Velocity'); xlabel('Time (s)'); ylabel('rad/s')
% angveli = animatedline('Color','b'); angvelj = animatedline('Color','r'); angvelk = animatedline('Color','k');
axis([0 Tmax -inf inf]); grid on
subplot(2,3,6); title('Angular Acceleration'); xlabel('Time (s)'); ylabel('rad/s^2')
% angacci=animatedline('Color','b'); angaccj = animatedline('Color','r'); angacck = animatedline('Color','k');
axis([0 Tmax -inf inf]); grid on
subplot(2,3,4);hold on
currentPointPosi = animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');
currentPointPosj = animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
currentPointPosk = animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
plot(ti,point(1,:),'b');plot(ti,point(2,:),'r');plot(ti,point(3,:),'k');
subplot(2,3,5);hold on
currentPointVeli = animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');
currentPointVelj = animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
currentPointVelk = animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
plot(ti(3:end),smolDiskAngVelRad(3:end,1),'b');plot(ti(3:end),smolDiskAngVelRad(3:end,2),'r');plot(ti(3:end),smolDiskAngVelRad(3:end,3),'k');
subplot(2,3,6);hold on
currentPointAcci = animatedline('Color','b','Marker','o','MarkerSize',10,'MarkerFaceColor','b','HandleVisibility','off');
currentPointAccj = animatedline('Color','r','Marker','o','MarkerSize',10,'MarkerFaceColor','r','HandleVisibility','off');
currentPointAcck = animatedline('Color','k','Marker','o','MarkerSize',10,'MarkerFaceColor','k','HandleVisibility','off');
plot(ti(4:end),angacc(4:end,1),'b');plot(ti(4:end),angacc(4:end,2),'r');plot(ti(4:end),angacc(4:end,3),'k');
pausePlay = uicontrol; pausePlay.String = 'Pause'; pausePlay.Style = 'togglebutton';
pausePlay.Position = [20 100 150 50]; pausePlay.Value = 1;
quit = uicontrol; quit.String = 'Quit'; quit.Style = 'togglebutton';
quit.Position = [20 40 150 50]; quit.Value = 1;
i = 1;
while true
    tic;
    %% Animation %%
    subplot(2,3,3); delete(patch1);
    hold on
    patch1 = patch('Faces',f1,'Vertices',[xxx1i(:,i) yyy1i(:,i) zzz1i(:,i)],'EdgeColor','none','FaceColor','blue');
    if exist('f2')
        delete(patch2);
        patch2 = patch('Faces',f2,'Vertices',[xxx2i(:,i) yyy2i(:,i) zzz2i(:,i)],'EdgeColor','none','FaceColor','red');
    end
    alpha(0.3)
    subplot(2,3,2); delete(patch3); hold on
    clearpoints(s3); addpoints(s3,point(1,i),point(2,i),point(3,i))
    patch3 = patch('Faces',f1,'Vertices',[xxx1i(:,i) yyy1i(:,i) zzz1i(:,i)],'EdgeColor','none','FaceColor','blue');
    if exist('f2')
        delete(patch4);
        patch4 = patch('Faces',f2,'Vertices',[xxx2i(:,i) yyy2i(:,i) zzz2i(:,i)],'EdgeColor','none','FaceColor','red');
    end
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