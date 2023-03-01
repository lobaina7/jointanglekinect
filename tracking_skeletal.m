%% setup
%% Code for detecting real elbow angle from upper limbs using a Kinect for Windows v1
close
vid = videoinput('kinect',1,'RGB_640x480');
depth = videoinput('kinect',2,'Depth_640x480');

src = getselectedsource(vid);
depthsrc= getselectedsource(depth);
src.CameraElevationAngle = 0;
depthsrc.CameraElevationAngle = 0;

vid.FramesPerTrigger = 1;
depth.FramesPerTrigger = 1;
vid.TriggerRepeat = Inf;
depth.TriggerRepeat = Inf;

%% tracking skeletal
% Turn on skeletal tracking.
depthsrc.TrackingMode = 'Skeleton';
depthsrc.BodyPosture = 'Seated';
%% Set the triggering mode to 'manual'
triggerconfig([vid depth],'manual');
%%
start([vid depth])% frameslogged is 0 while trigger is no able, because trigger configuration is manual 
%%
xyz_shoulder=[];
xyz_elbow=[];
xyz_wrist=[];
a=[];
time=[];
for i=1: 200
    trigger([vid depth])% frameslogged is different to 0 because trigger configuration is manual
    
    [frameData_Color, timeData_Color, metaData_Color] = getdata(vid);
    [frameData_Depth, timeData_Depth, metaData_Depth] = getdata(depth);
    image = frameData_Color(:, :, :, 1);
    imshow(image);
    %drawnow     % update figure window good practice but not in this case
    trackedSkeletons = find(metaData_Depth(1).IsSkeletonTracked);
    jointCoordinates = metaData_Depth(1).JointWorldCoordinates(:, :, trackedSkeletons);
    % Skeleton's joint indices with respect to the color image
    jointIndices = metaData_Depth(1).JointImageIndices(:, :, trackedSkeletons);
    % Find number of Skeletons tracked
    nSkeleton = length(trackedSkeletons);
    
     % Upper Skeleton connection map to link the joints
     %% from util_skeletonViewer and KEV3
    SkeletonConnectionMap = [[1 2]; % Spine
        [2 3];
        [3 4];
        [3 5]; 
        [5 6]; %Left arm
        [6 7]; %Left forearm
        [7 8];
        [3 9]; %Right Hand
        [9 10];
        [10 11];
        [11 12];
        [1 17]; % Right Leg
        [17 18];
        [18 19];
        [19 20];
        [1 13]; % Left Leg
        [13 14];
        [14 15];
        [15 16]];
     % Draw the skeletons on the RGB image
    jointIndices(jointIndices==0) = nan;
    
    if nSkeleton > 0
        hold on
        for j = 1:19
            X1 = [jointIndices(SkeletonConnectionMap(j,1),1,1) jointIndices(SkeletonConnectionMap(j,2),1,1)];
            Y1 = [jointIndices(SkeletonConnectionMap(j,1),2,1) jointIndices(SkeletonConnectionMap(j,2),2,1)];
            if j==5 || j==6
                line(X1,Y1, 'LineWidth', 3, 'LineStyle', '-', 'Marker', 'o', 'Color', 'y');
            else
                line(X1,Y1, 'LineWidth', 2, 'LineStyle', '-', 'Marker', '+', 'Color', 'r');
                
            end   
        end
        hold off;
%%     get joint coordinates
        joint_shoulder=5;
        joint_elbow=6;
        joint_wrist=7;
        
        xyz_shoulder = cat(1,xyz_shoulder,metaData_Depth(1).JointWorldCoordinates(joint_shoulder, :, trackedSkeletons));
        xyz_elbow = cat(1,xyz_shoulder,metaData_Depth(1).JointWorldCoordinates(joint_elbow, :, trackedSkeletons));
        xyz_wrist = cat(1,xyz_shoulder,metaData_Depth(1).JointWorldCoordinates(joint_wrist, :, trackedSkeletons));
%%     plot joints shoulder-elbow-wrist in 3D

% %        p=plot3(xyz_shoulder(end,:),xyz_elbow(end,:),xyz_wrist(end,:));
% %        view([90 -90 -90])
% %        xlabel('Z(m)')
% %        ylabel('X(m)')
% %        zlabel('Y(m)')
% %        p.LineWidth = 3;
% %        p.Marker = "o";
% %        grid on


%%     plot joint angles of elbow
       P1= xyz_elbow(end,:)-xyz_shoulder(end,:); % e *-> s
       P2= xyz_elbow(end,:)-xyz_wrist(end,:); % e *-> w
       angle = 180-atan2d(norm(cross(P1,P2)),dot(P1,P2)); % Angle in radians atand, in degree atan2d
       a = cat(1,a,angle);
       time=cat(1,time,timeData_Depth);

%         
    end
end
%% Draw the Skeleton Over the Corresponding Color Image



%%
stop([vid depth]);
delete([vid depth])

%%
% This path includes the skeletalViewer function which accepts the skeleton data,
% color image and number of skeletons as inputs and displays the skeleton
% overlaid on the color image
       figure
       p=plot(time,a);
       xlabel('tiempo(sec)')
       ylabel('ángulo (grades)')
       p.LineWidth = 2;