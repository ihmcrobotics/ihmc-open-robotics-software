clear;
clc;
close;

data = load("/home/rgriffin/Documents/IHMC/papers/2023_humanoids/pushRecoveryMatlab/data.scs2.mat");
desiredCMPX = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.WalkingControllerState.LinearMomentumRateControlModule.desiredCMPX;
desiredCMPY = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.WalkingControllerState.LinearMomentumRateControlModule.desiredCMPY;
desiredCMP = [desiredCMPX, desiredCMPY];

icpX = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.capturePointX;
icpY = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.capturePointY;
icp = [icpX, icpY];

desiredICPX = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.BalanceManager.desiredICPX;
desiredICPY = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.BalanceManager.desiredICPY;
desiredICP = [desiredICPX, desiredICPY];

offsetX = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.BalanceManager.finalDesiredICPX(2);
offsetY = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.BalanceManager.finalDesiredICPY(2);
offset = [offsetX, offsetY];

leftFootPolygon_0_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.leftFootPolygon_0_x;
leftFootPolygon_0_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.leftFootPolygon_0_y;
leftFootPolygon_1_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.leftFootPolygon_1_x;
leftFootPolygon_1_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.leftFootPolygon_1_y;
leftFootPolygon_2_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.leftFootPolygon_2_x;
leftFootPolygon_2_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.leftFootPolygon_2_y;
leftFootPolygon_3_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.leftFootPolygon_3_x;
leftFootPolygon_3_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.leftFootPolygon_3_y;

rightFootPolygon_0_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.rightFootPolygon_0_x;
rightFootPolygon_0_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.rightFootPolygon_0_y;
rightFootPolygon_1_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.rightFootPolygon_1_x;
rightFootPolygon_1_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.rightFootPolygon_1_y;
rightFootPolygon_2_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.rightFootPolygon_2_x;
rightFootPolygon_2_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.rightFootPolygon_2_y;
rightFootPolygon_3_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.rightFootPolygon_3_x;
rightFootPolygon_3_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerToolbox.BipedSupportPolygons.rightFootPolygon_3_y;

leftFootstep0Foothold_0_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0Foothold_0_x;
leftFootstep0Foothold_0_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0Foothold_0_y;
leftFootstep0Foothold_1_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0Foothold_1_x;
leftFootstep0Foothold_1_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0Foothold_1_y;
leftFootstep0Foothold_2_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0Foothold_2_x;
leftFootstep0Foothold_2_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0Foothold_2_y;
leftFootstep0Foothold_3_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0Foothold_3_x;
leftFootstep0Foothold_3_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0Foothold_3_y;
leftFootstep0PoseX = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0PoseX;
leftFootstep0PoseY = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0PoseY;
leftFootstep0PoseZ = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0PoseZ;
leftFootstep0PoseQx = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0PoseQx;
leftFootstep0PoseQy = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0PoseQy;
leftFootstep0PoseQz = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0PoseQz;
leftFootstep0PoseQs = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.leftFootstep0PoseQs;

rightFootstep0Foothold_0_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_0_x;
rightFootstep0Foothold_0_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_0_y;
rightFootstep0Foothold_1_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_1_x;
rightFootstep0Foothold_1_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_1_y;
rightFootstep0Foothold_2_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_2_x;
rightFootstep0Foothold_2_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_2_y;
rightFootstep0Foothold_3_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_3_x;
rightFootstep0Foothold_3_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_3_y;
rightFootstep0PoseX = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0PoseX;
rightFootstep0PoseY = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0PoseY;
rightFootstep0PoseZ = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0PoseZ;
rightFootstep0PoseQx = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0PoseQx;
rightFootstep0PoseQy = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0PoseQy;
rightFootstep0PoseQz = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0PoseQz;
rightFootstep0PoseQs = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0PoseQs;

leftFootstepDesired = [leftFootstep0Foothold_0_x, leftFootstep0Foothold_0_y];
leftFootstepDesired(:,:,2) = [leftFootstep0Foothold_1_x, leftFootstep0Foothold_1_y];
leftFootstepDesired(:,:,3) = [leftFootstep0Foothold_2_x, leftFootstep0Foothold_2_y];
leftFootstepDesired(:,:,4) = [leftFootstep0Foothold_3_x, leftFootstep0Foothold_3_y];

rightFootstepDesired = [rightFootstep0Foothold_0_x, rightFootstep0Foothold_0_y];
rightFootstepDesired(:,:,2) = [rightFootstep0Foothold_1_x, rightFootstep0Foothold_1_y];
rightFootstepDesired(:,:,3) = [rightFootstep0Foothold_2_x, rightFootstep0Foothold_2_y];
rightFootstepDesired(:,:,4) = [rightFootstep0Foothold_3_x, rightFootstep0Foothold_3_y];

total_size = length(desiredCMPX);

for height = 1:4
    for index = 1:total_size
        left_qx = leftFootstep0PoseQx(index);
        left_qy = leftFootstep0PoseQy(index);
        left_qz = leftFootstep0PoseQz(index);
        left_qs = leftFootstep0PoseQs(index);
        left_x = leftFootstepDesired(index,1,height);
        left_y = leftFootstepDesired(index,2,height);


        left_cross_x = -2.0 * (left_qz * left_y);
        left_cross_y = 2.0 * (left_qz * left_x);
        left_cross_z = 2.0 * (left_qx * left_y - left_qy * left_x);
        left_cross_cross_x = left_qy * left_cross_z - left_qz * left_cross_y;
        left_cross_cross_y = left_qz * left_cross_x - left_qx * left_cross_z;
        left_cross_cross_z = left_qx * left_cross_y - left_qy * left_cross_x;

        leftFootstepDesired(index,1,height) = left_x + left_qs * left_cross_x + left_cross_cross_x + leftFootstep0PoseX(index);
        leftFootstepDesired(index,2,height) = left_y + left_qs * left_cross_y + left_cross_cross_y + leftFootstep0PoseY(index);

        right_qx = rightFootstep0PoseQx(index);
        right_qy = rightFootstep0PoseQy(index);
        right_qz = rightFootstep0PoseQz(index);
        right_qs = rightFootstep0PoseQs(index);
        right_x = rightFootstepDesired(index,1,height);
        right_y = rightFootstepDesired(index,2,height);

        right_cross_x = -2.0 * (right_qz * right_y);
        right_cross_y = 2.0 * (right_qz * right_x);
        right_cross_z = 2.0 * (right_qx * right_y - right_qy * right_x);
        right_cross_cross_x = right_qy * right_cross_z - right_qz * right_cross_y;
        right_cross_cross_y = right_qz * right_cross_x - right_qx * right_cross_z;
        right_cross_cross_z = right_qx * right_cross_y - right_qy * right_cross_x;

        rightFootstepDesired(index,1,height) = right_x + right_qs * right_cross_x + right_cross_cross_x + rightFootstep0PoseX(index);
        rightFootstepDesired(index,2,height) = right_y + right_qs * right_cross_y + right_cross_cross_y + rightFootstep0PoseY(index);
    endfor
endfor

yaw = data.root.main.DRCEstimatorThread.NadiaEtherCATRealtimeThread.HardwareMap.pelvis_stim_imuYoEtherSnacksStimIMU.pelvis_stim_imuMahonyYaw(2)
yaw = -yaw - 3.145 / 2.0;;
rotationMatrixToPlotFrame = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];

stanceIndex0 = 183;

stepDesiredIndex0 = 15;
stepDesiredIndex1 = 502;
stepDesiredIndex2 = 894;
stepDesiredIndex3 = 1204;
stepDesiredIndex4 = 1588;

stepAchievedIndex0 = 381;
stepAchievedIndex1 = 890;
stepAchievedIndex2 = 1168;
stepAchievedIndex3 = 1544;
stepAchievedIndex4 = 2087;


desiredCMPInPlotFrame = zeros(total_size, 2);
desiredICPInPlotFrame = zeros(total_size, 2);
icpInPlotFrame = zeros(total_size, 2);
for row = 1:total_size
  desiredCMP(row,:) = desiredCMP(row,:) - offset;
  desiredCMPInPlotFrame(row,:) = (rotationMatrixToPlotFrame * desiredCMP(row,:)')';
    desiredICP(row,:) = desiredICP(row,:) - offset;
  desiredICPInPlotFrame(row,:) = (rotationMatrixToPlotFrame * desiredICP(row,:)')';
      icp(row,:) = icp(row,:) - offset;
  icpInPlotFrame(row,:) = (rotationMatrixToPlotFrame * icp(row,:)')';
endfor

stanceFoot0 = zeros(5,2);
stanceFoot0(1,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(stanceIndex0,:); rightFootPolygon_0_x(stanceIndex0,:)] - offset'))';
stanceFoot0(2,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(stanceIndex0,:); rightFootPolygon_0_x(stanceIndex0,:)] - offset'))';
stanceFoot0(3,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(stanceIndex0,:); rightFootPolygon_0_x(stanceIndex0,:)] - offset'))';
stanceFoot0(4,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(stanceIndex0,:); rightFootPolygon_0_x(stanceIndex0,:)] - offset'))';
stanceFoot0(5,:) = stanceFoot0(1,:);

stepDesired0 = zeros(5,2);
stepDesired0(1,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex0,1,1); leftFootstepDesired(stepDesiredIndex0,2,1)] - offset'))';
stepDesired0(2,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex0,1,2); leftFootstepDesired(stepDesiredIndex0,2,2)] - offset'))';
stepDesired0(3,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex0,1,3); leftFootstepDesired(stepDesiredIndex0,2,3)] - offset'))';
stepDesired0(4,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex0,1,4); leftFootstepDesired(stepDesiredIndex0,2,4)] - offset'))';
stepDesired0(5,:) = stepDesired0(1,:);

stepDesired1 = zeros(5,2);
stepDesired1(1,:) = (rotationMatrixToPlotFrame * ([rightFootstepDesired(stepDesiredIndex1,1,1); rightFootstepDesired(stepDesiredIndex1,2,1)] - offset'))';
stepDesired1(2,:) = (rotationMatrixToPlotFrame * ([rightFootstepDesired(stepDesiredIndex1,1,2); rightFootstepDesired(stepDesiredIndex1,2,2)] - offset'))';
stepDesired1(3,:) = (rotationMatrixToPlotFrame * ([rightFootstepDesired(stepDesiredIndex1,1,3); rightFootstepDesired(stepDesiredIndex1,2,3)] - offset'))';
stepDesired1(4,:) = (rotationMatrixToPlotFrame * ([rightFootstepDesired(stepDesiredIndex1,1,4); rightFootstepDesired(stepDesiredIndex1,2,4)] - offset'))';
stepDesired1(5,:) = stepDesired1(1,:);

stepDesired2 = zeros(5,2);
stepDesired2(1,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex2,1,1); leftFootstepDesired(stepDesiredIndex2,2,1)] - offset'))';
stepDesired2(2,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex2,1,2); leftFootstepDesired(stepDesiredIndex2,2,2)] - offset'))';
stepDesired2(3,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex2,1,3); leftFootstepDesired(stepDesiredIndex2,2,3)] - offset'))';
stepDesired2(4,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex2,1,4); leftFootstepDesired(stepDesiredIndex2,2,4)] - offset'))';
stepDesired2(5,:) = stepDesired2(1,:);

stepDesired3 = zeros(5,2);
stepDesired3(1,:) = (rotationMatrixToPlotFrame * ([rightFootstepDesired(stepDesiredIndex3,1,1); rightFootstepDesired(stepDesiredIndex3,2,1)] - offset'))';
stepDesired3(2,:) = (rotationMatrixToPlotFrame * ([rightFootstepDesired(stepDesiredIndex3,1,2); rightFootstepDesired(stepDesiredIndex3,2,2)] - offset'))';
stepDesired3(3,:) = (rotationMatrixToPlotFrame * ([rightFootstepDesired(stepDesiredIndex3,1,3); rightFootstepDesired(stepDesiredIndex3,2,3)] - offset'))';
stepDesired3(4,:) = (rotationMatrixToPlotFrame * ([rightFootstepDesired(stepDesiredIndex3,1,4); rightFootstepDesired(stepDesiredIndex3,2,4)] - offset'))';
stepDesired3(5,:) = stepDesired3(1,:);

stepDesired4 = zeros(5,2);
stepDesired4(1,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex4,1,1); leftFootstepDesired(stepDesiredIndex4,2,1)] - offset'))';
stepDesired4(2,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex4,1,2); leftFootstepDesired(stepDesiredIndex4,2,2)] - offset'))';
stepDesired4(3,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex4,1,3); leftFootstepDesired(stepDesiredIndex4,2,3)] - offset'))';
stepDesired4(4,:) = (rotationMatrixToPlotFrame * ([leftFootstepDesired(stepDesiredIndex4,1,4); leftFootstepDesired(stepDesiredIndex4,2,4)] - offset'))';
stepDesired4(5,:) = stepDesired4(1,:);

stepAchieved0 = zeros(5,2);
stepAchieved0(1,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_0_x(stepAchievedIndex0,:); leftFootPolygon_0_y(stepAchievedIndex0,:)] - offset'))';
stepAchieved0(2,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_1_x(stepAchievedIndex0,:); leftFootPolygon_1_y(stepAchievedIndex0,:)] - offset'))';
stepAchieved0(3,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_2_x(stepAchievedIndex0,:); leftFootPolygon_2_y(stepAchievedIndex0,:)] - offset'))';
stepAchieved0(4,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_3_x(stepAchievedIndex0,:); leftFootPolygon_3_y(stepAchievedIndex0,:)] - offset'))';
stepAchieved0(5,:) = stepAchieved0(1,:);

stepAchieved1 = zeros(5,2);
stepAchieved1(1,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(stepAchievedIndex1,:); rightFootPolygon_0_y(stepAchievedIndex1,:)] - offset'))';
stepAchieved1(2,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_1_x(stepAchievedIndex1,:); rightFootPolygon_1_y(stepAchievedIndex1,:)] - offset'))';
stepAchieved1(3,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_2_x(stepAchievedIndex1,:); rightFootPolygon_2_y(stepAchievedIndex1,:)] - offset'))';
stepAchieved1(4,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_3_x(stepAchievedIndex1,:); rightFootPolygon_3_y(stepAchievedIndex1,:)] - offset'))';
stepAchieved1(5,:) = stepAchieved1(1,:);

stepAchieved2 = zeros(5,2);
stepAchieved2(1,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_0_x(stepAchievedIndex2,:); leftFootPolygon_0_y(stepAchievedIndex2,:)] - offset'))';
stepAchieved2(2,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_1_x(stepAchievedIndex2,:); leftFootPolygon_1_y(stepAchievedIndex2,:)] - offset'))';
stepAchieved2(3,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_2_x(stepAchievedIndex2,:); leftFootPolygon_2_y(stepAchievedIndex2,:)] - offset'))';
stepAchieved2(4,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_3_x(stepAchievedIndex2,:); leftFootPolygon_3_y(stepAchievedIndex2,:)] - offset'))';
stepAchieved2(5,:) = stepAchieved2(1,:);

stepAchieved3 = zeros(5,2);
stepAchieved3(1,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(stepAchievedIndex3,:); rightFootPolygon_0_y(stepAchievedIndex3,:)] - offset'))';
stepAchieved3(2,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_1_x(stepAchievedIndex3,:); rightFootPolygon_1_y(stepAchievedIndex3,:)] - offset'))';
stepAchieved3(3,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_2_x(stepAchievedIndex3,:); rightFootPolygon_2_y(stepAchievedIndex3,:)] - offset'))';
stepAchieved3(4,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_3_x(stepAchievedIndex3,:); rightFootPolygon_3_y(stepAchievedIndex3,:)] - offset'))';
stepAchieved3(5,:) = stepAchieved3(1,:);

stepAchieved4 = zeros(5,2);
stepAchieved4(1,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_0_x(stepAchievedIndex4,:); leftFootPolygon_0_y(stepAchievedIndex4,:)] - offset'))';
stepAchieved4(2,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_1_x(stepAchievedIndex4,:); leftFootPolygon_1_y(stepAchievedIndex4,:)] - offset'))';
stepAchieved4(3,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_2_x(stepAchievedIndex4,:); leftFootPolygon_2_y(stepAchievedIndex4,:)] - offset'))';
stepAchieved4(4,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_3_x(stepAchievedIndex4,:); leftFootPolygon_3_y(stepAchievedIndex4,:)] - offset'))';
stepAchieved4(5,:) = stepAchieved4(1,:);

figure(1)
hold;
plot(desiredCMPInPlotFrame(2:total_size,1), desiredCMPInPlotFrame(2:total_size,2), 'm--',
     desiredICPInPlotFrame(2:total_size,1), desiredICPInPlotFrame(2:total_size,2),'y-.',
     icpInPlotFrame(2:total_size,1), icpInPlotFrame(2:total_size,2),'b');

plot(stanceFoot0(:,1), stanceFoot0(:,2), 'r--',
     stepDesired0(:,1), stepDesired0(:,2), 'g',
     stepDesired1(:,1), stepDesired1(:,2), 'r',
     stepDesired2(:,1), stepDesired2(:,2), 'g',
     stepDesired3(:,1), stepDesired3(:,2), 'r',
     stepDesired4(:,1), stepDesired4(:,2), 'g',
     stepAchieved0(:,1), stepAchieved0(:,2),'g--',
     stepAchieved1(:,1), stepAchieved1(:,2),'r--',
     stepAchieved2(:,1), stepAchieved2(:,2),'g--',
     stepAchieved3(:,1), stepAchieved3(:,2),'r--',
     stepAchieved4(:,1), stepAchieved4(:,2),'g--');

