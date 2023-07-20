clear;
clc;
close;

data = load("/home/rgriffin/Documents/IHMC/papers/2023_humanoids/crossOverMatlab/data.scs2.mat");

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

rightFootstep0Foothold_0_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_0_x;
rightFootstep0Foothold_0_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_0_y;
rightFootstep0Foothold_1_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_1_x;
rightFootstep0Foothold_1_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_1_y;
rightFootstep0Foothold_2_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_2_x;
rightFootstep0Foothold_2_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_2_y;
rightFootstep0Foothold_3_x = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_3_x;
rightFootstep0Foothold_3_y = data.root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.FootstepListVisualizer.rightFootstep0Foothold_3_y;

yaw = data.root.main.DRCEstimatorThread.NadiaEtherCATRealtimeThread.HardwareMap.pelvis_stim_imuYoEtherSnacksStimIMU.pelvis_stim_imuMahonyYaw(2)
yaw = -yaw - 3.145 / 2.0;;
rotationMatrixToPlotFrame = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];

dt = 0.0030235130 - 0.0019990670;
stance0Time = 0.6320260360;
stance1Time = 1.026035483;
stance2Time = 2.415013661;
stance3Time = 3.326010013;
stance4Time = 3.503038825;

step0DesiredTime = 0.0870039450;
step1DesiredTime = 0.931030718;
step2DesiredTime = 1.80198342;
step3DesiredTime = 2.959015072;

step0AchievedTime = 0.6320260360;
step1AchievedTime = 1.652034847;
step2AchievedTime = 3.148000808;
step3AchievedTime = 3.490040517;

stance0Index = floor(stance0Time / dt) + 1;
stance1Index = floor(stance1Time / dt) + 1;
stance2Index = floor(stance2Time / dt) + 1;
stance3Index = floor(stance3Time / dt) + 1;
stance4Index = floor(stance4Time / dt) + 1;

step0AchievedIndex = floor(step0AchievedTime / dt) + 1
step1AchievedIndex = floor(step1AchievedTime / dt) + 1
step2AchievedIndex = floor(step2AchievedTime / dt) + 1
step3AchievedIndex = floor(step3AchievedTime / dt) + 1

step0DesiredIndex = floor(step0DesiredTime / dt) + 1;
step1DesiredIndex = floor(step1DesiredTime / dt) + 1;
step2DesiredIndex = floor(step2DesiredTime / dt) + 1;
step3DesiredIndex = floor(step3DesiredTime / dt) + 1;

total_size = length(desiredCMPX);

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

stance0 = zeros(5,2);
stance0(1,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_0_x(stance0Index,:); leftFootPolygon_0_y(stance0Index,:)] - offset'))';
stance0(2,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_1_x(stance0Index,:); leftFootPolygon_1_y(stance0Index,:)] - offset'))';
stance0(3,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_2_x(stance0Index,:); leftFootPolygon_2_y(stance0Index,:)] - offset'))';
stance0(4,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_3_x(stance0Index,:); leftFootPolygon_3_y(stance0Index,:)] - offset'))';
stance0(5,:) = stance0(1,:);

stance1 = zeros(5,2);
stance1(1,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(stance1Index,:); rightFootPolygon_0_y(stance1Index,:)] - offset'))';
stance1(2,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_1_x(stance1Index,:); rightFootPolygon_1_y(stance1Index,:)] - offset'))';
stance1(3,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_2_x(stance1Index,:); rightFootPolygon_2_y(stance1Index,:)] - offset'))';
stance1(4,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_3_x(stance1Index,:); rightFootPolygon_3_y(stance1Index,:)] - offset'))';
stance1(5,:) = stance1(1,:);

stance2 = zeros(5,2);
stance2(1,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_0_x(stance2Index,:); leftFootPolygon_0_y(stance2Index,:)] - offset'))';
stance2(2,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_1_x(stance2Index,:); leftFootPolygon_1_y(stance2Index,:)] - offset'))';
stance2(3,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_2_x(stance2Index,:); leftFootPolygon_2_y(stance2Index,:)] - offset'))';
stance2(4,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_3_x(stance2Index,:); leftFootPolygon_3_y(stance2Index,:)] - offset'))';
stance2(5,:) = stance2(1,:);

stance3 = zeros(5,2);
stance3(1,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(stance3Index,:); rightFootPolygon_0_y(stance3Index,:)] - offset'))';
stance3(2,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_1_x(stance3Index,:); rightFootPolygon_1_y(stance3Index,:)] - offset'))';
stance3(3,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_2_x(stance3Index,:); rightFootPolygon_2_y(stance3Index,:)] - offset'))';
stance3(4,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_3_x(stance3Index,:); rightFootPolygon_3_y(stance3Index,:)] - offset'))';
stance3(5,:) = stance3(1,:);

stance4 = zeros(5,2);
stance4(1,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_0_x(stance4Index,:); leftFootPolygon_0_y(stance4Index,:)] - offset'))';
stance4(2,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_1_x(stance4Index,:); leftFootPolygon_1_y(stance4Index,:)] - offset'))';
stance4(3,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_2_x(stance4Index,:); leftFootPolygon_2_y(stance4Index,:)] - offset'))';
stance4(4,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_3_x(stance4Index,:); leftFootPolygon_3_y(stance4Index,:)] - offset'))';
stance4(5,:) = stance4(1,:);


achievedStep0 = zeros(5,2);
achievedStep0(1,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(step0AchievedIndex); rightFootPolygon_0_y(step0AchievedIndex)] - offset'))';
achievedStep0(2,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_1_x(step0AchievedIndex); rightFootPolygon_1_y(step0AchievedIndex)] - offset'))';
achievedStep0(3,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_2_x(step0AchievedIndex); rightFootPolygon_2_y(step0AchievedIndex)] - offset'))';
achievedStep0(4,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_3_x(step0AchievedIndex); rightFootPolygon_3_y(step0AchievedIndex)] - offset'))';
achievedStep0(5,:) = achievedStep0(1,:);

achievedStep1 = zeros(5,2);
achievedStep1(1,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_0_x(step1AchievedIndex); leftFootPolygon_0_y(step1AchievedIndex)] - offset'))';
achievedStep1(2,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_1_x(step1AchievedIndex); leftFootPolygon_1_y(step1AchievedIndex)] - offset'))';
achievedStep1(3,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_2_x(step1AchievedIndex); leftFootPolygon_2_y(step1AchievedIndex)] - offset'))';
achievedStep1(4,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_3_x(step1AchievedIndex); leftFootPolygon_3_y(step1AchievedIndex)] - offset'))';
achievedStep1(5,:) = achievedStep1(1,:);

achievedStep2 = zeros(5,2);
achievedStep2(1,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_0_x(step2AchievedIndex); rightFootPolygon_0_y(step2AchievedIndex)] - offset'))';
achievedStep2(2,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_1_x(step2AchievedIndex); rightFootPolygon_1_y(step2AchievedIndex)] - offset'))';
achievedStep2(3,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_2_x(step2AchievedIndex); rightFootPolygon_2_y(step2AchievedIndex)] - offset'))';
achievedStep2(4,:) = (rotationMatrixToPlotFrame * ([rightFootPolygon_3_x(step2AchievedIndex); rightFootPolygon_3_y(step2AchievedIndex)] - offset'))';
achievedStep2(5,:) = achievedStep2(1,:);

achievedStep3 = zeros(5,2);
achievedStep3(1,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_0_x(step3AchievedIndex); leftFootPolygon_0_y(step3AchievedIndex)] - offset'))';
achievedStep3(2,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_1_x(step3AchievedIndex); leftFootPolygon_1_y(step3AchievedIndex)] - offset'))';
achievedStep3(3,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_2_x(step3AchievedIndex); leftFootPolygon_2_y(step3AchievedIndex)] - offset'))';
achievedStep3(4,:) = (rotationMatrixToPlotFrame * ([leftFootPolygon_3_x(step3AchievedIndex); leftFootPolygon_3_y(step3AchievedIndex)] - offset'))';
achievedStep3(5,:) = achievedStep3(1,:);


figure(1)
hold;
plot(desiredCMPInPlotFrame(2:total_size,1), desiredCMPInPlotFrame(2:total_size,2), 'm--',
     desiredICPInPlotFrame(2:total_size,1), desiredICPInPlotFrame(2:total_size,2),'y-.',
     icpInPlotFrame(2:total_size,1), icpInPlotFrame(2:total_size,2),'b');

plot(stance0(:,1), stance0(:,2), 'g',
     stance1(:,1), stance1(:,2), 'r',
     %stance2(:,1), stance2(:,2),
     %stance3(:,1), stance3(:,2),
     %stance4(:,1), stance4(:,2),
     achievedStep0(:,1), achievedStep0(:,2),'r--',
     achievedStep1(:,1), achievedStep1(:,2),'g--',
     achievedStep2(:,1), achievedStep2(:,2),'r--',
     achievedStep3(:,1), achievedStep3(:,2),'g--');

axis([-0.3, 0.4, -0.2, 0.5]);
