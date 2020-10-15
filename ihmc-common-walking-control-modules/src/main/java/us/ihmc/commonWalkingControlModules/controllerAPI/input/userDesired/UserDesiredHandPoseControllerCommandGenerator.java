package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class UserDesiredHandPoseControllerCommandGenerator
{
   
   public enum BaseForControl
   {
      CHEST, WORLD, WALKING_PATH
   }
   
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean userDoHandPose = new YoBoolean("userDoHandPose", registry);
   private final YoBoolean userDesiredSetHandPoseToActual = new YoBoolean("userDesiredSetHandPoseToActual", registry);

   private final YoDouble userDesiredHandPoseTrajectoryTime = new YoDouble("userDesiredHandPoseTrajectoryTime", registry);

   private final YoEnum<RobotSide> userHandPoseSide = new YoEnum<RobotSide>("userHandPoseSide", registry, RobotSide.class);
   private final YoEnum<BaseForControl> userHandPoseBaseForControl = new YoEnum<BaseForControl>("userHandPoseBaseForControl", registry, BaseForControl.class);

   private final YoFramePoseUsingYawPitchRoll userDesiredHandPose;

   private final ReferenceFrame chestFrame;

   private final FramePose3D framePose = new FramePose3D(ReferenceFrame.getWorldFrame());

   public UserDesiredHandPoseControllerCommandGenerator(final CommandInputManager controllerCommandInputManager, final FullHumanoidRobotModel fullRobotModel, double defaultTrajectoryTime, YoRegistry parentRegistry)
   {
      userDesiredHandPose = new YoFramePoseUsingYawPitchRoll("userDesiredHandPose", ReferenceFrame.getWorldFrame(), registry);

      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();


      userDesiredSetHandPoseToActual.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (userDesiredSetHandPoseToActual.getBooleanValue())
            {
               ReferenceFrame referenceFrame = getReferenceFrameToUse();

               ReferenceFrame wristFrame = fullRobotModel.getEndEffectorFrame(userHandPoseSide.getEnumValue(), LimbName.ARM);
               FramePose3D currentPose = new FramePose3D(wristFrame);

               currentPose.changeFrame(referenceFrame);

               userDesiredHandPose.setPosition(new Point3D(currentPose.getPosition()));
               userDesiredHandPose.setOrientation(currentPose.getOrientation());

               userDesiredSetHandPoseToActual.set(false);
            }
         }
      });

      userDoHandPose.addListener(new YoVariableChangedListener()
      {
         public void changed(YoVariable v)
         {
            if (userDoHandPose.getBooleanValue())
            {
               framePose.setIncludingFrame(userDesiredHandPose);

               ReferenceFrame referenceFrameToUse = getReferenceFrameToUse();
               framePose.setIncludingFrame(referenceFrameToUse, framePose);

//               framePose.changeFrame(ReferenceFrame.getWorldFrame());
//               System.out.println("framePose " + framePose);

               HandTrajectoryCommand handTrajectoryControllerCommand = new HandTrajectoryCommand(userHandPoseSide.getEnumValue(), referenceFrameToUse, referenceFrameToUse);

               FrameSE3TrajectoryPoint trajectoryPoint = new FrameSE3TrajectoryPoint(referenceFrameToUse);
               trajectoryPoint.setTime(userDesiredHandPoseTrajectoryTime.getDoubleValue());
               trajectoryPoint.setPosition(framePose.getPosition());
               trajectoryPoint.setOrientation(framePose.getOrientation());
               trajectoryPoint.setLinearVelocity(new Vector3D());
               trajectoryPoint.setAngularVelocity(new Vector3D());

               handTrajectoryControllerCommand.getSE3Trajectory().addTrajectoryPoint(trajectoryPoint);


               System.out.println("Submitting " + handTrajectoryControllerCommand);
               controllerCommandInputManager.submitCommand(handTrajectoryControllerCommand);

               userDoHandPose.set(false);
            }
         }
      });

      userDesiredHandPoseTrajectoryTime.set(defaultTrajectoryTime);
      userHandPoseSide.set(RobotSide.LEFT);
      userHandPoseBaseForControl.set(BaseForControl.CHEST);
      parentRegistry.addChild(registry);
   }

   private ReferenceFrame getReferenceFrameToUse()
   {
      ReferenceFrame referenceFrame;
      switch(userHandPoseBaseForControl.getEnumValue())
      {
      case CHEST:
      {
         referenceFrame = chestFrame;
         break;
      }
      case WORLD:
      {
         referenceFrame = ReferenceFrame.getWorldFrame();
      }
      case WALKING_PATH:
      {
         // TODO: What to do for walking path?
         referenceFrame = ReferenceFrame.getWorldFrame();
      }
      default:
      {
         throw new RuntimeException("Shouldn't get here!");
      }
      }
      return referenceFrame;
   }

}
