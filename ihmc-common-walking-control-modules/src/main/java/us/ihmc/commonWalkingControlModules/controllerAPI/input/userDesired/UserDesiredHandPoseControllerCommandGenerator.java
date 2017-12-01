package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;

public class UserDesiredHandPoseControllerCommandGenerator
{
   
   public enum BaseForControl
   {
      CHEST, WORLD, WALKING_PATH
   }
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean userDoHandPose = new YoBoolean("userDoHandPose", registry);
   private final YoBoolean userDesiredSetHandPoseToActual = new YoBoolean("userDesiredSetHandPoseToActual", registry);

   private final YoDouble userDesiredHandPoseTrajectoryTime = new YoDouble("userDesiredHandPoseTrajectoryTime", registry);

   private final YoEnum<RobotSide> userHandPoseSide = new YoEnum<RobotSide>("userHandPoseSide", registry, RobotSide.class);
   private final YoEnum<BaseForControl> userHandPoseBaseForControl = new YoEnum<BaseForControl>("userHandPoseBaseForControl", registry, BaseForControl.class);

   private final YoFramePose userDesiredHandPose;

   private final ReferenceFrame chestFrame;

   private final FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());

   public UserDesiredHandPoseControllerCommandGenerator(final CommandInputManager controllerCommandInputManager, final FullHumanoidRobotModel fullRobotModel, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      userDesiredHandPose = new YoFramePose("userDesiredHandPose", ReferenceFrame.getWorldFrame(), registry);

      chestFrame = fullRobotModel.getChest().getBodyFixedFrame();


      userDesiredSetHandPoseToActual.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userDesiredSetHandPoseToActual.getBooleanValue())
            {
               ReferenceFrame referenceFrame = getReferenceFrameToUse();

               ReferenceFrame wristFrame = fullRobotModel.getEndEffectorFrame(userHandPoseSide.getEnumValue(), LimbName.ARM);
               FramePose currentPose = new FramePose(wristFrame);

               currentPose.changeFrame(referenceFrame);

               userDesiredHandPose.setPosition(new Point3D(currentPose.getFramePointCopy()));
               userDesiredHandPose.setOrientation(currentPose.getFrameOrientationCopy());

               userDesiredSetHandPoseToActual.set(false);
            }
         }
      });

      userDoHandPose.addVariableChangedListener(new VariableChangedListener()
      {
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userDoHandPose.getBooleanValue())
            {
               userDesiredHandPose.getFramePoseIncludingFrame(framePose);

               ReferenceFrame referenceFrameToUse = getReferenceFrameToUse();
               framePose.setIncludingFrame(referenceFrameToUse, framePose.getGeometryObject());

//               framePose.changeFrame(ReferenceFrame.getWorldFrame());
//               System.out.println("framePose " + framePose);

               HandTrajectoryCommand handTrajectoryControllerCommand = new HandTrajectoryCommand(userHandPoseSide.getEnumValue(), referenceFrameToUse, referenceFrameToUse);

               FrameSE3TrajectoryPoint trajectoryPoint = new FrameSE3TrajectoryPoint(referenceFrameToUse);
               trajectoryPoint.setTime(userDesiredHandPoseTrajectoryTime.getDoubleValue());
               trajectoryPoint.setPosition(framePose.getFramePointCopy());
               trajectoryPoint.setOrientation(framePose.getFrameOrientationCopy());
               trajectoryPoint.setLinearVelocity(new Vector3D());
               trajectoryPoint.setAngularVelocity(new Vector3D());

               handTrajectoryControllerCommand.addTrajectoryPoint(trajectoryPoint);


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
