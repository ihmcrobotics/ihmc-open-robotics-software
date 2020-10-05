package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class UserDesiredFootPoseControllerCommandGenerator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean userDoFootPose = new YoBoolean("userDoFootPose", registry);
   private final YoDouble userDesiredFootPoseTrajectoryTime = new YoDouble("userDesiredFootPoseTrajectoryTime", registry);

   private final YoEnum<RobotSide> userFootPoseSide = new YoEnum<RobotSide>("userFootPoseSide", registry, RobotSide.class);

   private final YoFramePoseUsingYawPitchRoll userDesiredFootPose;
   
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

   private final FramePose3D framePose = new FramePose3D(ReferenceFrame.getWorldFrame());
   
   public UserDesiredFootPoseControllerCommandGenerator(final CommandInputManager controllerCommandInputManager, final FullHumanoidRobotModel fullRobotModel, double defaultTrajectoryTime, YoRegistry parentRegistry)
   {
      userDesiredFootPose = new YoFramePoseUsingYawPitchRoll("userDesiredFootPose", ReferenceFrame.getWorldFrame(), registry);

      for (RobotSide robotSide : RobotSide.values())
      {
         ReferenceFrame ankleFrame = fullRobotModel.getSoleFrame(robotSide).getParent();
         ZUpFrame zUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUpFrame");
         ankleZUpFrames.set(robotSide, zUpFrame);
      }
      
      userDoFootPose.addListener(new YoVariableChangedListener()
      {
         public void changed(YoVariable v)
         {
            if (userDoFootPose.getBooleanValue())
            {
               framePose.setIncludingFrame(userDesiredFootPose);
               ReferenceFrame soleZUpFrame = ankleZUpFrames.get(userFootPoseSide.getEnumValue());
               soleZUpFrame.update();
               framePose.setIncludingFrame(soleZUpFrame, framePose);

               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               System.out.println("framePose " + framePose);

               FootTrajectoryCommand footTrajectoryControllerCommand = new FootTrajectoryCommand();
               
               FrameSE3TrajectoryPoint trajectoryPoint = new FrameSE3TrajectoryPoint(ReferenceFrame.getWorldFrame());
               trajectoryPoint.setTime(userDesiredFootPoseTrajectoryTime.getDoubleValue());
               trajectoryPoint.setPosition(framePose.getPosition());
               trajectoryPoint.setOrientation(framePose.getOrientation());
               trajectoryPoint.setLinearVelocity(new Vector3D());
               trajectoryPoint.setAngularVelocity(new Vector3D());
    
               footTrajectoryControllerCommand.getSE3Trajectory().addTrajectoryPoint(trajectoryPoint);
               
               footTrajectoryControllerCommand.setRobotSide(userFootPoseSide.getEnumValue());

               System.out.println("Submitting " + footTrajectoryControllerCommand);
               controllerCommandInputManager.submitCommand(footTrajectoryControllerCommand);
               
               userDoFootPose.set(false);
            }
         }
      });

      userDesiredFootPoseTrajectoryTime.set(defaultTrajectoryTime);
      userFootPoseSide.set(RobotSide.LEFT);
      parentRegistry.addChild(registry);
   }


}
