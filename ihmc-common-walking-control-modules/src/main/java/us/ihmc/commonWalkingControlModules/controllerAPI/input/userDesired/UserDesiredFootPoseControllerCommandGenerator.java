package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class UserDesiredFootPoseControllerCommandGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean userDoFootPose = new YoBoolean("userDoFootPose", registry);
   private final YoDouble userDesiredFootPoseTrajectoryTime = new YoDouble("userDesiredFootPoseTrajectoryTime", registry);

   private final YoEnum<RobotSide> userFootPoseSide = new YoEnum<RobotSide>("userFootPoseSide", registry, RobotSide.class);

   private final YoFramePose userDesiredFootPose;
   
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

   private final FramePose3D framePose = new FramePose3D(ReferenceFrame.getWorldFrame());
   
   public UserDesiredFootPoseControllerCommandGenerator(final CommandInputManager controllerCommandInputManager, final FullHumanoidRobotModel fullRobotModel, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      userDesiredFootPose = new YoFramePose("userDesiredFootPose", ReferenceFrame.getWorldFrame(), registry);

      for (RobotSide robotSide : RobotSide.values())
      {
         ReferenceFrame ankleFrame = fullRobotModel.getSoleFrame(robotSide).getParent();
         ZUpFrame zUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUpFrame");
         ankleZUpFrames.set(robotSide, zUpFrame);
      }
      
      userDoFootPose.addVariableChangedListener(new VariableChangedListener()
      {
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (userDoFootPose.getBooleanValue())
            {
               userDesiredFootPose.getFramePose(framePose);
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
