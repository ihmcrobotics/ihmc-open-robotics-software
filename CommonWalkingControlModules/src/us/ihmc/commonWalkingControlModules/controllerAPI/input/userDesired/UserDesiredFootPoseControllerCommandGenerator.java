package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class UserDesiredFootPoseControllerCommandGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable userDoFootPose = new BooleanYoVariable("userDoFootPose", registry);
   private final DoubleYoVariable userDesiredFootPoseTrajectoryTime = new DoubleYoVariable("userDesiredFootPoseTrajectoryTime", registry);

   private final EnumYoVariable<RobotSide> userFootPoseSide = new EnumYoVariable<RobotSide>("userFootPoseSide", registry, RobotSide.class);

   private final YoFramePose userDesiredFootPose;
   
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

   private final FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());
   
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
         public void variableChanged(YoVariable<?> v)
         {
            if (userDoFootPose.getBooleanValue())
            {
               userDesiredFootPose.getFramePose(framePose);
               ReferenceFrame soleZUpFrame = ankleZUpFrames.get(userFootPoseSide.getEnumValue());
               soleZUpFrame.update();
               framePose.setIncludingFrame(soleZUpFrame, framePose.getGeometryObject());

               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               System.out.println("framePose " + framePose);

               FootTrajectoryCommand footTrajectoryControllerCommand = new FootTrajectoryCommand();
               
               FrameSE3TrajectoryPoint trajectoryPoint = new FrameSE3TrajectoryPoint(ReferenceFrame.getWorldFrame());
               trajectoryPoint.setTime(userDesiredFootPoseTrajectoryTime.getDoubleValue());
               trajectoryPoint.setPosition(framePose.getFramePointCopy());
               trajectoryPoint.setOrientation(framePose.getFrameOrientationCopy());
               trajectoryPoint.setLinearVelocity(new Vector3D());
               trajectoryPoint.setAngularVelocity(new Vector3D());
    
               footTrajectoryControllerCommand.addTrajectoryPoint(trajectoryPoint);
               
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
