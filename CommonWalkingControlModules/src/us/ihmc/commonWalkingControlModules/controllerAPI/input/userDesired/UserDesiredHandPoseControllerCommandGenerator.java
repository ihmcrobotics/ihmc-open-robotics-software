package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
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
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class UserDesiredHandPoseControllerCommandGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable userDoHandPose = new BooleanYoVariable("userDoHandPose", registry);
   private final BooleanYoVariable userDesiredSetHandPoseToActual = new BooleanYoVariable("userDesiredSetHandPoseToActual", registry);
   
   private final DoubleYoVariable userDesiredHandPoseTrajectoryTime = new DoubleYoVariable("userDesiredHandPoseTrajectoryTime", registry);

   private final EnumYoVariable<RobotSide> userHandPoseSide = new EnumYoVariable<RobotSide>("userHandPoseSide", registry, RobotSide.class);
   private final EnumYoVariable<BaseForControl> userHandPoseBaseForControl = new EnumYoVariable<BaseForControl>("userHandPoseBaseForControl", registry, BaseForControl.class);

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
         public void variableChanged(YoVariable<?> v)
         {
            if (userDesiredSetHandPoseToActual.getBooleanValue())
            {
               ReferenceFrame referenceFrame = getReferenceFrameToUse();

               ReferenceFrame wristFrame = fullRobotModel.getEndEffectorFrame(userHandPoseSide.getEnumValue(), LimbName.ARM);
               FramePose currentPose = new FramePose(wristFrame);

               currentPose.changeFrame(referenceFrame);

               userDesiredHandPose.setPosition(currentPose.getFramePointCopy().getPointCopy());
               userDesiredHandPose.setOrientation(currentPose.getFrameOrientationCopy().getQuaternionCopy());

               userDesiredSetHandPoseToActual.set(false);
            }
         }
      });
      
      userDoHandPose.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            if (userDoHandPose.getBooleanValue())
            {
               userDesiredHandPose.getFramePoseIncludingFrame(framePose);
               
               ReferenceFrame referenceFrame = getReferenceFrameToUse();
               framePose.setIncludingFrame(referenceFrame, framePose.getGeometryObject());

//               framePose.changeFrame(ReferenceFrame.getWorldFrame());
//               System.out.println("framePose " + framePose);

               HandTrajectoryCommand handTrajectoryControllerCommand = new HandTrajectoryCommand(referenceFrame, userHandPoseSide.getEnumValue(), userHandPoseBaseForControl.getEnumValue());
                
               FrameSE3TrajectoryPoint trajectoryPoint = new FrameSE3TrajectoryPoint(referenceFrame);
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
