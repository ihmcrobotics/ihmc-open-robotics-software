package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.humanoidRobotics.model.FullHumanoidRobotModel;
import us.ihmc.humanoidRobotics.partNames.LimbName;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;

public class UserDesiredFootPoseProvider implements FootPoseProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());


   private final BooleanYoVariable userFootPoseTakeEm = new BooleanYoVariable("userFootPoseTakeEm", registry);
   private final DoubleYoVariable userFootPoseTrajectoryTime = new DoubleYoVariable("userDesiredPelvisTrajectoryTime", registry);

   private final EnumYoVariable<RobotSide> userFootPoseSide = new EnumYoVariable<RobotSide>("userFootPoseSide", registry, RobotSide.class);

   private final DoubleYoVariable userFootPoseX = new DoubleYoVariable("userFootPoseX", registry);
   private final DoubleYoVariable userFootPoseY = new DoubleYoVariable("userFootPoseY", registry);
   private final DoubleYoVariable userFootPoseZ = new DoubleYoVariable("userFootPoseZ", registry);

   private final DoubleYoVariable footPoseYawCheck = new DoubleYoVariable("footPoseYawCheck", registry);
   private final DoubleYoVariable footPosePitchCheck = new DoubleYoVariable("footPosePitchCheck", registry);
   private final DoubleYoVariable footPoseRollCheck = new DoubleYoVariable("footPoseRollCheck", registry);

   
   private final FullHumanoidRobotModel fullRobotModel;

   public UserDesiredFootPoseProvider(FullHumanoidRobotModel fullRobotModel, double defaultTrajectoryTime, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;

      parentRegistry.addChild(registry);
      userFootPoseTrajectoryTime.set(defaultTrajectoryTime);
   }


   @Override
   public boolean checkForNewPose(RobotSide robotSide)
   {
      if (userFootPoseSide.getEnumValue() != robotSide)
         return false;

      return userFootPoseTakeEm.getBooleanValue();
   }

   @Override
   public void clear()
   {
      userFootPoseTakeEm.set(false);
   }

   @Override
   public RobotSide checkForNewPose()
   {
      if (userFootPoseTakeEm.getBooleanValue())
      {
         return userFootPoseSide.getEnumValue();
      }

      return null;
   }

   @Override
   public FramePose getDesiredFootPose(RobotSide robotSide)
   {
      ReferenceFrame footFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(userFootPoseX.getDoubleValue(), userFootPoseY.getDoubleValue(), userFootPoseZ.getDoubleValue());
      
      
      FramePose framePose = new FramePose(footFrame, transform);
      
      framePose.changeFrame(ReferenceFrame.getWorldFrame());
      
      footPoseYawCheck.set(framePose.getYaw());
      footPosePitchCheck.set(framePose.getPitch());
      footPoseRollCheck.set(framePose.getRoll());

      
      userFootPoseTakeEm.set(false);
      
      return framePose;
   }

   @Override
   public double getTrajectoryTime()
   {
      return userFootPoseTrajectoryTime.getDoubleValue();
   }
}
