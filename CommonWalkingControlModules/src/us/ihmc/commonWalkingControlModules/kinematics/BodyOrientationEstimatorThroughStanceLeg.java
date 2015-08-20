package us.ihmc.commonWalkingControlModules.kinematics;

import javax.vecmath.Matrix3d;

import us.ihmc.simulationconstructionset.robotController.SensorProcessor;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;

public class BodyOrientationEstimatorThroughStanceLeg implements SensorProcessor
{
   private final String name;
   private final YoVariableRegistry registry;
   private final RobotSide robotSide;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameOrientation orientation;

   public BodyOrientationEstimatorThroughStanceLeg(RobotSide robotSide, CommonHumanoidReferenceFrames referenceFrames)
   {
      this.name = robotSide.getSideNameFirstLetter() + getClass().getSimpleName();
      this.registry = new YoVariableRegistry(name);
      this.robotSide = robotSide;
      this.referenceFrames = referenceFrames;
      this.orientation =  new YoFrameOrientation(robotSide.getSideNameFirstLetter() + "bodyOrientationThroughStanceLeg", "", worldFrame, registry);
   }

   public void packBodyOrientation(FrameOrientation bodyOrientationToPack)
   {
      bodyOrientationToPack.setIncludingFrame(orientation.getFrameOrientationCopy());  
   }

   private final double[] tempYawPitchRoll = new double[3];
   public void estimateBodyOrientation()
   {
      RigidBodyTransform footToBody = referenceFrames.getIMUFrame().getTransformToDesiredFrame(referenceFrames.getFootFrame(robotSide));
      RotationFunctions.getYawPitchRoll(tempYawPitchRoll, footToBody);
      
      RigidBodyTransform bodyToWorld = referenceFrames.getIMUFrame().getTransformToDesiredFrame(worldFrame);
      Matrix3d bodyToWorldRotation = new Matrix3d();
      bodyToWorld.get(bodyToWorldRotation);
      double bodyYaw = RotationFunctions.getYaw(bodyToWorldRotation);
      tempYawPitchRoll[0] = bodyYaw;
      orientation.setYawPitchRoll(tempYawPitchRoll[0], tempYawPitchRoll[1], tempYawPitchRoll[2]);
   }

   public void initialize()
   {
      update();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void update()
   {
      estimateBodyOrientation();
   }
}
