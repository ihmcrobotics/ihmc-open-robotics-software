package us.ihmc.commonWalkingControlModules.kinematics;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.yoUtilities.YoVariableRegistry;

import com.yobotics.simulationconstructionset.robotController.SensorProcessor;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;

public class BodyOrientationEstimatorThroughStanceLeg implements SensorProcessor
{
   private final String name;
   private final YoVariableRegistry registry;
   private final RobotSide robotSide;
   private final CommonWalkingReferenceFrames referenceFrames;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameOrientation orientation;

   public BodyOrientationEstimatorThroughStanceLeg(RobotSide robotSide, CommonWalkingReferenceFrames referenceFrames)
   {
      this.name = robotSide + getClass().getSimpleName();
      this.registry = new YoVariableRegistry(name);
      this.robotSide = robotSide;
      this.referenceFrames = referenceFrames;
      this.orientation =  new YoFrameOrientation(robotSide + "bodyOrientationThroughStanceLeg", "", worldFrame, registry);
   }

   public void packBodyOrientation(FrameOrientation bodyOrientationToPack)
   {
      bodyOrientationToPack.setIncludingFrame(orientation.getFrameOrientationCopy());  
   }

   private final double[] tempYawPitchRoll = new double[3];
   public void estimateBodyOrientation()
   {
      Transform3D footToBody = referenceFrames.getIMUFrame().getTransformToDesiredFrame(referenceFrames.getFootFrame(robotSide));
      RotationFunctions.getYawPitchRoll(tempYawPitchRoll, footToBody);
      
      Transform3D bodyToWorld = referenceFrames.getIMUFrame().getTransformToDesiredFrame(worldFrame);
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
