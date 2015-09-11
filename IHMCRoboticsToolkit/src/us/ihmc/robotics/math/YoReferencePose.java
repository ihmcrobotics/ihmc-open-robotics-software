package us.ihmc.robotics.math;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.kinematics.TransformInterpolationCalculator;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;

public class YoReferencePose extends ReferenceFrame
{
   private static final long serialVersionUID = -7908261385108357220L;

   private final YoFramePose yoFramePose;

   //Below are used for interpolation only
   private final TransformInterpolationCalculator transformInterpolationCalculator = new TransformInterpolationCalculator();
   private final RigidBodyTransform interpolationStartingPosition = new RigidBodyTransform();
   private final RigidBodyTransform interpolationGoalPosition = new RigidBodyTransform();
   private final RigidBodyTransform output = new RigidBodyTransform();

   //Below are used for updating YoFramePose only
   private final Quat4d rotation = new Quat4d();
   private final Vector3d translation = new Vector3d();
   private final double[] yawPitchRoll = new double[3];

   public YoReferencePose(String frameName, ReferenceFrame parentFrame, YoVariableRegistry registry)
   {
      super(frameName, parentFrame);
      yoFramePose = new YoFramePose(frameName + "_", this, registry);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      yoFramePose.getOrientation().getQuaternion(rotation);
      transformToParent.setRotation(rotation);
      YoFramePoint yoFramePoint = yoFramePose.getPosition();
      transformToParent.setTranslation(yoFramePoint.getX(), yoFramePoint.getY(), yoFramePoint.getZ());
   }

   public void setAndUpdate(RigidBodyTransform transform)
   {
      transform.get(rotation, translation);
      setAndUpdate(rotation, translation);
   }

   public void setAndUpdate(Vector3d newTranslation)
   {
      set(newTranslation);
      update();
   }

   public void setAndUpdate(Quat4d newRotation)
   {
      set(newRotation);
      update();
   }

   public void setAndUpdate(Quat4d newRotation, Vector3d newTranslation)
   {
      set(newRotation);
      set(newTranslation);
      update();
   }

   private void set(Quat4d newRotation)
   {
      RotationFunctions.setYawPitchRollBasedOnQuaternion(yawPitchRoll, newRotation);
      yoFramePose.setYawPitchRoll(yawPitchRoll);
   }

   private void set(Vector3d newTranslation)
   {
      yoFramePose.setXYZ(newTranslation.getX(), newTranslation.getY(), newTranslation.getZ());
   }

   public void interpolate(YoReferencePose start, YoReferencePose goal, double alpha)
   {
      start.getTransformToDesiredFrame(interpolationStartingPosition, parentFrame);
      goal.getTransformToDesiredFrame(interpolationGoalPosition, parentFrame);

      transformInterpolationCalculator.computeInterpolation(interpolationStartingPosition, interpolationGoalPosition, output, alpha);
      setAndUpdate(output);
   }

   public void get(Quat4d rotation)
   {
      yoFramePose.getOrientation().getQuaternion(rotation);
   }

   public void get(Vector3d translation)
   {
      yoFramePose.getPosition().get(translation);
   }
}