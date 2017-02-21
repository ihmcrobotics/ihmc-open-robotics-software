package us.ihmc.robotics.math;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.kinematics.TransformInterpolationCalculator;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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
   private final Quaternion rotation = new Quaternion();
   private final Vector3D translation = new Vector3D();

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

   public void setAndUpdate(Vector3D newTranslation)
   {
      set(newTranslation);
      update();
   }

   public void setAndUpdate(Quaternion newRotation)
   {
      set(newRotation);
      update();
   }

   public void setAndUpdate(Quaternion newRotation, Vector3D newTranslation)
   {
      set(newRotation);
      set(newTranslation);
      update();
   }

   private void set(Quaternion newRotation)
   {
      yoFramePose.setOrientation(newRotation);
   }

   private void set(Vector3D newTranslation)
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

   public void get(Quaternion rotation)
   {
      yoFramePose.getOrientation().getQuaternion(rotation);
   }

   public void get(Vector3D translation)
   {
      yoFramePose.getPosition().get(translation);
   }
}