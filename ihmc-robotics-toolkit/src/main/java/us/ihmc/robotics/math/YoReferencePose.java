package us.ihmc.robotics.math;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.kinematics.TransformInterpolationCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoReferencePose extends ReferenceFrame
{
   private final YoFramePose3D yoFramePose;

   //Below are used for interpolation only
   private final RigidBodyTransform interpolationStartingPosition = new RigidBodyTransform();
   private final RigidBodyTransform interpolationGoalPosition = new RigidBodyTransform();
   private final RigidBodyTransform output = new RigidBodyTransform();

   public YoReferencePose(String frameName, ReferenceFrame parentFrame, YoRegistry registry)
   {
      super(frameName, parentFrame);
      yoFramePose = new YoFramePose3D(frameName + "_", this, registry);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.getRotation().set(yoFramePose.getOrientation());
      YoFramePoint3D yoFramePoint = yoFramePose.getPosition();
      transformToParent.getTranslation().set(yoFramePoint.getX(), yoFramePoint.getY(), yoFramePoint.getZ());
   }

   public void setAndUpdate(RigidBodyTransformReadOnly transform)
   {
      setAndUpdate(transform.getRotation(), transform.getTranslation());
   }

   public void setAndUpdate(Tuple3DReadOnly newTranslation)
   {
      setTranslation(newTranslation);
      update();
   }

   public void setAndUpdate(Orientation3DReadOnly newRotation)
   {
      setRotation(newRotation);
      update();
   }

   public void setAndUpdate(Orientation3DReadOnly newRotation, Tuple3DReadOnly newTranslation)
   {
      setRotation(newRotation);
      setTranslation(newTranslation);
      update();
   }

   private void setRotation(Orientation3DReadOnly newRotation)
   {
      yoFramePose.getRotation().set(newRotation);
   }

   private void setTranslation(Tuple3DReadOnly newTranslation)
   {
      yoFramePose.getPosition().set(newTranslation);
   }

   public void interpolate(YoReferencePose start, YoReferencePose goal, double alpha)
   {
      start.getTransformToDesiredFrame(interpolationStartingPosition, getParent());
      goal.getTransformToDesiredFrame(interpolationGoalPosition, getParent());

      output.interpolate(interpolationStartingPosition, interpolationGoalPosition, alpha);
      setAndUpdate(output);
   }

   public void getRotation(Orientation3DBasics rotation)
   {
      rotation.set(yoFramePose.getOrientation());
   }

   public void getTranslation(Tuple3DBasics translation)
   {
      translation.set(yoFramePose.getPosition());
   }
}