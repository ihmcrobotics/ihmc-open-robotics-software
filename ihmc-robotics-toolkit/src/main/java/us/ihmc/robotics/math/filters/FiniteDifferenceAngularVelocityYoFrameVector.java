package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class FiniteDifferenceAngularVelocityYoFrameVector extends YoFrameVector
{
   private final YoFrameQuaternion orientation;
   private final YoFrameQuaternion orientationPreviousValue;

   private final YoBoolean hasBeenCalled;

   private final RotationMatrix currentOrientationMatrix = new RotationMatrix();
   private final RotationMatrix previousOrientationMatrix = new RotationMatrix();
   private final RotationMatrix deltaOrientationMatrix = new RotationMatrix();
   private final AxisAngle deltaAxisAngle = new AxisAngle();

   private final double dt;

   public FiniteDifferenceAngularVelocityYoFrameVector(String namePrefix, ReferenceFrame referenceFrame, double dt, YoVariableRegistry registry)
   {
      this(namePrefix, null, referenceFrame, dt, registry);
   }

   public FiniteDifferenceAngularVelocityYoFrameVector(String namePrefix, YoFrameQuaternion orientationToDifferentiate, double dt, YoVariableRegistry registry)
   {
      this(namePrefix, orientationToDifferentiate, orientationToDifferentiate.getReferenceFrame(), dt, registry);
   }

   private FiniteDifferenceAngularVelocityYoFrameVector(String namePrefix, YoFrameQuaternion orientationToDifferentiate, ReferenceFrame referenceFrame, double dt, YoVariableRegistry registry)
   {
      super(namePrefix, referenceFrame, registry);

      this.dt = dt;

      orientation = orientationToDifferentiate;
      orientationPreviousValue = new YoFrameQuaternion(namePrefix + "_previous", referenceFrame, registry);

      hasBeenCalled = new YoBoolean(namePrefix + "HasBeenCalled", registry);
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (orientation == null)
      {
         throw new NullPointerException("FiniteDifferenceAngularVelocityYoFrameVector must be constructed with a non null "
               + "orientation variable to call update(), otherwise use update(FrameOrientation)");
      }

      orientation.get(currentOrientationMatrix);
      update(currentOrientationMatrix);
   }

   public void update(FrameQuaternion currentOrientation)
   {
      checkReferenceFrameMatch(currentOrientation);

      currentOrientationMatrix.set(currentOrientation);
      update(currentOrientationMatrix);
   }

   public void update(Quaternion currentOrientation)
   {
      currentOrientationMatrix.set(currentOrientation);
      update(currentOrientationMatrix);
   }

   public void update(AxisAngle currentOrientation)
   {
      currentOrientationMatrix.set(currentOrientation);
      update(currentOrientationMatrix);
   }

   public void update(RotationMatrix rotationMatrix)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         orientationPreviousValue.set(rotationMatrix);
         hasBeenCalled.set(true);
      }

      if (rotationMatrix != currentOrientationMatrix)
         currentOrientationMatrix.set(rotationMatrix);
      orientationPreviousValue.get(previousOrientationMatrix);
      deltaOrientationMatrix.set(currentOrientationMatrix);
      deltaOrientationMatrix.multiplyTransposeOther(previousOrientationMatrix);
      deltaAxisAngle.set(deltaOrientationMatrix);

      set(deltaAxisAngle.getX(), deltaAxisAngle.getY(), deltaAxisAngle.getZ());
      scale(deltaAxisAngle.getAngle() / dt);

      orientationPreviousValue.set(currentOrientationMatrix);
   }
}
