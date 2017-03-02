package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FiniteDifferenceAngularVelocityYoFrameVector extends YoFrameVector
{
   private final YoFrameQuaternion orientation;
   private final YoFrameQuaternion orientationPreviousValue;

   private final BooleanYoVariable hasBeenCalled;

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

      hasBeenCalled = new BooleanYoVariable(namePrefix + "HasBeenCalled", registry);
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

   public void update(FrameOrientation currentOrientation)
   {
      checkReferenceFrameMatch(currentOrientation);

      currentOrientation.getMatrix3d(currentOrientationMatrix);
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
