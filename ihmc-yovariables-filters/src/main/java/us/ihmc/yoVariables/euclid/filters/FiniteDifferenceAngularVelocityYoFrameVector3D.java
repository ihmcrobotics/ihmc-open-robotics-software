package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.VariableTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class FiniteDifferenceAngularVelocityYoFrameVector3D extends YoFrameVector3D
{
   private final YoFrameQuaternion orientation;
   private final YoFrameQuaternion orientationPreviousValue;

   private final YoBoolean hasBeenCalled;

   private final RotationMatrix currentOrientationMatrix = new RotationMatrix();
   private final RotationMatrix previousOrientationMatrix = new RotationMatrix();
   private final RotationMatrix deltaOrientationMatrix = new RotationMatrix();
   private final AxisAngle deltaAxisAngle = new AxisAngle();

   private final double dt;

   public FiniteDifferenceAngularVelocityYoFrameVector3D(String namePrefix, ReferenceFrame referenceFrame, double dt, YoRegistry registry)
   {
      this(namePrefix, null, referenceFrame, dt, registry);
   }

   public FiniteDifferenceAngularVelocityYoFrameVector3D(String namePrefix, YoFrameQuaternion orientationToDifferentiate, double dt, YoRegistry registry)
   {
      this(namePrefix, orientationToDifferentiate, orientationToDifferentiate.getReferenceFrame(), dt, registry);
   }

   private FiniteDifferenceAngularVelocityYoFrameVector3D(String namePrefix, YoFrameQuaternion orientationToDifferentiate, ReferenceFrame referenceFrame, double dt, YoRegistry registry)
   {
      super(namePrefix, referenceFrame, registry);

      this.dt = dt;

      orientation = orientationToDifferentiate;
      orientationPreviousValue = new YoFrameQuaternion(namePrefix + "_previous", referenceFrame, registry);

      hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(namePrefix, "", registry);
      hasBeenCalled.set(false);
   }

   public void update()
   {
      if (orientation == null)
      {
         throw new NullPointerException("FiniteDifferenceAngularVelocityYoFrameVector must be constructed with a non null "
               + "orientation variable to call update(), otherwise use update(FrameOrientation)");
      }

      currentOrientationMatrix.set(orientation);
      update(currentOrientationMatrix);
   }

   public void update(FrameOrientation3DReadOnly currentOrientation)
   {
      checkReferenceFrameMatch(currentOrientation);

      currentOrientationMatrix.set(currentOrientation);
      update(currentOrientationMatrix);
   }

   public void update(Orientation3DReadOnly currentOrientation)
   {
      currentOrientationMatrix.set(currentOrientation);
      update(currentOrientationMatrix);
   }

   public void update(RotationMatrixReadOnly rotationMatrix)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         orientationPreviousValue.set(rotationMatrix);
         hasBeenCalled.set(true);
      }

      if (rotationMatrix != currentOrientationMatrix)
         currentOrientationMatrix.set(rotationMatrix);
      previousOrientationMatrix.set(orientationPreviousValue);
      deltaOrientationMatrix.set(currentOrientationMatrix);
      deltaOrientationMatrix.multiplyTransposeOther(previousOrientationMatrix);
      deltaAxisAngle.set(deltaOrientationMatrix);

      set(deltaAxisAngle.getX(), deltaAxisAngle.getY(), deltaAxisAngle.getZ());
      scale(deltaAxisAngle.getAngle() / dt);

      orientationPreviousValue.set(currentOrientationMatrix);
   }
}
