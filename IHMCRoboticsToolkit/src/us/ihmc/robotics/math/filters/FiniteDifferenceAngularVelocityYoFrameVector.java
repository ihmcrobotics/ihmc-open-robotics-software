package us.ihmc.robotics.math.filters;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

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

   private final FrameOrientation currentFrameOrientation = new FrameOrientation();
   private final Matrix3d currentOrientationMatrix = new Matrix3d();
   private final Matrix3d previousOrientationMatrix = new Matrix3d();
   private final Matrix3d deltaOrientationMatrix = new Matrix3d();
   private final AxisAngle4d deltaAxisAngle = new AxisAngle4d();

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

      orientation.getFrameOrientationIncludingFrame(currentFrameOrientation);
      update(currentFrameOrientation);
   }

   public void update(FrameOrientation currentOrientation)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         orientationPreviousValue.set(currentOrientation);
         hasBeenCalled.set(true);
      }

      currentOrientation.getMatrix3d(currentOrientationMatrix);
      orientationPreviousValue.get(previousOrientationMatrix);
      deltaOrientationMatrix.mulTransposeRight(currentOrientationMatrix, previousOrientationMatrix);
      deltaAxisAngle.set(deltaOrientationMatrix);

      set(deltaAxisAngle.getX(), deltaAxisAngle.getY(), deltaAxisAngle.getZ());
      scale(deltaAxisAngle.getAngle() / dt);

      orientationPreviousValue.set(currentOrientation);
   }
}
