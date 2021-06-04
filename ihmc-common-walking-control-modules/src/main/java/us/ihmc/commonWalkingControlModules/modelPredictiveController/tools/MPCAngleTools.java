package us.ihmc.commonWalkingControlModules.modelPredictiveController.tools;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class MPCAngleTools
{
   private final Matrix3D rotationA = new Matrix3D();
   private final Matrix3D rotationB = new Matrix3D();

   private final Matrix3D transformedRotationA = new Matrix3D();
   private final Matrix3D transformedRotationB = new Matrix3D();

   public void computeRotationError(Orientation3DReadOnly orientationA, Orientation3DReadOnly orientationB, Vector3DBasics errorVectorToPack)
   {
      orientationA.get(rotationA);
      orientationB.get(rotationB);

      computeRotationError(rotationA, rotationB, errorVectorToPack);
   }

   public void computeRotationError(Matrix3DReadOnly rotationA, Matrix3DReadOnly rotationB, Vector3DBasics errorVectorToPack)
   {
      transformedRotationB.set(rotationA);
      transformedRotationB.multiplyInvertThis(rotationB);
      transformedRotationA.set(rotationB);
      transformedRotationA.multiplyInvertThis(rotationA);

      transformedRotationB.sub(transformedRotationA);
      transformedRotationB.scale(0.5);

      errorVectorToPack.setX(transformedRotationB.getM21());
      errorVectorToPack.setY(transformedRotationB.getM02());
      errorVectorToPack.setZ(transformedRotationB.getM10());
   }
}
