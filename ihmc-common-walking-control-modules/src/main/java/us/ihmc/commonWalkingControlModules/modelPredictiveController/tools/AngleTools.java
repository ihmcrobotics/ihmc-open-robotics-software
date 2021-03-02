package us.ihmc.commonWalkingControlModules.modelPredictiveController.tools;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class AngleTools
{
   private final Matrix3D rotationA = new Matrix3D();
   private final Matrix3D rotationB = new Matrix3D();

   private final Matrix3D transformedRotationA = new Matrix3D();
   private final Matrix3D transformedRotationB = new Matrix3D();

   public void computeRotationError(FrameOrientation3DReadOnly orientationA, FrameQuaternionReadOnly orientationB, Vector3DBasics errorVectorToPack)
   {
      orientationA.get(rotationA);
      orientationB.get(rotationB);

      computeRotationError(rotationA, rotationB, errorVectorToPack);
   }

   public void computeRotationError(Matrix3DReadOnly rotationA, Matrix3DReadOnly rotationB, Vector3DBasics errorVectorToPack)
   {
      rotationA.inverseTransform(rotationB, transformedRotationB);
      rotationB.inverseTransform(rotationA, transformedRotationA);
      transformedRotationB.sub(transformedRotationA);
      transformedRotationB.scale(0.5);
      errorVectorToPack.setX(transformedRotationB.getM21());
      errorVectorToPack.setY(transformedRotationB.getM02());
      errorVectorToPack.setZ(transformedRotationB.getM10());
   }
}
