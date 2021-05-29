package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.tools.MPCAngleTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

public class VariationalLQRController
{
   private static final double defaultQR = 1100;
   private static final double defaultQw = 5;
   private static final double defaultR = 1.25;

   private final MPCAngleTools angleTools = new MPCAngleTools();

   private final VariationalCommonValues commonValues = new VariationalCommonValues();

   private final Vector3DBasics axisAngleError = new Vector3D();

   private final DMatrixRMaj wBd = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj desiredRotationMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj wB = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj RBerrorVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wBerror = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj state = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj desiredTorque = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj feedbackTorque = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj deltaTau = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);

   private final AlgebraicVariationalFunction variationalFunction = new AlgebraicVariationalFunction();

   public VariationalLQRController()
   {
      CommonOps_DDRM.setIdentity(inertia);
      commonValues.setInertia(inertia);

      commonValues.computeCostMatrices(defaultQR, defaultQw, defaultR);
   }

   public void setInertia(SpatialInertiaReadOnly inertia)
   {
      inertia.getMomentOfInertia().get(this.inertia);
      commonValues.setInertia(this.inertia);
   }

   private final QuaternionBasics desiredRotation = new Quaternion();
   private final RotationMatrix tempRotation = new RotationMatrix();

   public void setDesired(QuaternionReadOnly desiredRotation, Vector3DReadOnly desiredAngularVelocityInBodyFrame, Vector3DReadOnly desiredNetAngularMomentum)
   {
      this.desiredRotation.set(desiredRotation);
      desiredRotation.get(tempRotation);
      tempRotation.get(desiredRotationMatrix);
      desiredAngularVelocityInBodyFrame.get(wBd);
      desiredNetAngularMomentum.get(desiredTorque);

      variationalFunction.setDesired(desiredRotation, desiredAngularVelocityInBodyFrame, desiredNetAngularMomentum, commonValues);
   }

   public void compute(QuaternionReadOnly currentRotation, Vector3DReadOnly currentAngularVelocityInBodyFrame)
   {
      currentAngularVelocityInBodyFrame.get(wB);

      angleTools.computeRotationError(desiredRotation, currentRotation, axisAngleError);
      axisAngleError.get(RBerrorVector);

      CommonOps_DDRM.subtract(wB, wBd, wBerror);

      MatrixTools.setMatrixBlock(state, 0, 0, RBerrorVector, 0, 0, 3, 1, 1.0);
      MatrixTools.setMatrixBlock(state, 3, 0, wBerror, 0, 0, 3, 1, 1.0);

      CommonOps_DDRM.mult(-1.0, variationalFunction.getK(), state, deltaTau);

      CommonOps_DDRM.add(deltaTau, desiredTorque, feedbackTorque);
   }

   public void getDesiredTorque(Vector3DBasics tau)
   {
      tau.set(this.feedbackTorque);
   }
}
