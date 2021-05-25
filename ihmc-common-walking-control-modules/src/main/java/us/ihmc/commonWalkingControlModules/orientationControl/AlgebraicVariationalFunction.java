package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;

public class AlgebraicVariationalFunction implements VariationalFunction
{
   private final DMatrixRMaj P = new DMatrixRMaj(6, 6);

   private final DMatrixRMaj BTransposeP = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj K = new DMatrixRMaj(3, 6);

   private final VariationalDynamicsCalculator dynamicsCalculator = new VariationalDynamicsCalculator();
   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   public void setDesired(QuaternionReadOnly desiredRotation,
                          Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame,
                          Vector3DReadOnly desiredNetAngularMomentum,
                          VariationalCommonValues commonValues)
   {
      setDesired(desiredRotation,
                 desiredBodyAngularVelocityInBodyFrame,
                 desiredNetAngularMomentum,
                 commonValues.getQ(),
                 commonValues.getR(),
                 commonValues.getRInverse(),
                 commonValues.getInertia(),
                 commonValues.getInertiaInverse());
   }

   public void setDesired(QuaternionReadOnly desiredRotation,
                          Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame,
                          Vector3DReadOnly desiredNetAngularMomentum,
                          DMatrixRMaj Q,
                          DMatrixRMaj R,
                          DMatrixRMaj RInverse,
                          DMatrixRMaj inertia,
                          DMatrixRMaj inertiaInverse)
   {
      dynamicsCalculator.compute(desiredRotation,
                                 desiredBodyAngularVelocityInBodyFrame,
                                 desiredNetAngularMomentum,
                                 inertia,
                                 inertiaInverse);

      careSolver.setMatrices(dynamicsCalculator.getA(), dynamicsCalculator.getB(), null, Q, R);
      P.set(careSolver.getP());

      CommonOps_DDRM.multTransA(dynamicsCalculator.getB(), P, BTransposeP);
      CommonOps_DDRM.mult(RInverse, BTransposeP, K);
   }

   public DMatrixRMaj getP()
   {
      return P;
   }

   public DMatrixRMaj getB()
   {
      return dynamicsCalculator.getB();
   }

   public DMatrixRMaj getK()
   {
      return K;
   }

   @Override
   public void compute(double timeInState, DMatrixRMaj PToPack, DMatrixRMaj KToPack)
   {
      PToPack.set(P);
      KToPack.set(K);
   }
}
