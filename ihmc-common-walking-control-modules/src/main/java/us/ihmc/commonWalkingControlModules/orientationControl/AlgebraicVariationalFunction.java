package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;

public class AlgebraicVariationalFunction
{
   private final DMatrixRMaj P = new DMatrixRMaj(6, 6);

   private final VariationalDynamicsCalculator dynamicsCalculator = new VariationalDynamicsCalculator();
   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   public void setDesired(QuaternionReadOnly desiredRotation,
                          Vector3DReadOnly desiredBodyAngularVelocityInBodyFrame,
                          Vector3DReadOnly desiredNetAngularMomentum,
                          DMatrixRMaj Q,
                          DMatrixRMaj R,
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
   }

   public DMatrixRMaj getP()
   {
      return P;
   }

}
