package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;

public class ContactPlaneTestHelper
{
   private final DMatrixRMaj linearPositionJacobianMatrix;
   private final DMatrixRMaj linearVelocityJacobianMatrix;
   private final DMatrixRMaj linearAccelerationJacobianMatrix;
   private final DMatrixRMaj linearJerkJacobianMatrix;

   private final DMatrixRMaj rhoMagnitudeJacobianMatrix;
   private final DMatrixRMaj rhoRateJacobianMatrix;
   private final DMatrixRMaj rhoAccelerationJacobianMatrix;
   private final DMatrixRMaj rhoJerkJacobianMatrix;

   private final ContactPlaneHelper contactPlane;

   private final ContactPointTestHelper[] contactPointHelpers;


   public ContactPlaneTestHelper(ContactPlaneHelper contactPlane, int numberOfBasisVectorsPerContactPoint)
   {
      this.contactPlane = contactPlane;
      int coefficientsSize = contactPlane.getCoefficientSize();
      int rhoSize = contactPlane.getRhoSize();
      linearPositionJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearVelocityJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearAccelerationJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearJerkJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);

      rhoMagnitudeJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      rhoRateJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      rhoAccelerationJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      rhoJerkJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);

      contactPointHelpers = new ContactPointTestHelper[contactPlane.getNumberOfContactPoints()];
      for (int i = 0; i < contactPlane.getNumberOfContactPoints(); i++)
      {
         contactPointHelpers[i] = new ContactPointTestHelper(contactPlane.getContactPointHelper(i), numberOfBasisVectorsPerContactPoint);
      }

   }

   public DMatrixRMaj getLinearJacobian(int derivativeOrder)
   {
      switch (derivativeOrder)
      {
         case 0:
            return getLinearPositionJacobian();
         case 1:
            return getLinearVelocityJacobian();
         case 2:
            return getLinearAccelerationJacobian();
         case 3:
            return getLinearJerkJacobian();
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }

   /**
    * Returns the Jacobian that maps from the generalized contact value coefficients to a vector of generalized contact force values for all the coefficients
    * in this contact plane.
    *
    * @param derivativeOrder order of the generalized contact forces to return, where position is zero.
    * @return vector of generalized contact values.
    */
   public DMatrixRMaj getRhoJacobian(int derivativeOrder)
   {
      switch (derivativeOrder)
      {
         case 0:
            return getRhoMagnitudeJacobian();
         case 1:
            return getRhoRateJacobian();
         case 2:
            return getRhoAccelerationJacobian();
         case 3:
            return getRhoJerkJacobian();
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }

   private DMatrixRMaj getLinearPositionJacobian()
   {
      return linearPositionJacobianMatrix;
   }

   private DMatrixRMaj getLinearVelocityJacobian()
   {
      return linearVelocityJacobianMatrix;
   }

   public DMatrixRMaj getLinearAccelerationJacobian()
   {
      return linearAccelerationJacobianMatrix;
   }

   private DMatrixRMaj getLinearJerkJacobian()
   {
      return linearJerkJacobianMatrix;
   }

   private DMatrixRMaj getRhoMagnitudeJacobian()
   {
      return rhoMagnitudeJacobianMatrix;
   }

   private DMatrixRMaj getRhoRateJacobian()
   {
      return rhoRateJacobianMatrix;
   }

   private DMatrixRMaj getRhoAccelerationJacobian()
   {
      return rhoAccelerationJacobianMatrix;
   }

   private DMatrixRMaj getRhoJerkJacobian()
   {
      return rhoJerkJacobianMatrix;
   }


   /**
    * Computes the Jacobians at time {@param time} that map from the coefficient values to the motion function value.
    * <p>
    * If this has been called for the same time and the same basis vectors, the Jacobians are not recomputed to save computation.
    *
    * @param time time to compute the function
    * @param omega time constant for the motion function
    */
   public void computeJacobians(double time, double omega)
   {
      linearPositionJacobianMatrix.zero();
      linearVelocityJacobianMatrix.zero();
      linearAccelerationJacobianMatrix.zero();
      linearJerkJacobianMatrix.zero();

      rhoMagnitudeJacobianMatrix.zero();
      rhoRateJacobianMatrix.zero();
      rhoAccelerationJacobianMatrix.zero();
      rhoJerkJacobianMatrix.zero();

      int columnStart = 0;
      int rowStart = 0;
      for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
      {
         ContactPointTestHelper contactPoint = contactPointHelpers[contactPointIdx];
         contactPoint.computeJacobians(time, omega);

         int coefficientSize = contactPlane.getContactPointHelper(contactPointIdx).getCoefficientsSize();
         int rhoSize = contactPlane.getContactPointHelper(contactPointIdx).getRhoSize();

         for (int derivativeOrder = 0; derivativeOrder < 4; derivativeOrder++)
         {
            MatrixTools.setMatrixBlock(getLinearJacobian(derivativeOrder),
                                       0,
                                       columnStart,
                                       contactPoint.getLinearJacobian(derivativeOrder),
                                       0,
                                       0,
                                       3,
                                       coefficientSize,
                                       1.0);

            MatrixTools.setMatrixBlock(getRhoJacobian(derivativeOrder),
                                       rowStart,
                                       columnStart,
                                       contactPoint.getRhoJacobian(derivativeOrder),
                                       0,
                                       0,
                                       rhoSize,
                                       coefficientSize,
                                       1.0);
         }
         rowStart += rhoSize;
         columnStart += coefficientSize;
      }
   }
}
