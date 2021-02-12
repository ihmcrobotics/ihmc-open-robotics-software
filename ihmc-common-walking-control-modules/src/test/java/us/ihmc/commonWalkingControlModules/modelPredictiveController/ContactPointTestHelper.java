package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLongTime;

public class ContactPointTestHelper
{
   private final DMatrixRMaj linearPositionJacobianMatrix;
   private final DMatrixRMaj linearVelocityJacobianMatrix;
   private final DMatrixRMaj linearAccelerationJacobianMatrix;
   private final DMatrixRMaj linearJerkJacobianMatrix;

   private final DMatrixRMaj rhoMagnitudeJacobianMatrix;
   private final DMatrixRMaj rhoRateJacobianMatrix;
   private final DMatrixRMaj rhoAccelerationJacobianMatrix;
   private final DMatrixRMaj rhoJerkJacobianMatrix;

   private final int numberOfBasisVectorsPerContactPoint;
   private final ContactPointHelper contactPoint;

   public ContactPointTestHelper(ContactPointHelper contactPoint, int numberOfBasisVectorsPerContactPoint)
   {
      this.contactPoint = contactPoint;
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;

      int coefficientsSize = contactPoint.getCoefficientsSize();
      int rhoSize = contactPoint.getRhoSize();
      linearPositionJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearVelocityJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearAccelerationJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearJerkJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);

      rhoMagnitudeJacobianMatrix = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint, coefficientsSize);
      rhoRateJacobianMatrix = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint, coefficientsSize);
      rhoAccelerationJacobianMatrix = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint, coefficientsSize);
      rhoJerkJacobianMatrix = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint, coefficientsSize);



   }

   /**
    * Computes the Jacobians at time {@param time} that map from the coefficient values to the motion function value.
    *
    * If this has been called for the same time and the same basis vectors, the Jacobians are not recomputed to save computation.
    *
    * @param time time to compute the function
    * @param omega time constant for the motion function
    */
   public void computeJacobians(double time, double omega)
   {
      time = Math.min(time, sufficientlyLongTime);

      linearPositionJacobianMatrix.zero();
      linearVelocityJacobianMatrix.zero();
      linearAccelerationJacobianMatrix.zero();
      linearJerkJacobianMatrix.zero();

      rhoMagnitudeJacobianMatrix.zero();
      rhoRateJacobianMatrix.zero();
      rhoAccelerationJacobianMatrix.zero();
      rhoJerkJacobianMatrix.zero();

      double t2 = time * time;
      double t3 = time * t2;
      double positiveExponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double negativeExponential = 1.0 / positiveExponential;
      double firstVelocityCoefficient = omega * positiveExponential;
      double secondVelocityCoefficient = -omega * negativeExponential;
      double firstAccelerationCoefficient = omega * firstVelocityCoefficient;
      double secondAccelerationCoefficient = -omega * secondVelocityCoefficient;
      double firstJerkCoefficient = omega * firstAccelerationCoefficient;
      double secondJerkCoefficient = -omega * secondAccelerationCoefficient;
      boolean setTimeCoefficients = !MathTools.epsilonEquals(time, 0.0, 1e-4);
      double thirdVelocityCoefficient = 3 * t2;
      double fourthVelocityCoefficient = 2 * time;
      double thirdAccelerationCoefficient = 6 * time;

      for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
      {
         int startColumn = basisVectorIndex * LinearMPCIndexHandler.coefficientsPerRho;
         FrameVector3DReadOnly basisVector = contactPoint.getBasisVector(basisVectorIndex);

         rhoMagnitudeJacobianMatrix.unsafe_set(basisVectorIndex, startColumn, positiveExponential);
         rhoMagnitudeJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 1, negativeExponential);

         rhoRateJacobianMatrix.unsafe_set(basisVectorIndex, startColumn, firstVelocityCoefficient);
         rhoRateJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 1, secondVelocityCoefficient);

         rhoAccelerationJacobianMatrix.unsafe_set(basisVectorIndex, startColumn, firstAccelerationCoefficient);
         rhoAccelerationJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 1, secondAccelerationCoefficient);
         rhoAccelerationJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 3, 2.0);

         rhoJerkJacobianMatrix.unsafe_set(basisVectorIndex, startColumn, firstJerkCoefficient);
         rhoJerkJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 1, secondJerkCoefficient);
         rhoJerkJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 2, 6.0);

         if (setTimeCoefficients)
         {
            rhoMagnitudeJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 2, t3);
            rhoMagnitudeJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 3, t2);

            rhoRateJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 2, thirdVelocityCoefficient);
            rhoRateJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 3, fourthVelocityCoefficient);

            rhoAccelerationJacobianMatrix.unsafe_set(basisVectorIndex, startColumn + 2, thirdAccelerationCoefficient);
         }

         for (int ordinal = 0; ordinal < 3; ordinal++)
         {
            linearPositionJacobianMatrix.unsafe_set(ordinal, startColumn, basisVector.getElement(ordinal) * positiveExponential);
            linearPositionJacobianMatrix.unsafe_set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * negativeExponential);

            linearVelocityJacobianMatrix.unsafe_set(ordinal, startColumn, basisVector.getElement(ordinal) * firstVelocityCoefficient);
            linearVelocityJacobianMatrix.unsafe_set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondVelocityCoefficient);

            linearAccelerationJacobianMatrix.unsafe_set(ordinal, startColumn, basisVector.getElement(ordinal) * firstAccelerationCoefficient);
            linearAccelerationJacobianMatrix.unsafe_set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondAccelerationCoefficient);
            linearAccelerationJacobianMatrix.unsafe_set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * 2.0);

            linearJerkJacobianMatrix.unsafe_set(ordinal, startColumn, basisVector.getElement(ordinal) * firstJerkCoefficient);
            linearJerkJacobianMatrix.unsafe_set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondJerkCoefficient);
            linearJerkJacobianMatrix.unsafe_set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * 6.0);

            if (setTimeCoefficients)
            {
               linearPositionJacobianMatrix.unsafe_set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * t3);
               linearPositionJacobianMatrix.unsafe_set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * t2);

               linearVelocityJacobianMatrix.unsafe_set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * thirdVelocityCoefficient);
               linearVelocityJacobianMatrix.unsafe_set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * fourthVelocityCoefficient);

               linearAccelerationJacobianMatrix.unsafe_set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * thirdAccelerationCoefficient);
            }
         }
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
    * in this contact point.
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

   private DMatrixRMaj getLinearAccelerationJacobian()
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
}
