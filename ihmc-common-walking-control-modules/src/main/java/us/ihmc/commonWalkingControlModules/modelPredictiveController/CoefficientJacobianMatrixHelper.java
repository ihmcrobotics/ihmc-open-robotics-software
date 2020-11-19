package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;

public class CoefficientJacobianMatrixHelper
{
   private final int maxNumberOfContactPoints;
   private final int numberOfContactPointsInContact;
   private final int numberOfBasisVectorsPerContactPoint;

   private final DMatrixRMaj positionJacobianMatrix;
   private final DMatrixRMaj velocityJacobianMatrix;
   private final DMatrixRMaj accelerationJacobianMatrix;
   private final DMatrixRMaj accelerationIntegrationHessian;
   private final DMatrixRMaj jerkIntegrationHessian;

   private int rhoSize;
   private int coefficientsSize;
   private double currentTime = Double.NaN;
   
   public CoefficientJacobianMatrixHelper(int maxNumberOfContactPoints, int numberOfBasisVectorsPerContactPoint)
   {
      this.maxNumberOfContactPoints = maxNumberOfContactPoints;
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      this.numberOfContactPointsInContact = maxNumberOfContactPoints;

      rhoSize = maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      coefficientsSize = rhoSize * MPCIndexHandler.coefficientsPerRho;

      positionJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      velocityJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      accelerationJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      accelerationIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);
      jerkIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);
   }

   public void reshape(int numberOfContactPointsInContact)
   {
      rhoSize = numberOfContactPointsInContact * numberOfBasisVectorsPerContactPoint;
      coefficientsSize = rhoSize * MPCIndexHandler.coefficientsPerRho;
      positionJacobianMatrix.reshape(rhoSize, coefficientsSize);
      velocityJacobianMatrix.reshape(rhoSize, coefficientsSize);
      accelerationJacobianMatrix.reshape(rhoSize, coefficientsSize);
      accelerationIntegrationHessian.reshape(coefficientsSize, coefficientsSize);
      jerkIntegrationHessian.reshape(coefficientsSize, coefficientsSize);
   }

   public int getCoefficientSize()
   {
      return coefficientsSize;
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public void computeMatrices(double timeOfContact, double omega)
   {
      if (numberOfContactPointsInContact > maxNumberOfContactPoints)
         throw new RuntimeException("Unhandled number of contact points: " + numberOfContactPointsInContact);

      if (MathTools.epsilonEquals(currentTime, timeOfContact, 1e-5))
         return;

      int rhoIndex = 0;
      double t2 = timeOfContact * timeOfContact;
      double t3 = timeOfContact * t2;
      double positiveExponential = Math.exp(omega * timeOfContact);
      double negativeExponential = 1.0 / positiveExponential;
      double firstVelocityCoefficient = omega * positiveExponential;
      double secondVelocityCoefficient = -omega * negativeExponential;
      double firstAccelerationCoefficient = omega * firstVelocityCoefficient;
      double secondAccelerationCoefficient = -omega * secondVelocityCoefficient;
      boolean setTimeCoefficients = !MathTools.epsilonEquals(timeOfContact, 0.0, 1e-4);
      double thirdVelocityCoefficient = 3 * t2;
      double fourthVelocityCoefficient = 2 * timeOfContact;
      double thirdAccelerationCoefficient = 6 * timeOfContact;

      for (int contactPointIndex = 0; contactPointIndex < numberOfContactPointsInContact; contactPointIndex++)
      {
         for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
         {
            int startColumn = rhoIndex * MPCIndexHandler.coefficientsPerRho;

            positionJacobianMatrix.set(rhoIndex, startColumn, positiveExponential);
            positionJacobianMatrix.set(rhoIndex, startColumn + 1, negativeExponential);

            velocityJacobianMatrix.set(rhoIndex, startColumn, firstVelocityCoefficient);
            velocityJacobianMatrix.set(rhoIndex, startColumn + 1, secondVelocityCoefficient);

            accelerationJacobianMatrix.set(rhoIndex, startColumn, firstAccelerationCoefficient);
            accelerationJacobianMatrix.set(rhoIndex, startColumn + 1, secondAccelerationCoefficient);
            accelerationJacobianMatrix.set(rhoIndex, startColumn + 3, 2.0);

            if (setTimeCoefficients)
            {
               positionJacobianMatrix.set(rhoIndex, startColumn + 2, t3);
               positionJacobianMatrix.set(rhoIndex, startColumn + 3, t2);

               velocityJacobianMatrix.set(rhoIndex, startColumn + 2, thirdVelocityCoefficient);
               velocityJacobianMatrix.set(rhoIndex, startColumn + 3, fourthVelocityCoefficient);

               accelerationJacobianMatrix.set(rhoIndex, startColumn + 2, thirdAccelerationCoefficient);
            }

            rhoIndex++;
         }
      }

      currentTime = timeOfContact;
   }

   public DMatrixRMaj getJacobianMatrix(int derivativeOrder)
   {
      switch (derivativeOrder)
      {
         case 0:
            return getPositionJacobianMatrix();
         case 1:
            return getVelocityJacobianMatrix();
         case 2:
            return getAccelerationJacobianMatrix();
         default:
            throw new IllegalArgumentException("Derivative order must be less than 3.");
      }
   }

   public DMatrixRMaj getPositionJacobianMatrix()
   {
      return positionJacobianMatrix;
   }

   public DMatrixRMaj getVelocityJacobianMatrix()
   {
      return velocityJacobianMatrix;
   }

   public DMatrixRMaj getAccelerationJacobianMatrix()
   {
      return accelerationJacobianMatrix;
   }

   public void computeAccelerationIntegrationMatrix(double duration, double omega)
   {
      int rhoIndex = 0;

      double positiveExponential = Math.exp(omega * duration);
      double positiveExponential2 = positiveExponential * positiveExponential;
      double negativeExponential = 1.0 / positiveExponential;
      double negativeExponential2 = negativeExponential * negativeExponential;
      double duration2 = duration * duration;
      double duration3 = duration * duration2;
      double omega2 = omega * omega;
      double omega3 = omega * omega2;
      double omega4 = omega2 * omega2;
      double c00 = omega3 / 2.0 * (positiveExponential2 - 1.0);
      double c01 = omega4 * duration;
      double c02 = 6.0 * (positiveExponential * (omega * duration - 1.0) + 1.0);
      double c03 = 2.0 * omega * (positiveExponential - 1.0);
      double c11 = -omega3 / 2.0 * (negativeExponential2 - 1.0);
      double c12 = -6.0 * (negativeExponential * (omega * duration + 1.0) - 1.0);
      double c13 = -2.0 * omega * (negativeExponential - 1.0);
      double c22 = 12.0 * duration3;
      double c23 = 6.0 * duration2;
      double c33 = 4.0 * duration;

      for (int contactPointIndex = 0; contactPointIndex < numberOfContactPointsInContact; contactPointIndex++)
      {
         for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
         {
            int startColumn = rhoIndex * MPCIndexHandler.coefficientsPerRho;

            accelerationIntegrationHessian.set(startColumn, startColumn, c00);
            accelerationIntegrationHessian.set(startColumn, startColumn + 1, c01);
            accelerationIntegrationHessian.set(startColumn, startColumn + 2, c02);
            accelerationIntegrationHessian.set(startColumn, startColumn + 2, c03);
            accelerationIntegrationHessian.set(startColumn + 1, startColumn, c01);
            accelerationIntegrationHessian.set(startColumn + 1, startColumn + 1, c11);
            accelerationIntegrationHessian.set(startColumn + 1, startColumn + 2, c12);
            accelerationIntegrationHessian.set(startColumn + 1, startColumn + 3, c13);
            accelerationIntegrationHessian.set(startColumn + 2, startColumn, c02);
            accelerationIntegrationHessian.set(startColumn + 2, startColumn + 1, c12);
            accelerationIntegrationHessian.set(startColumn + 2, startColumn + 2, c22);
            accelerationIntegrationHessian.set(startColumn + 2, startColumn + 3, c23);
            accelerationIntegrationHessian.set(startColumn + 3, startColumn, c03);
            accelerationIntegrationHessian.set(startColumn + 3, startColumn + 1, c13);
            accelerationIntegrationHessian.set(startColumn + 3, startColumn + 2, c23);
            accelerationIntegrationHessian.set(startColumn + 3, startColumn + 3, c33);

            rhoIndex++;
         }
      }
   }

   public void computeJerkIntegrationMatrix(double duration, double omega)
   {
      int rhoIndex = 0;

      double positiveExponential = Math.exp(omega * duration);
      double positiveExponential2 = positiveExponential * positiveExponential;
      double negativeExponential = 1.0 / positiveExponential;
      double negativeExponential2 = negativeExponential * negativeExponential;
      double omega2 = omega * omega;
      double omega3 = omega * omega2;
      double omega5 = omega2 * omega3;
      double omega6 = omega3 * omega3;
      double c00 = omega5 / 2.0 * (positiveExponential2 - 1.0);
      double c01 = omega6 * duration;
      double c02 = 6.0 * omega2 * (positiveExponential - 1.0);
      double c11 = -omega5 / 2.0 * (negativeExponential2 - 1.0);
      double c12 = 6.0 * omega2 * (negativeExponential  - 1.0);
      double c22 = 36.0 * duration;

      for (int contactPointIndex = 0; contactPointIndex < numberOfContactPointsInContact; contactPointIndex++)
      {
         for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
         {
            int startColumn = rhoIndex * MPCIndexHandler.coefficientsPerRho;

            jerkIntegrationHessian.set(startColumn, startColumn, c00);
            jerkIntegrationHessian.set(startColumn, startColumn + 1, c01);
            jerkIntegrationHessian.set(startColumn, startColumn + 2, c02);
            jerkIntegrationHessian.set(startColumn + 1, startColumn, c01);
            jerkIntegrationHessian.set(startColumn + 1, startColumn + 1, c11);
            jerkIntegrationHessian.set(startColumn + 1, startColumn + 2, c12);
            jerkIntegrationHessian.set(startColumn + 2, startColumn, c02);
            jerkIntegrationHessian.set(startColumn + 2, startColumn + 1, c12);
            jerkIntegrationHessian.set(startColumn + 2, startColumn + 2, c22);

            rhoIndex++;
         }
      }
   }

}
