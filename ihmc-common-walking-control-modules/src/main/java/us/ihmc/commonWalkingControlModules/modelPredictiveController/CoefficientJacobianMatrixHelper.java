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
   }

   public void reshape(int numberOfContactPointsInContact)
   {
      rhoSize = numberOfContactPointsInContact * numberOfBasisVectorsPerContactPoint;
      coefficientsSize = rhoSize * MPCIndexHandler.coefficientsPerRho;
      positionJacobianMatrix.reshape(rhoSize, coefficientsSize);
      velocityJacobianMatrix.reshape(rhoSize, coefficientsSize);
      accelerationJacobianMatrix.reshape(rhoSize, coefficientsSize);
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
      double negativeExponential = Math.exp(-omega * timeOfContact);
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
}
