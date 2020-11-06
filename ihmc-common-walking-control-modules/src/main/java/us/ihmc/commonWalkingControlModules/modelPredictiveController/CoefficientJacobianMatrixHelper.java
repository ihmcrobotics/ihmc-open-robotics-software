package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class CoefficientJacobianMatrixHelper
{
   private final int maxNumberOfContactPoints;
   private final int numberOfBasisVectorsPerContactPoint;

   private final DMatrixRMaj positionJacobianMatrix;
   private final DMatrixRMaj velocityJacobianMatrix;
   private final DMatrixRMaj accelerationJacobianMatrix;
   
   public CoefficientJacobianMatrixHelper(int maxNumberOfContactPoints, int numberOfBasisVectorsPerContactPoint)
   {
      this.maxNumberOfContactPoints = maxNumberOfContactPoints;
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;

      int rhoSize = maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;

      positionJacobianMatrix = new DMatrixRMaj(rhoSize, rhoSize * MPCIndexHandler.coefficientsPerRho);
      velocityJacobianMatrix = new DMatrixRMaj(rhoSize, rhoSize * MPCIndexHandler.coefficientsPerRho);
      accelerationJacobianMatrix = new DMatrixRMaj(rhoSize, rhoSize * MPCIndexHandler.coefficientsPerRho);
   }

   public void computeMatrices(int numberOfContactPointsInContact, double timeOfContact, double omega)
   {
      if (numberOfContactPointsInContact > maxNumberOfContactPoints)
         throw new RuntimeException("Unhandled number of contact points: " + numberOfContactPointsInContact);


      int rhoSize = numberOfContactPointsInContact * numberOfBasisVectorsPerContactPoint;
      positionJacobianMatrix.reshape(rhoSize, rhoSize * MPCIndexHandler.coefficientsPerRho);
      velocityJacobianMatrix.reshape(rhoSize, rhoSize * MPCIndexHandler.coefficientsPerRho);
      accelerationJacobianMatrix.reshape(rhoSize, rhoSize * MPCIndexHandler.coefficientsPerRho);

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

   }

}
