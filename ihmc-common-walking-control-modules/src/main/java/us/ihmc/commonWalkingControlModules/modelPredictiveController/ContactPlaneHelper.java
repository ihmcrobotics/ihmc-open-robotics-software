package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class ContactPlaneHelper
{
   private final int maxNumberOfContactPoints;
   private final int numberOfBasisVectorsPerContactPoint;
   private int numberOfContactPoints;
   private int coefficientSize;
   private int rhoSize;
   private final ContactPointHelper[] contactPoints;
   private final PoseReferenceFrame planeFrame;

   private final FrictionConeRotationCalculator coneRotationCalculator;

   private final DMatrixRMaj contactWrenchCoefficientMatrix;

   private boolean jacobiansNeedUpdating = true;
   private double timeOfContact = Double.NaN;

   private final DMatrixRMaj positionJacobianMatrix;
   private final DMatrixRMaj velocityJacobianMatrix;
   private final DMatrixRMaj accelerationJacobianMatrix;
   private final DMatrixRMaj jerkJacobianMatrix;
   private final DMatrixRMaj accelerationIntegrationHessian;
   private final DMatrixRMaj accelerationIntegrationGradient;
   private final DMatrixRMaj jerkIntegrationHessian;

   public ContactPlaneHelper(int maxNumberOfContactPoints, int numberOfBasisVectorsPerContactPoint, FrictionConeRotationCalculator coneRotationCalculator)
   {
      this.maxNumberOfContactPoints = maxNumberOfContactPoints;
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      this.coneRotationCalculator = coneRotationCalculator;

      numberOfContactPoints = maxNumberOfContactPoints;
      rhoSize = maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      coefficientSize = MPCIndexHandler.coefficientsPerRho * maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;

      contactPoints = new ContactPointHelper[maxNumberOfContactPoints];
      planeFrame = new PoseReferenceFrame("ContactFrame", ReferenceFrame.getWorldFrame());

      for (int i = 0; i < numberOfContactPoints; i++)
      {
         contactPoints[i] = new ContactPointHelper(numberOfBasisVectorsPerContactPoint);
      }

      int coefficientsSize = MPCIndexHandler.coefficientsPerRho * rhoSize;
      positionJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      velocityJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      accelerationJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      jerkJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      accelerationIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);
      accelerationIntegrationGradient = new DMatrixRMaj(coefficientsSize, 1);
      jerkIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);

      contactWrenchCoefficientMatrix = new DMatrixRMaj(3, 4);
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public int getCoefficientSize()
   {
      return coefficientSize;
   }

   public ContactPointHelper getContactPointHelper(int index)
   {
      return contactPoints[index];
   }

   public DMatrixRMaj getJacobian(int derivativeOrder)
   {
      switch (derivativeOrder)
      {
         case 0:
            return getPositionJacobian();
         case 1:
            return getVelocityJacobian();
         case 2:
            return getAccelerationJacobian();
         case 3:
            return getJerkJacobian();
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }

   public DMatrixRMaj getPositionJacobian()
   {
      return positionJacobianMatrix;
   }

   public DMatrixRMaj getVelocityJacobian()
   {
      return velocityJacobianMatrix;
   }

   public DMatrixRMaj getAccelerationJacobian()
   {
      return accelerationJacobianMatrix;
   }

   public DMatrixRMaj getJerkJacobian()
   {
      return jerkJacobianMatrix;
   }

   private final Point3D point = new Point3D();

   public void computeBasisVectors(ConvexPolygon2DReadOnly contactPointsInPlaneFrame, FramePose3DReadOnly framePose, double mu)
   {
      jacobiansNeedUpdating = true;
      numberOfContactPoints = contactPointsInPlaneFrame.getNumberOfVertices();
      if (numberOfContactPoints > maxNumberOfContactPoints)
         throw new RuntimeException("Unhandled number of contact points: " + numberOfContactPoints);

      planeFrame.setPoseAndUpdate(framePose);

      rhoSize = numberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      coefficientSize = MPCIndexHandler.coefficientsPerRho * rhoSize;

      int contactPointIndex = 0;

      for (; contactPointIndex < numberOfContactPoints; contactPointIndex++)
      {
         Point2DReadOnly contactPoint = contactPointsInPlaneFrame.getVertex(contactPointIndex);

         // rotate each friction cone approximation to point one vector towards the center of the foot
         point.set(contactPointsInPlaneFrame.getVertex(contactPointIndex));
         double angleOffset = coneRotationCalculator.computeConeRotation(contactPointsInPlaneFrame, point);

         contactPoints[contactPointIndex].computeBasisVectors(contactPoint, framePose, angleOffset, mu);
      }

      // Should not get there as long as the number of contact points of the contactable body is less or equal to maxNumberOfContactPoints.
      for (; contactPointIndex < maxNumberOfContactPoints; contactPointIndex++)
         clear(contactPointIndex);
   }

   public void computeJacobians(double time, double omega)
   {
      if (!MathTools.epsilonEquals(time, timeOfContact, 1e-5))
         jacobiansNeedUpdating = true;

      if (!jacobiansNeedUpdating)
         return;

      positionJacobianMatrix.zero();
      velocityJacobianMatrix.zero();
      accelerationJacobianMatrix.zero();
      jerkJacobianMatrix.zero();

      int columnStart = 0;
      for (int contactPointIdx = 0; contactPointIdx < numberOfContactPoints; contactPointIdx++)
      {
         ContactPointHelper contactPoint = contactPoints[contactPointIdx];
         contactPoint.computeJacobians(time, omega);

         for (int derivativeOrder = 0; derivativeOrder < 4; derivativeOrder++)
         {
            MatrixTools.setMatrixBlock(getJacobian(derivativeOrder),
                                       0,
                                       columnStart,
                                       contactPoint.getJacobian(derivativeOrder),
                                       0,
                                       0,
                                       3,
                                       contactPoint.getCoefficientsSize(),
                                       1.0);
         }
         columnStart += contactPoint.getCoefficientsSize();
      }

      jacobiansNeedUpdating = false;
      timeOfContact = time;
   }

   private void clear(int contactPointIndex)
   {
      contactPoints[contactPointIndex].clear();
   }

   public void computeContactForceCoefficientMatrix(DMatrixRMaj solutionVector, int solutionStartIdx)
   {
      int startIdx = solutionStartIdx;
      for (int contactPointIdx = 0; contactPointIdx < numberOfContactPoints; contactPointIdx++)
      {
         ContactPointHelper contactPointHelper = contactPoints[contactPointIdx];
         for (int rhoIndex = 0; rhoIndex < contactPointHelper.getRhoSize(); rhoIndex++)
         {
            FrameVector3DReadOnly basisVector = contactPointHelper.getBasisVector(rhoIndex);
            for (int coeffIdx = 0; coeffIdx < MPCIndexHandler.coefficientsPerRho; coeffIdx++)
            {
               double rhoCoeff = solutionVector.get(startIdx++, 0);
               contactWrenchCoefficientMatrix.add(0, coeffIdx, basisVector.getX() * rhoCoeff);
               contactWrenchCoefficientMatrix.add(1, coeffIdx, basisVector.getY() * rhoCoeff);
               contactWrenchCoefficientMatrix.add(2, coeffIdx, basisVector.getZ() * rhoCoeff);
            }
         }
      }
   }

   public DMatrixRMaj getContactWrenchCoefficientMatrix()
   {
      return contactWrenchCoefficientMatrix;
   }
}
