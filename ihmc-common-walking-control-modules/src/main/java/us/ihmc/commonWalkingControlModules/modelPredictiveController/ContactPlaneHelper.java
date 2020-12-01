package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
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

   private final DMatrixRMaj rhoMaxMatrix;

   private final DMatrixRMaj linearPositionJacobianMatrix;
   private final DMatrixRMaj linearVelocityJacobianMatrix;
   private final DMatrixRMaj linearAccelerationJacobianMatrix;
   private final DMatrixRMaj linearJerkJacobianMatrix;

   private final DMatrixRMaj rhoMagnitudeJacobianMatrix;
   private final DMatrixRMaj rhoRateJacobianMatrix;
   private final DMatrixRMaj rhoAccelerationJacobianMatrix;
   private final DMatrixRMaj rhoJerkJacobianMatrix;

   private final DMatrixRMaj accelerationIntegrationHessian;
   private final DMatrixRMaj accelerationIntegrationGradient;
   private final DMatrixRMaj jerkIntegrationHessian;

   private final FrameVector3D contactAcceleration = new FrameVector3D();
   private final FramePoint3D contactCentroid = new FramePoint3D();

   private ContactPlaneForceViewer viewer;

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

      rhoMaxMatrix = new DMatrixRMaj(rhoSize, 1);

      linearPositionJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearVelocityJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearAccelerationJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearJerkJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);

      rhoMagnitudeJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      rhoRateJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      rhoAccelerationJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);
      rhoJerkJacobianMatrix = new DMatrixRMaj(rhoSize, coefficientsSize);

      accelerationIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);
      accelerationIntegrationGradient = new DMatrixRMaj(coefficientsSize, 1);
      jerkIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);

      contactWrenchCoefficientMatrix = new DMatrixRMaj(3, 4);
   }


   public void setContactPointForceViewer(ContactPlaneForceViewer viewer)
   {
      this.viewer = viewer;

      for (ContactPointHelper pointHelper : contactPoints)
         pointHelper.setContactPointForceViewer(viewer.getNextPointForceViewer());
   }

   public void setMaxNormalForce(double maxNormalForce)
   {
      double pointNormalForce = maxNormalForce / numberOfContactPoints;
      for (int i = 0; i < numberOfContactPoints; i++)
         contactPoints[i].setMaxNormalForce(pointNormalForce);
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

   public DMatrixRMaj getLinearPositionJacobian()
   {
      return linearPositionJacobianMatrix;
   }

   public DMatrixRMaj getLinearVelocityJacobian()
   {
      return linearVelocityJacobianMatrix;
   }

   public DMatrixRMaj getLinearAccelerationJacobian()
   {
      return linearAccelerationJacobianMatrix;
   }

   public DMatrixRMaj getLinearJerkJacobian()
   {
      return linearJerkJacobianMatrix;
   }

   public DMatrixRMaj getRhoMagnitudeJacobian()
   {
      return rhoMagnitudeJacobianMatrix;
   }

   public DMatrixRMaj getRhoRateJacobian()
   {
      return rhoRateJacobianMatrix;
   }

   public DMatrixRMaj getRhoAccelerationJacobian()
   {
      return rhoAccelerationJacobianMatrix;
   }

   public DMatrixRMaj getRhoJerkJacobian()
   {
      return rhoJerkJacobianMatrix;
   }

   public DMatrixRMaj getRhoMaxMatrix()
   {
      return rhoMaxMatrix;
   }

   public DMatrixRMaj getAccelerationIntegrationHessian()
   {
      return accelerationIntegrationHessian;
   }

   public DMatrixRMaj getAccelerationIntegrationGradient()
   {
      return accelerationIntegrationGradient;
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

      rhoMaxMatrix.reshape(rhoSize, 1);
      rhoMaxMatrix.zero();

      linearPositionJacobianMatrix.reshape(3, coefficientSize);
      linearVelocityJacobianMatrix.reshape(3, coefficientSize);
      linearAccelerationJacobianMatrix.reshape(3, coefficientSize);
      linearJerkJacobianMatrix.reshape(3, coefficientSize);

      rhoMagnitudeJacobianMatrix.reshape(rhoSize, coefficientSize);
      rhoRateJacobianMatrix.reshape(rhoSize, coefficientSize);
      rhoAccelerationJacobianMatrix.reshape(rhoSize, coefficientSize);
      rhoJerkJacobianMatrix.reshape(rhoSize, coefficientSize);

      int contactPointIndex = 0;
      int rowStart = 0;

      for (; contactPointIndex < numberOfContactPoints; contactPointIndex++)
      {
         Point2DReadOnly contactPointLocation = contactPointsInPlaneFrame.getVertex(contactPointIndex);


         // rotate each friction cone approximation to point one vector towards the center of the foot
         point.set(contactPointsInPlaneFrame.getVertex(contactPointIndex));
         double angleOffset = coneRotationCalculator.computeConeRotation(contactPointsInPlaneFrame, point);

         ContactPointHelper contactPoint = contactPoints[contactPointIndex];
         contactPoint.computeBasisVectors(contactPointLocation, framePose, angleOffset, mu);
         MatrixTools.setMatrixBlock(rhoMaxMatrix, rowStart, 0, contactPoint.getRhoMaxMatrix(), 0, 0, contactPoint.getRhoSize(), 1, 1.0);


         rowStart += contactPoint.getRhoSize();
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
      for (int contactPointIdx = 0; contactPointIdx < numberOfContactPoints; contactPointIdx++)
      {
         ContactPointHelper contactPoint = contactPoints[contactPointIdx];
         contactPoint.computeJacobians(time, omega);

         for (int derivativeOrder = 0; derivativeOrder < 4; derivativeOrder++)
         {
            MatrixTools.setMatrixBlock(getLinearJacobian(derivativeOrder),
                                       0,
                                       columnStart,
                                       contactPoint.getLinearJacobian(derivativeOrder),
                                       0,
                                       0,
                                       3,
                                       contactPoint.getCoefficientsSize(),
                                       1.0);

            MatrixTools.setMatrixBlock(getRhoJacobian(derivativeOrder),
                                       rowStart,
                                       columnStart,
                                       contactPoint.getRhoJacobian(derivativeOrder),
                                       0,
                                       0,
                                       contactPoint.getRhoSize(),
                                       contactPoint.getCoefficientsSize(),
                                       1.0);
         }
         rowStart += contactPoint.getRhoSize();
         columnStart += contactPoint.getCoefficientsSize();
      }

      jacobiansNeedUpdating = false;
      timeOfContact = time;
   }

   public void computeAccelerationIntegrationMatrix(double duration, double omega, double goalValueForPlane)
   {
      double goalValueForPoint = goalValueForPlane / numberOfContactPoints;
      int startIdx = 0;
      for (int contactPointIdx = 0; contactPointIdx < numberOfContactPoints; contactPointIdx++)
      {
         ContactPointHelper contactPoint = contactPoints[contactPointIdx];
         contactPoint.computeAccelerationIntegrationMatrix(duration, omega, goalValueForPoint);

         MatrixTools.setMatrixBlock(accelerationIntegrationHessian,
                                    startIdx,
                                    startIdx,
                                    contactPoint.getAccelerationIntegrationHessian(),
                                    0,
                                    0,
                                    contactPoint.getCoefficientsSize(),
                                    contactPoint.getCoefficientsSize(),
                                    1.0);

         MatrixTools.setMatrixBlock(accelerationIntegrationGradient,
                                    startIdx,
                                    0,
                                    contactPoint.getAccelerationIntegrationGradient(),
                                    0,
                                    0,
                                    contactPoint.getCoefficientsSize(),
                                    1,
                                    1.0);

         startIdx += contactPoint.getCoefficientsSize();
      }
   }

   public void computeJerkIntegrationMatrix(double duration, double omega)
   {
      int startIdx = 0;
      for (int contactPointIdx = 0; contactPointIdx < numberOfContactPoints; contactPointIdx++)
      {
         ContactPointHelper contactPoint = contactPoints[contactPointIdx];
         contactPoint.computeJerkIntegrationMatrix(duration, omega);

         MatrixTools.setMatrixBlock(jerkIntegrationHessian,
                                    startIdx,
                                    startIdx,
                                    contactPoint.getJerkIntegrationHessian(),
                                    0,
                                    0,
                                    contactPoint.getCoefficientsSize(),
                                    contactPoint.getCoefficientsSize(),
                                    1.0);

         startIdx += contactPoint.getCoefficientsSize();
      }
   }

   private void clear(int contactPointIndex)
   {
      contactPoints[contactPointIndex].clear();
   }

   public void computeContactForceCoefficientMatrix(DMatrixRMaj solutionVector, int solutionStartIdx)
   {
      int startIdx = solutionStartIdx;
      contactWrenchCoefficientMatrix.zero();
      for (int contactPointIdx = 0; contactPointIdx < numberOfContactPoints; contactPointIdx++)
      {
         ContactPointHelper contactPointHelper = contactPoints[contactPointIdx];
         contactPointHelper.computeContactForceCoefficientMatrix(solutionVector, startIdx);
         CommonOps_DDRM.addEquals(contactWrenchCoefficientMatrix, contactPointHelper.getContactWrenchCoefficientMatrix());
         startIdx += contactPointHelper.getCoefficientsSize();
      }
   }

   public DMatrixRMaj getContactWrenchCoefficientMatrix()
   {
      return contactWrenchCoefficientMatrix;
   }

   public void computeContactForce(double omega, double time)
   {
      contactAcceleration.setToZero();
      contactCentroid.setToZero();
      for (int i = 0; i < numberOfContactPoints; i++)
      {
         ContactPointHelper contactPoint = contactPoints[i];
         contactPoint.computeContactForce(omega, time);
         contactAcceleration.add(contactPoint.getContactAcceleration());
         contactCentroid.scaleAdd(0.25, contactPoint.getBasisVectorOrigin(), contactCentroid);
      }

      if (viewer != null)
         viewer.update(contactCentroid, contactAcceleration);
   }
}
