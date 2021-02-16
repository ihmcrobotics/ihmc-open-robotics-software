package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPlaneForceViewer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

/**
 * This class is a helper class to compute the Jacobians that map from generalized contact force coefficient values to the motion function. That is, they scale
 * the contact force along the reaction vector, and compute the value along the desired derivative order.
 */
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

   private ContactPlaneForceViewer viewer;

   public ContactPlaneHelper(int maxNumberOfContactPoints, int numberOfBasisVectorsPerContactPoint, FrictionConeRotationCalculator coneRotationCalculator)
   {
      this.maxNumberOfContactPoints = maxNumberOfContactPoints;
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      this.coneRotationCalculator = coneRotationCalculator;

      numberOfContactPoints = maxNumberOfContactPoints;
      rhoSize = maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      coefficientSize = LinearMPCIndexHandler.coefficientsPerRho * maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;

      contactPoints = new ContactPointHelper[maxNumberOfContactPoints];
      planeFrame = new PoseReferenceFrame("ContactFrame", ReferenceFrame.getWorldFrame());

      for (int i = 0; i < numberOfContactPoints; i++)
      {
         contactPoints[i] = new ContactPointHelper(numberOfBasisVectorsPerContactPoint);
      }

      int coefficientsSize = LinearMPCIndexHandler.coefficientsPerRho * rhoSize;

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

   /**
    * Sets the net maximum force allowed for the entire contact plane.
    * @param maxNormalForce maximum normal force.
    */
   public void setMaxNormalForce(double maxNormalForce)
   {
      double pointNormalForce = maxNormalForce / numberOfContactPoints;
      for (int i = 0; i < numberOfContactPoints; i++)
         contactPoints[i].setMaxNormalForce(pointNormalForce);
   }

   /**
    * Gets the total number of generalized contact force vectors in this plane.
    *
    * This value should be equal to {@code numberOfBasisVectorsPerContactPoint} times {@link #getNumberOfContactPoints()}}.
    * @return total number of vectors
    */
   public int getRhoSize()
   {
      return rhoSize;
   }

   /**
    * Gets the total number of coefficients in the generalized contact forces in this plane.
    *
    * This value should be equal to {@link LinearMPCIndexHandler#coefficientsPerRho} times {@link #getRhoSize()}.
    * @return total number of coefficients.
    */
   public int getCoefficientSize()
   {
      return coefficientSize;
   }

   /**
    * Gets the total number of contact points in this plane.
    * @return number of contact points.
    */
   public int getNumberOfContactPoints()
   {
      return numberOfContactPoints;
   }

   /**
    * Gets the {@link ContactPointHelper} for the {@param index}th contact point.
    * @param index contact point query.
    * @return {@link ContactPointHelper} for the {@param index} point.
    */
   public ContactPointHelper getContactPointHelper(int index)
   {
      return contactPoints[index];
   }

   /**
    * Returns the Jacobian that maps from the generalized contact value coefficients to the corresponding linear Euclidean motion function value for all the
    * coefficients in this contact plane.
    *
    * @param derivativeOrder order of the Euclidean motion function Jacobian to return, where position is zero.
    * @return Euclidean motion function Jacobian.
    */
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
    * Returns a vector of the maximum generalized contact values that are allowed to achieve the maximum total contact force specified by
    * {@link #setMaxNormalForce(double)}
    * @return vector of maximum generalized contact values.
    */
   public DMatrixRMaj getRhoMaxMatrix()
   {
      return rhoMaxMatrix;
   }

   /**
    * Gets the quadratic cost hessian for the cost term that aims at tracking a desired acceleration value over the segment duration.
    *
    * The returned matrix should always be square of size {@link #getCoefficientSize()}.
    * @return quadratic cost hessian.
    */
   public DMatrixRMaj getAccelerationIntegrationHessian()
   {
      return accelerationIntegrationHessian;
   }

   /**
    * Gets the quadratic cost gradient for the cost term that aims at tracking a desired acceleration value over the segment duration.
    *
    * The returned matrix should always be a vector of size {@link #getCoefficientSize()}.
    * @return quadratic cost gradient.
    */
   public DMatrixRMaj getAccelerationIntegrationGradient()
   {
      return accelerationIntegrationGradient;
   }

   private final Point3D point = new Point3D();

   /**
    * Computes the basis vectors for the contact plane along which the generalized contact values act.
    *
    * @param contactPointsInPlaneFrame origins of the basis vectors.
    * @param framePose pose of the plane, which gives the contact normal
    * @param mu coefficient of friction
    */
   public void computeBasisVectors(ConvexPolygon2DReadOnly contactPointsInPlaneFrame, FramePose3DReadOnly framePose, double mu)
   {
      jacobiansNeedUpdating = true;
      numberOfContactPoints = contactPointsInPlaneFrame.getNumberOfVertices();
      if (numberOfContactPoints > maxNumberOfContactPoints)
         throw new RuntimeException("Unhandled number of contact points: " + numberOfContactPoints);

      planeFrame.setPoseAndUpdate(framePose);

      rhoSize = numberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      coefficientSize = LinearMPCIndexHandler.coefficientsPerRho * rhoSize;

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

   /**
    * Computes the equivalent quadratic cost function components that minimize the difference from the acceleration and some net goal value for the plane over some time
    * @param duration duration for the integration
    * @param omega time constant for the motion function
    * @param goalValueForPlane nominal value for the acceleration to track.
    */
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

   /**
    * Computes the equivalent quadratic cost function components that minimize the integral of the jerk over the duration
    * @param duration duration for the integration
    * @param omega time constant for the motion function
    */
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

   /**
    * Rests the indicated contact point
    * @param contactPointIndex contact point index to reset
    */
   private void clear(int contactPointIndex)
   {
      contactPoints[contactPointIndex].clear();
   }


   /**
    * Computes the collapsed wrench function. That is, it adds all the generalized contact force functions together in Euclidean space to get a much more
    * compact time function.
    *
    * @param solutionVector vector containing all the coefficients for the generalized contact force functions
    * @param solutionStartIdx index of the vector where the coefficients for this plane start
    */
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

   /**
    * Returns the collapsed wrench function. This is the sum of all the generalized contact forces in Euclidean space. This function should be a 3x4 matrix,
    * where the rows match the corresponding Euclidean coordinate axis.
    * @return contact wrench matrix
    */
   public DMatrixRMaj getContactWrenchCoefficientMatrix()
   {
      return contactWrenchCoefficientMatrix;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ContactPlaneHelper)
      {
         ContactPlaneHelper other = (ContactPlaneHelper) object;
         if (contactPoints.length != other.contactPoints.length)
            return false;
         if (maxNumberOfContactPoints != other.maxNumberOfContactPoints)
            return false;
         if (numberOfContactPoints != other.numberOfContactPoints)
            return false;
         if (numberOfBasisVectorsPerContactPoint != other.numberOfBasisVectorsPerContactPoint)
            return false;
         if (coefficientSize != other.coefficientSize)
            return false;
         if (rhoSize != other.rhoSize)
            return false;
         if (!planeFrame.equals(other.planeFrame))
            return false;

         for (int i = 0; i < contactPoints.length; i++)
         {
            if (!contactPoints[i].equals(other.contactPoints[i]))
               return false;
         }

         return true;
      }
      else
      {
         return false;
      }
   }
}
