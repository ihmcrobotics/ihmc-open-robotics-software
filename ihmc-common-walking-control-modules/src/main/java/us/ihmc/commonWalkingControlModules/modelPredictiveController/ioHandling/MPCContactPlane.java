package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPlaneForceViewer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

/**
 * This class is a helper class to compute the Jacobians that map from generalized contact force coefficient values to the motion function. That is, they scale
 * the contact force along the reaction vector, and compute the value along the desired derivative order.
 */
public class MPCContactPlane
{
   private final int maxNumberOfContactPoints;
   private final int numberOfBasisVectorsPerContactPoint;
   private int numberOfContactPoints;
   private int coefficientSize;
   private int rhoSize;
   private final MPCContactPoint[] contactPoints;
   private final PoseReferenceFrame planeFrame;

   private final FrameVector3D contactAcceleration = new FrameVector3D();
   private final FrameVector3D contactJerk = new FrameVector3D();
   private final FrictionConeRotationCalculator coneRotationCalculator;

   private final DMatrixRMaj contactWrenchCoefficientMatrix;

   private final DMatrixRMaj jerkIntegrationHessian;

   private ContactPlaneForceViewer viewer;

   public MPCContactPlane(int maxNumberOfContactPoints, int numberOfBasisVectorsPerContactPoint, FrictionConeRotationCalculator coneRotationCalculator)
   {
      this.maxNumberOfContactPoints = maxNumberOfContactPoints;
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      this.coneRotationCalculator = coneRotationCalculator;

      numberOfContactPoints = maxNumberOfContactPoints;
      rhoSize = maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      coefficientSize = LinearMPCIndexHandler.coefficientsPerRho * maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;

      contactPoints = new MPCContactPoint[maxNumberOfContactPoints];
      planeFrame = new PoseReferenceFrame("ContactFrame", ReferenceFrame.getWorldFrame());

      for (int i = 0; i < numberOfContactPoints; i++)
      {
         contactPoints[i] = new MPCContactPoint(numberOfBasisVectorsPerContactPoint);
      }

      int coefficientsSize = LinearMPCIndexHandler.coefficientsPerRho * rhoSize;

      jerkIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);

      contactWrenchCoefficientMatrix = new DMatrixRMaj(3, 4);
   }

   public void setContactPointForceViewer(ContactPlaneForceViewer viewer)
   {
      this.viewer = viewer;

      for (MPCContactPoint pointHelper : contactPoints)
         pointHelper.setContactPointForceViewer(viewer.getNextPointForceViewer());
   }

   /**
    * Gets the total number of generalized contact force vectors in this plane.
    * <p>
    * This value should be equal to {@code numberOfBasisVectorsPerContactPoint} times {@link #getNumberOfContactPoints()}}.
    *
    * @return total number of vectors
    */
   public int getRhoSize()
   {
      return rhoSize;
   }

   /**
    * Gets the total number of coefficients in the generalized contact forces in this plane.
    * <p>
    * This value should be equal to {@link LinearMPCIndexHandler#coefficientsPerRho} times {@link #getRhoSize()}.
    *
    * @return total number of coefficients.
    */
   public int getCoefficientSize()
   {
      return coefficientSize;
   }

   /**
    * Gets the total number of contact points in this plane.
    *
    * @return number of contact points.
    */
   public int getNumberOfContactPoints()
   {
      return numberOfContactPoints;
   }

   /**
    * Gets the {@link MPCContactPoint} for the {@param index}th contact point.
    *
    * @param index contact point query.
    * @return {@link MPCContactPoint} for the {@param index} point.
    */
   public MPCContactPoint getContactPointHelper(int index)
   {
      return contactPoints[index];
   }

   public double getRhoNormalZ(int index)
   {
      return contactPoints[index].getRhoNormalZ();
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
      numberOfContactPoints = contactPointsInPlaneFrame.getNumberOfVertices();
      if (numberOfContactPoints > maxNumberOfContactPoints)
         throw new RuntimeException("Unhandled number of contact points: " + numberOfContactPoints);

      planeFrame.setPoseAndUpdate(framePose);

      rhoSize = numberOfContactPoints * numberOfBasisVectorsPerContactPoint;
      coefficientSize = LinearMPCIndexHandler.coefficientsPerRho * rhoSize;

      int contactPointIndex = 0;

      for (; contactPointIndex < numberOfContactPoints; contactPointIndex++)
      {
         Point2DReadOnly contactPointLocation = contactPointsInPlaneFrame.getVertex(contactPointIndex);

         // rotate each friction cone approximation to point one vector towards the center of the foot
         point.set(contactPointsInPlaneFrame.getVertex(contactPointIndex));
         double angleOffset = coneRotationCalculator.computeConeRotation(contactPointsInPlaneFrame, point);

         MPCContactPoint contactPoint = contactPoints[contactPointIndex];
         contactPoint.computeBasisVectors(contactPointLocation, framePose, angleOffset, mu);
      }

      // Should not get there as long as the number of contact points of the contactable body is less or equal to maxNumberOfContactPoints.
      for (; contactPointIndex < maxNumberOfContactPoints; contactPointIndex++)
         clear(contactPointIndex);
   }

   /**
    * Convenience function for getting the basis vector for iterating.
    * @param basisIdx
    * @return
    */
   public FrameVector3DReadOnly getBasisVector(int basisIdx)
   {
      int pastBases = 0;
      for (int pointIdx = 0; pointIdx < getNumberOfContactPoints(); pointIdx++)
      {
         int localIdx = basisIdx - pastBases;
         MPCContactPoint contactPoint = getContactPointHelper(pointIdx);

         if (localIdx < contactPoint.getRhoSize())
            return contactPoint.getBasisVector(localIdx);

         pastBases += contactPoint.getRhoSize();
      }

      return null;
   }

   public FrameVector3DReadOnly getBasisVectorInPlaneFrame(int basisIdx)
   {
      int pastBases = 0;
      for (int pointIdx = 0; pointIdx < getNumberOfContactPoints(); pointIdx++)
      {
         int localIdx = basisIdx - pastBases;
         MPCContactPoint contactPoint = getContactPointHelper(pointIdx);

         if (localIdx < contactPoint.getRhoSize())
            return contactPoint.getBasisVectorInPlaneFrame(localIdx);

         pastBases += contactPoint.getRhoSize();
      }

      return null;
   }

   public DMatrixRMaj getBasisCoefficients(int rhoIdx)
   {
      if (rhoIdx < rhoSize)
      {
         int pointIdx = Math.floorDiv(rhoIdx, numberOfBasisVectorsPerContactPoint);
         int localIdx = rhoIdx - numberOfBasisVectorsPerContactPoint * pointIdx;

         return contactPoints[pointIdx].getBasisCoefficients(localIdx);
      }

      return null;
   }

   public FrameVector3DReadOnly getBasisMagnitude(int rhoIdx)
   {
      if (rhoIdx < rhoSize)
      {
         int pointIdx = Math.floorDiv(rhoIdx, numberOfBasisVectorsPerContactPoint);
         int localIdx = rhoIdx - numberOfBasisVectorsPerContactPoint * pointIdx;

         return contactPoints[pointIdx].getBasisMagnitude(localIdx);
      }

      return null;
   }

   /**
    * Computes the equivalent quadratic cost function components that minimize the integral of the jerk over the duration
    *
    * @param duration duration for the integration
    * @param omega time constant for the motion function
    */
   public void computeJerkIntegrationMatrix(double duration, double omega)
   {
      int startIdx = 0;
      for (int contactPointIdx = 0; contactPointIdx < numberOfContactPoints; contactPointIdx++)
      {
         MPCContactPoint contactPoint = contactPoints[contactPointIdx];
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
    *
    * @param contactPointIndex contact point index to reset
    */
   private void clear(int contactPointIndex)
   {
      contactPoints[contactPointIndex].clear();
   }

   public void reset()
   {
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
         MPCContactPoint contactPointHelper = contactPoints[contactPointIdx];
         contactPointHelper.computeContactForceCoefficientMatrix(solutionVector, startIdx);
         CommonOps_DDRM.addEquals(contactWrenchCoefficientMatrix, contactPointHelper.getContactForceCoefficientMatrix());
         startIdx += contactPointHelper.getCoefficientsSize();
      }
   }

   /**
    * Returns the collapsed wrench function. This is the sum of all the generalized contact forces in Euclidean space. This function should be a 3x4 matrix,
    * where the rows match the corresponding Euclidean coordinate axis.
    *
    * @return contact wrench matrix
    */
   public DMatrixRMaj getContactWrenchCoefficientMatrix()
   {
      return contactWrenchCoefficientMatrix;
   }

   public void computeContactForce(double omega, double time)
   {
      contactAcceleration.setToZero();
      contactJerk.setToZero();
      for (int i = 0; i < numberOfContactPoints; i++)
      {
         contactPoints[i].computeContactForce(omega, time);
         contactAcceleration.add(contactPoints[i].getContactAcceleration());
         contactJerk.add(contactPoints[i].getContactJerk());
      }
   }

   public FrameVector3DReadOnly getContactAcceleration()
   {
      return contactAcceleration;
   }

   public FrameVector3DReadOnly getContactJerk()
   {
      return contactJerk;
   }

   public void clearViz()
   {
      for (MPCContactPoint contactPoint : contactPoints)
      {
         contactPoint.clearViz();
      }
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MPCContactPlane)
      {
         MPCContactPlane other = (MPCContactPlane) object;
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
