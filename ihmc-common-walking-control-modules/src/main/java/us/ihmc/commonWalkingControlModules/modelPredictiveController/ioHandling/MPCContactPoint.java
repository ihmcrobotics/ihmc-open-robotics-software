package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPointForceViewer;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLongTime;

public class MPCContactPoint
{
   private final int numberOfBasisVectorsPerContactPoint;
   private final int coefficientsSize;
   private final FixedFrameVector3DBasics[] basisVectorsInWorld;
   private final FixedFrameVector3DBasics[] basisVectorsInPlaneFrame;
   private final FramePoint3D basisVectorOrigin = new FramePoint3D();
   private final double basisVectorAngleIncrement;
   private final PoseReferenceFrame planeFrame;

   private double rhoNormalZ;

   private final RotationMatrix normalContactVectorRotationMatrix = new RotationMatrix();

   private final DMatrixRMaj jerkIntegrationHessian;

   private final DMatrixRMaj contactWrenchCoefficientMatrix;

   private final FrameVector3D contactAcceleration = new FrameVector3D();
   private final FrameVector3D contactJerk = new FrameVector3D();
   private final FrameVector3D[] basisMagnitudes;
   private final FrameVector3D[] basisRates;
   private final DMatrixRMaj[] basisCoefficients;

   private ContactPointForceViewer viewer;

   public MPCContactPoint(int numberOfBasisVectorsPerContactPoint)
   {
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      coefficientsSize = LinearMPCIndexHandler.coefficientsPerRho * numberOfBasisVectorsPerContactPoint;

      basisVectorAngleIncrement = 2.0 * Math.PI / numberOfBasisVectorsPerContactPoint;

      basisVectorsInWorld = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisVectorsInPlaneFrame = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisMagnitudes = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisRates = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisCoefficients = new DMatrixRMaj[this.numberOfBasisVectorsPerContactPoint];
      planeFrame = new PoseReferenceFrame("ContactFrame", ReferenceFrame.getWorldFrame());

      jerkIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);

      contactWrenchCoefficientMatrix = new DMatrixRMaj(3, 4);

      for (int i = 0; i < this.numberOfBasisVectorsPerContactPoint; i++)
      {
         basisVectorsInWorld[i] = new FrameVector3D(ReferenceFrame.getWorldFrame());
         basisVectorsInPlaneFrame[i] = new FrameVector3D(planeFrame);
         basisMagnitudes[i] = new FrameVector3D(ReferenceFrame.getWorldFrame());
         basisRates[i] = new FrameVector3D(ReferenceFrame.getWorldFrame());
         basisCoefficients[i] = new DMatrixRMaj(1, 4);
      }

      clear();
   }

   public void setContactPointForceViewer(ContactPointForceViewer viewer)
   {
      this.viewer = viewer;
   }

   /**
    * Gets the total number of generalized contact value vectors for this contact point
    * @return total number of vectors
    */
   public int getRhoSize()
   {
      return numberOfBasisVectorsPerContactPoint;
   }

   /**
    * Gets the total number of coefficients used to express the force at this contact point
    * @return total number of coefficients
    */
   public int getCoefficientsSize()
   {
      return coefficientsSize;
   }

   /**
    * Gets the basis vector along which the generalized contact value acts.
    * @param index index of the basis vector to query
    * @return basis vector direction
    */
   public FrameVector3DReadOnly getBasisVector(int index)
   {
      return basisVectorsInWorld[index];
   }

   public FrameVector3DReadOnly getBasisVectorInPlaneFrame(int index)
   {
      return basisVectorsInPlaneFrame[index];
   }

   /**
    * Gets the origin of the basis vectors. Should correspond to the contact point location.
    * @return origin of the basis vectors.
    */
   public FramePoint3DReadOnly getBasisVectorOrigin()
   {
      return basisVectorOrigin;
   }

   /**
    * Computes the basis vectors for the contact point along which the generalized contact values act.
    *
    * @param contactPointInPlaneFrame origin of the basis vectors.
    * @param framePose pose of the plane, which gives the contact normal
    * @param mu coefficient of friction
    */
   public void computeBasisVectors(Point2DReadOnly contactPointInPlaneFrame, FramePose3DReadOnly framePose, double rotationOffset, double mu)
   {
      planeFrame.setPoseAndUpdate(framePose);

      // Compute the orientation of the normal contact vector and the corresponding transformation matrix
      computeNormalContactVectorRotation(normalContactVectorRotationMatrix);

      int rhoIndex = 0;

      basisVectorOrigin.setIncludingFrame(planeFrame, contactPointInPlaneFrame, 0.0);
      basisVectorOrigin.changeFrame(ReferenceFrame.getWorldFrame());

      // rotate each friction cone approximation to point one vector towards the center of the foot
      for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
      {
         FixedFrameVector3DBasics basisVector = basisVectorsInPlaneFrame[rhoIndex];

         computeBasisVector(basisVectorIndex, rotationOffset, normalContactVectorRotationMatrix, basisVector, mu);

         basisVectorsInWorld[rhoIndex].setMatchingFrame(basisVector);

         rhoIndex++;
      }

      rhoNormalZ = basisVectorsInWorld[0].getZ();
   }



   /**
    * Computes the equivalent quadratic cost function components that minimize the integral of the jerk over the duration
    * @param duration duration for the integration
    * @param omega time constant for the motion function
    */
   public void computeJerkIntegrationMatrix(double duration, double omega)
   {
      duration = Math.min(duration, sufficientlyLongTime);

      double positiveExponential = Math.min(Math.exp(omega * duration), sufficientlyLargeValue);
      double positiveExponential2 = Math.min(positiveExponential * positiveExponential, sufficientlyLargeValue);
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
      double c12 = 6.0 * omega2 * (negativeExponential - 1.0);
      double c22 = 36.0 * duration;

      // FIXME I don't think this includes the cross-terms
      for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectorsPerContactPoint; basisVectorIndexI++)
      {
         int startIdxI = basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;

         FrameVector3DReadOnly basisVectorI = basisVectorsInWorld[basisVectorIndexI];

         jerkIntegrationHessian.unsafe_set(startIdxI, startIdxI, c00);
         jerkIntegrationHessian.unsafe_set(startIdxI, startIdxI + 1, c01);
         jerkIntegrationHessian.unsafe_set(startIdxI, startIdxI + 2, c02);
         jerkIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI, c01);
         jerkIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI + 1, c11);
         jerkIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI + 2, c12);
         jerkIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI, c02);
         jerkIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI + 1, c12);
         jerkIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI + 2, c22);

         for (int basisVectorIndexJ = basisVectorIndexI + 1; basisVectorIndexJ < numberOfBasisVectorsPerContactPoint; basisVectorIndexJ++)
         {
            FrameVector3DReadOnly basisVectorJ = basisVectorsInWorld[basisVectorIndexJ];

            double basisDot = basisVectorI.dot(basisVectorJ);

            int startIdxJ = basisVectorIndexJ * LinearMPCIndexHandler.coefficientsPerRho;

            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI, startIdxJ, basisDot * c00);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI, startIdxJ + 1, basisDot * c01);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI, startIdxJ + 2, basisDot * c02);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI + 1, startIdxJ, basisDot * c01);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI + 1, startIdxJ + 1, basisDot * c11);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI + 1, startIdxJ + 2, basisDot * c12);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI + 2, startIdxJ, basisDot * c02);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI + 2, startIdxJ + 1, basisDot * c12);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxI + 2, startIdxJ + 2, basisDot * c22);

            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ, startIdxI, basisDot * c00);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ, startIdxI + 1, basisDot * c01);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ, startIdxI + 2, basisDot * c02);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ + 1, startIdxI, basisDot * c01);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ + 1, startIdxI + 1, basisDot * c11);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ + 1, startIdxI + 2, basisDot * c12);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ + 2, startIdxI, basisDot * c02);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ + 2, startIdxI + 1, basisDot * c12);
            MatrixMissingTools.unsafe_add(jerkIntegrationHessian, startIdxJ + 2, startIdxI + 2, basisDot * c22);
         }
      }
   }

   private final FrameVector3D contactNormalVector = new FrameVector3D();

   private void computeNormalContactVectorRotation(RotationMatrix normalContactVectorRotationMatrixToPack)
   {
      contactNormalVector.setIncludingFrame(planeFrame, 0.0, 0.0, 1.0);
      EuclidGeometryTools.orientation3DFromZUpToVector3D(contactNormalVector, normalContactVectorRotationMatrixToPack);
   }

   public void clear()
   {
      basisVectorOrigin.setToZero(ReferenceFrame.getWorldFrame());
      for (int rhoIndex = 0; rhoIndex < basisVectorsInWorld.length; rhoIndex++)
      {
         basisVectorsInWorld[rhoIndex].setToZero();
         basisVectorsInPlaneFrame[rhoIndex].setToZero();
      }

      if (viewer != null)
         viewer.reset();
   }

   private void computeBasisVector(int basisVectorIndex,
                                   double rotationOffset,
                                   RotationMatrix normalContactVectorRotationMatrix,
                                   FixedFrameVector3DBasics basisVectorToPack,
                                   double mu)
   {
      double angle = rotationOffset + basisVectorIndex * basisVectorAngleIncrement;

      // Compute the linear part considering a normal contact vector pointing z-up
      basisVectorToPack.set(Math.cos(angle) * mu, Math.sin(angle) * mu, 1.0);

      // Transforming the result to consider the actual normal contact vector
      normalContactVectorRotationMatrix.transform(basisVectorToPack);
      basisVectorToPack.normalize();
   }

   public double getRhoNormalZ()
   {
      return rhoNormalZ;
   }

   public DMatrixRMaj getJerkIntegrationHessian()
   {
      return jerkIntegrationHessian;
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
      contactWrenchCoefficientMatrix.zero();

      int startIdx = solutionStartIdx;
      for (int rhoIndex = 0; rhoIndex < numberOfBasisVectorsPerContactPoint; rhoIndex++)
      {
         FrameVector3DReadOnly basisVector = basisVectorsInWorld[rhoIndex];
         for (int coeffIdx = 0; coeffIdx < LinearMPCIndexHandler.coefficientsPerRho; coeffIdx++)
         {
            double rhoCoeff = solutionVector.get(startIdx, 0);
            contactWrenchCoefficientMatrix.add(0, coeffIdx, basisVector.getX() * rhoCoeff);
            contactWrenchCoefficientMatrix.add(1, coeffIdx, basisVector.getY() * rhoCoeff);
            contactWrenchCoefficientMatrix.add(2, coeffIdx, basisVector.getZ() * rhoCoeff);

            basisCoefficients[rhoIndex].set(0, coeffIdx, rhoCoeff);

            startIdx++;
         }
      }
   }

   /**
    * Returns the collapsed wrench function. This is the sum of all the generalized contact forces in Euclidean space. This function should be a 3x4 matrix,
    * where the rows match the corresponding Euclidean coordinate axis.
    * @return contact wrench matrix
    */
   public DMatrixRMaj getContactForceCoefficientMatrix()
   {
      return contactWrenchCoefficientMatrix;
   }

   public DMatrixRMaj getBasisCoefficients(int rhoIdx)
   {
      return basisCoefficients[rhoIdx];
   }

   public void computeContactForce(double omega, double time)
   {
      double omega2 = omega * omega;
      double omega3 = omega * omega2;
      double exponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double negativeExponential = 1.0 / exponential;
      double a0 = omega2 * exponential;
      double a1 = omega2 * negativeExponential;
      double a2 = 6.0 * time;
      double a3 = 2.0;

      double aDot0 = omega3 * exponential;
      double aDot1 = -omega3 * negativeExponential;
      double aDot2 = 6.0;

      contactAcceleration.setToZero();
      contactJerk.setToZero();

      for (int rhoIdx = 0; rhoIdx < numberOfBasisVectorsPerContactPoint; rhoIdx++)
      {
         double rhoValue = a0 * basisCoefficients[rhoIdx].get(0, 0);
         rhoValue += a1 * basisCoefficients[rhoIdx].get(0, 1);
         rhoValue += a2 * basisCoefficients[rhoIdx].get(0, 2);
         rhoValue += a3 * basisCoefficients[rhoIdx].get(0, 3);

         double rhoRate = aDot0 * basisCoefficients[rhoIdx].get(0, 0);
         rhoRate += aDot1 * basisCoefficients[rhoIdx].get(0, 1);
         rhoRate += aDot2 * basisCoefficients[rhoIdx].get(0, 2);

         basisMagnitudes[rhoIdx].setAndScale(rhoValue, basisVectorsInWorld[rhoIdx]);
         basisRates[rhoIdx].setAndScale(rhoRate, basisVectorsInWorld[rhoIdx]);

         contactAcceleration.add(basisMagnitudes[rhoIdx]);
         contactJerk.add(basisRates[rhoIdx]);
      }

      if (viewer != null)
         viewer.update(basisVectorOrigin, contactAcceleration, basisMagnitudes);
   }

   public FrameVector3DReadOnly getBasisMagnitude(int rhoIdx)
   {
      return basisMagnitudes[rhoIdx];
   }

   public void clearViz()
   {
      if (viewer != null)
         viewer.reset();
   }

   public FrameVector3DReadOnly getContactAcceleration()
   {
      return contactAcceleration;
   }

   public FrameVector3DReadOnly getContactJerk()
   {
      return contactJerk;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MPCContactPoint)
      {
         MPCContactPoint other = (MPCContactPoint) object;
         if (numberOfBasisVectorsPerContactPoint != other.numberOfBasisVectorsPerContactPoint)
            return false;
         if (coefficientsSize != other.coefficientsSize)
            return false;
         if (!planeFrame.equals(other.planeFrame))
            return false;
         if (!basisVectorOrigin.equals(other.basisVectorOrigin))
            return false;
         if (basisVectorsInWorld.length != other.basisVectorsInWorld.length)
            return false;
         if (rhoNormalZ != other.rhoNormalZ)
            return false;
         if (basisCoefficients.length != other.basisCoefficients.length)
            return false;
         for (int i = 0; i < basisCoefficients.length; i++)
         {
            if (!basisCoefficients[i].equals(other.basisCoefficients[i]))
               return false;
         }
         for (int i = 0; i < basisVectorsInWorld.length; i++)
         {
            if (!basisVectorsInWorld[i].equals(other.basisVectorsInWorld[i]))
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
