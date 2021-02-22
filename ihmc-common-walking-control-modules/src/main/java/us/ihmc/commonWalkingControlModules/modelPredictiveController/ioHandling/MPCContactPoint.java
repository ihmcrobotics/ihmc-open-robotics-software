package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.ContactPointForceViewer;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLongTime;

public class MPCContactPoint
{
   private final int numberOfBasisVectorsPerContactPoint;
   private final int coefficientsSize;
   private final FrameVector3D[] basisVectors;
   private final FramePoint3D basisVectorOrigin = new FramePoint3D();
   private final double basisVectorAngleIncrement;
   private final PoseReferenceFrame planeFrame;

   private final DMatrixRMaj rhoMaxMatrix;

   private double maxContactForce;
   private double timeOfContact = Double.NaN;

   private final RotationMatrix normalContactVectorRotationMatrix = new RotationMatrix();

   private final DMatrixRMaj accelerationIntegrationHessian;
   private final DMatrixRMaj accelerationIntegrationGradient;
   private final DMatrixRMaj jerkIntegrationHessian;

   private final DMatrixRMaj contactWrenchCoefficientMatrix;

   private final FrameVector3D contactAcceleration = new FrameVector3D();
   private final FrameVector3D[] basisMagnitudes;
   private final DMatrixRMaj[] basisCoefficients;

   private ContactPointForceViewer viewer;

   public MPCContactPoint(int numberOfBasisVectorsPerContactPoint)
   {
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      coefficientsSize = LinearMPCIndexHandler.coefficientsPerRho * numberOfBasisVectorsPerContactPoint;

      basisVectorAngleIncrement = 2.0 * Math.PI / numberOfBasisVectorsPerContactPoint;

      rhoMaxMatrix = new DMatrixRMaj(this.numberOfBasisVectorsPerContactPoint, 1);

      maxContactForce = Double.POSITIVE_INFINITY;

      basisVectors = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisMagnitudes = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisCoefficients = new DMatrixRMaj[this.numberOfBasisVectorsPerContactPoint];
      planeFrame = new PoseReferenceFrame("ContactFrame", ReferenceFrame.getWorldFrame());

      accelerationIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);
      accelerationIntegrationGradient = new DMatrixRMaj(coefficientsSize, 1);
      jerkIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);

      contactWrenchCoefficientMatrix = new DMatrixRMaj(3, 4);

      for (int i = 0; i < this.numberOfBasisVectorsPerContactPoint; i++)
      {
         basisVectors[i] = new FrameVector3D(ReferenceFrame.getWorldFrame());
         basisMagnitudes[i] = new FrameVector3D(ReferenceFrame.getWorldFrame());
         basisCoefficients[i] = new DMatrixRMaj(1, 4);
      }

      clear();
   }

   public void setContactPointForceViewer(ContactPointForceViewer viewer)
   {
      this.viewer = viewer;
   }

   /**
    * Sets the maximum net force allowed in the normal force for this contact point.
    * @param maxNormalForce maximum normal force
    */
   public void setMaxNormalForce(double maxNormalForce)
   {
      this.maxContactForce = maxNormalForce;
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
      return basisVectors[index];
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
      timeOfContact = Double.NaN;
      planeFrame.setPoseAndUpdate(framePose);

      // Compute the orientation of the normal contact vector and the corresponding transformation matrix
      computeNormalContactVectorRotation(normalContactVectorRotationMatrix);

      rhoMaxMatrix.reshape(numberOfBasisVectorsPerContactPoint, 1);
      rhoMaxMatrix.zero();

      int rhoIndex = 0;

      double maxRhoZ = maxContactForce / numberOfBasisVectorsPerContactPoint;
      basisVectorOrigin.setIncludingFrame(planeFrame, contactPointInPlaneFrame, 0.0);
      basisVectorOrigin.changeFrame(ReferenceFrame.getWorldFrame());

      // rotate each friction cone approximation to point one vector towards the center of the foot
      for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
      {
         FrameVector3D basisVector = basisVectors[rhoIndex];

         computeBasisVector(basisVectorIndex, rotationOffset, normalContactVectorRotationMatrix, basisVector, mu);

         rhoMaxMatrix.set(rhoIndex, 0, maxRhoZ / basisVector.getZ());

         rhoIndex++;
      }
   }


   /**
    * Computes the equivalent quadratic cost function components that minimize the difference from the acceleration and some net goal value for the point over some time
    * @param duration duration for the integration
    * @param omega time constant for the motion function
    * @param goalValueForPoint nominal value for the acceleration to track.
    */
   public void computeAccelerationIntegrationMatrix(double duration, double omega, double goalValueForPoint)
   {
      duration = Math.min(duration, sufficientlyLongTime);
      double goalValueForBasis = goalValueForPoint / numberOfBasisVectorsPerContactPoint;

      double positiveExponential = Math.min(Math.exp(omega * duration), sufficientlyLargeValue);
      double positiveExponential2 = Math.min(positiveExponential * positiveExponential, sufficientlyLargeValue);
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

      double g0 = omega * (positiveExponential - 1.0) * goalValueForBasis;
      double g1 = -omega * (negativeExponential - 1.0) * goalValueForBasis;
      double g2 = 3.0 * duration2 * goalValueForBasis;
      double g3 = 2.0 * duration * goalValueForBasis;

      for (int basisVectorIndexI = 0; basisVectorIndexI < numberOfBasisVectorsPerContactPoint; basisVectorIndexI++)
      {
         int startIdxI = basisVectorIndexI * LinearMPCIndexHandler.coefficientsPerRho;

         FrameVector3DReadOnly basisVectorI = basisVectors[basisVectorIndexI];

         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI, c00);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 1, c01);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 2, c02);
         accelerationIntegrationHessian.unsafe_set(startIdxI, startIdxI + 2, c03);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI, c01);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI + 1, c11);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI + 2, c12);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 1, startIdxI + 3, c13);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI, c02);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI + 1, c12);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI + 2, c22);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 2, startIdxI + 3, c23);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI, c03);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI + 1, c13);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI + 2, c23);
         accelerationIntegrationHessian.unsafe_set(startIdxI + 3, startIdxI + 3, c33);

         accelerationIntegrationGradient.unsafe_set(startIdxI, 0, g0);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 1, 0, g1);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 2, 0, g2);
         accelerationIntegrationGradient.unsafe_set(startIdxI + 3, 0, g3);

         for (int basisVectorIndexJ = basisVectorIndexI + 1; basisVectorIndexJ < numberOfBasisVectorsPerContactPoint; basisVectorIndexJ++)
         {
            FrameVector3DReadOnly basisVectorJ = basisVectors[basisVectorIndexJ];

            double basisDot = basisVectorI.dot(basisVectorJ);

            int startIdxJ = basisVectorIndexJ * LinearMPCIndexHandler.coefficientsPerRho;

            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI, startIdxJ, basisDot * c00);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI, startIdxJ + 1, basisDot * c01);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI, startIdxJ + 2, basisDot * c02);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI, startIdxJ + 3, basisDot * c03);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 1, startIdxJ, basisDot * c01);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 1, startIdxJ + 1, basisDot * c11);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 1, startIdxJ + 2, basisDot * c12);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 1, startIdxJ + 3, basisDot * c13);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 2, startIdxJ, basisDot * c02);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 2, startIdxJ + 1, basisDot * c12);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 2, startIdxJ + 2, basisDot * c22);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 2, startIdxJ + 3, basisDot * c23);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 3, startIdxJ, basisDot * c03);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 3, startIdxJ + 1, basisDot * c13);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 3, startIdxJ + 2, basisDot * c23);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxI + 3, startIdxJ + 3, basisDot * c33);

            // we know it's symmetric, and this way we can avoid iterating as much
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ, startIdxI, basisDot * c00);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ, startIdxI + 1, basisDot * c01);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ, startIdxI + 2, basisDot * c02);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ, startIdxI + 3, basisDot * c03);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 1, startIdxI, basisDot * c01);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 1, startIdxI + 1, basisDot * c11);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 1, startIdxI + 2, basisDot * c12);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 1, startIdxI + 3, basisDot * c13);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 2, startIdxI, basisDot * c02);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 2, startIdxI + 1, basisDot * c12);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 2, startIdxI + 2, basisDot * c22);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 2, startIdxI + 3, basisDot * c23);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 3, startIdxI, basisDot * c03);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 3, startIdxI + 1, basisDot * c13);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 3, startIdxI + 2, basisDot * c23);
            MatrixMissingTools.unsafe_add(accelerationIntegrationHessian, startIdxJ + 3, startIdxI + 3, basisDot * c33);
         }
      }
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

         FrameVector3DReadOnly basisVectorI = basisVectors[basisVectorIndexI];

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
            FrameVector3DReadOnly basisVectorJ = basisVectors[basisVectorIndexJ];

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
      for (int rhoIndex = 0; rhoIndex < basisVectors.length; rhoIndex++)
      {
         FrameVector3D basisVector = basisVectors[rhoIndex];
         basisVector.setToZero(ReferenceFrame.getWorldFrame());

         rhoMaxMatrix.set(rhoIndex, 0, Double.POSITIVE_INFINITY);
      }

      if (viewer != null)
         viewer.reset();
   }

   private void computeBasisVector(int basisVectorIndex,
                                   double rotationOffset,
                                   RotationMatrix normalContactVectorRotationMatrix,
                                   FrameVector3D basisVectorToPack,
                                   double mu)
   {
      double angle = rotationOffset + basisVectorIndex * basisVectorAngleIncrement;

      // Compute the linear part considering a normal contact vector pointing z-up
      basisVectorToPack.setIncludingFrame(planeFrame, Math.cos(angle) * mu, Math.sin(angle) * mu, 1.0);

      // Transforming the result to consider the actual normal contact vector
      normalContactVectorRotationMatrix.transform(basisVectorToPack);
      basisVectorToPack.normalize();
      basisVectorToPack.changeFrame(ReferenceFrame.getWorldFrame());
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
         FrameVector3DReadOnly basisVector = basisVectors[rhoIndex];
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
   public DMatrixRMaj getContactWrenchCoefficientMatrix()
   {
      return contactWrenchCoefficientMatrix;
   }


   public void computeContactForce(double omega, double time)
   {
      double omega2 = omega * omega;
      double exponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
      double negativeExponential = 1.0 / exponential;
      double a0 = omega2 * exponential;
      double a1 = omega2 * negativeExponential;
      double a2 = 6.0 * time;
      double a3 = 2.0;

      contactAcceleration.setToZero();

      for (int rhoIdx = 0; rhoIdx < numberOfBasisVectorsPerContactPoint; rhoIdx++)
      {
         double rhoValue = a0 * basisCoefficients[rhoIdx].get(0, 0);
         rhoValue += a1 * basisCoefficients[rhoIdx].get(0, 1);
         rhoValue += a2 * basisCoefficients[rhoIdx].get(0, 2);
         rhoValue += a3 * basisCoefficients[rhoIdx].get(0, 3);

         basisMagnitudes[rhoIdx].setAndScale(rhoValue, basisVectors[rhoIdx]);
         contactAcceleration.add(basisMagnitudes[rhoIdx]);
      }

      if (viewer != null)
         viewer.update(basisVectorOrigin, contactAcceleration, basisMagnitudes);
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
         if (basisVectors.length != other.basisVectors.length)
            return false;
         if (timeOfContact != other.timeOfContact)
            return false;
         if (maxContactForce != other.maxContactForce)
            return false;
         if (basisCoefficients.length != other.basisCoefficients.length)
            return false;
         for (int i = 0; i < basisCoefficients.length; i++)
         {
            if (!basisCoefficients[i].equals(other.basisCoefficients[i]))
               return false;
         }
         for (int i = 0; i < basisVectors.length; i++)
         {
            if (!basisVectors[i].equals(other.basisVectors[i]))
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
