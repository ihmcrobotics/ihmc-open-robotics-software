package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLongTime;

public class ContactPointHelper
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

//   private final DMatrixRMaj totalJacobianInWorldFrame;
//   private final DMatrixRMaj linearJacobianInWorldFrame;

   private final RotationMatrix normalContactVectorRotationMatrix = new RotationMatrix();

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

   private final DMatrixRMaj contactWrenchCoefficientMatrix;


   private final FrameVector3D contactAcceleration = new FrameVector3D();
   private final FrameVector3D[] basisMagnitudes;
   private final DMatrixRMaj[] basisCoefficients;

   private ContactPointForceViewer viewer;

   public ContactPointHelper(int numberOfBasisVectorsPerContactPoint)
   {
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      coefficientsSize = MPCIndexHandler.coefficientsPerRho * numberOfBasisVectorsPerContactPoint;

      basisVectorAngleIncrement = 2.0 * Math.PI / numberOfBasisVectorsPerContactPoint;

      rhoMaxMatrix = new DMatrixRMaj(this.numberOfBasisVectorsPerContactPoint, 1);

      maxContactForce = Double.POSITIVE_INFINITY;

      basisVectors = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisMagnitudes = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisCoefficients = new DMatrixRMaj[this.numberOfBasisVectorsPerContactPoint];
      planeFrame = new PoseReferenceFrame("ContactFrame", ReferenceFrame.getWorldFrame());

//      totalJacobianInWorldFrame = new DMatrixRMaj(Wrench.SIZE, this.numberOfBasisVectorsPerContactPoint);
//      linearJacobianInWorldFrame = new DMatrixRMaj(3, this.numberOfBasisVectorsPerContactPoint);

      linearPositionJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearVelocityJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearAccelerationJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      linearJerkJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);

      rhoMagnitudeJacobianMatrix = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint, coefficientsSize);
      rhoRateJacobianMatrix = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint, coefficientsSize);
      rhoAccelerationJacobianMatrix = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint, coefficientsSize);
      rhoJerkJacobianMatrix = new DMatrixRMaj(numberOfBasisVectorsPerContactPoint, coefficientsSize);

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

   public void setMaxNormalForce(double maxNormalForce)
   {
      this.maxContactForce = maxNormalForce;
   }

   public int getRhoSize()
   {
      return numberOfBasisVectorsPerContactPoint;
   }

   public int getCoefficientsSize()
   {
      return coefficientsSize;
   }

   public FrameVector3DReadOnly getBasisVector(int index)
   {
      return basisVectors[index];
   }

   public FramePoint3DReadOnly getBasisVectorOrigin()
   {
      return basisVectorOrigin;
   }

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

//      computeWrenchJacobianInFrame(ReferenceFrame.getWorldFrame(), totalJacobianInWorldFrame, linearJacobianInWorldFrame);
   }

   public void computeJacobians(double time, double omega)
   {
      if (MathTools.epsilonEquals(time, timeOfContact, 1e-5))
         return;

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
         int startColumn = basisVectorIndex * MPCIndexHandler.coefficientsPerRho;
         FrameVector3DReadOnly basisVector = basisVectors[basisVectorIndex];

         rhoMagnitudeJacobianMatrix.set(basisVectorIndex, startColumn, positiveExponential);
         rhoMagnitudeJacobianMatrix.set(basisVectorIndex, startColumn + 1, negativeExponential);

         rhoRateJacobianMatrix.set(basisVectorIndex, startColumn, firstVelocityCoefficient);
         rhoRateJacobianMatrix.set(basisVectorIndex, startColumn + 1, secondVelocityCoefficient);

         rhoAccelerationJacobianMatrix.set(basisVectorIndex, startColumn, firstAccelerationCoefficient);
         rhoAccelerationJacobianMatrix.set(basisVectorIndex, startColumn + 1, secondAccelerationCoefficient);
         rhoAccelerationJacobianMatrix.set(basisVectorIndex, startColumn + 3, 2.0);

         rhoJerkJacobianMatrix.set(basisVectorIndex, startColumn, firstJerkCoefficient);
         rhoJerkJacobianMatrix.set(basisVectorIndex, startColumn + 1, secondJerkCoefficient);
         rhoJerkJacobianMatrix.set(basisVectorIndex, startColumn + 2, 6.0);

         if (setTimeCoefficients)
         {
            rhoMagnitudeJacobianMatrix.set(basisVectorIndex, startColumn + 2, t3);
            rhoMagnitudeJacobianMatrix.set(basisVectorIndex, startColumn + 3, t2);

            rhoRateJacobianMatrix.set(basisVectorIndex, startColumn + 2, thirdVelocityCoefficient);
            rhoRateJacobianMatrix.set(basisVectorIndex, startColumn + 3, fourthVelocityCoefficient);

            rhoAccelerationJacobianMatrix.set(basisVectorIndex, startColumn + 2, thirdAccelerationCoefficient);
         }

         for (int ordinal = 0; ordinal < 3; ordinal++)
         {
            linearPositionJacobianMatrix.set(ordinal, startColumn, basisVector.getElement(ordinal) * positiveExponential);
            linearPositionJacobianMatrix.set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * negativeExponential);

            linearVelocityJacobianMatrix.set(ordinal, startColumn, basisVector.getElement(ordinal) * firstVelocityCoefficient);
            linearVelocityJacobianMatrix.set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondVelocityCoefficient);

            linearAccelerationJacobianMatrix.set(ordinal, startColumn, basisVector.getElement(ordinal) * firstAccelerationCoefficient);
            linearAccelerationJacobianMatrix.set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondAccelerationCoefficient);
            linearAccelerationJacobianMatrix.set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * 2.0);

            linearJerkJacobianMatrix.set(ordinal, startColumn, basisVector.getElement(ordinal) * firstJerkCoefficient);
            linearJerkJacobianMatrix.set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondJerkCoefficient);
            linearJerkJacobianMatrix.set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * 6.0);

            if (setTimeCoefficients)
            {
               linearPositionJacobianMatrix.set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * t3);
               linearPositionJacobianMatrix.set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * t2);

               linearVelocityJacobianMatrix.set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * thirdVelocityCoefficient);
               linearVelocityJacobianMatrix.set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * fourthVelocityCoefficient);

               linearAccelerationJacobianMatrix.set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * thirdAccelerationCoefficient);
            }
         }
      }

      timeOfContact = time;
   }

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
         int startIdxI = basisVectorIndexI * MPCIndexHandler.coefficientsPerRho;

         FrameVector3DReadOnly basisVectorI = basisVectors[basisVectorIndexI];

         accelerationIntegrationHessian.set(startIdxI, startIdxI, c00);
         accelerationIntegrationHessian.set(startIdxI, startIdxI + 1, c01);
         accelerationIntegrationHessian.set(startIdxI, startIdxI + 2, c02);
         accelerationIntegrationHessian.set(startIdxI, startIdxI + 2, c03);
         accelerationIntegrationHessian.set(startIdxI + 1, startIdxI, c01);
         accelerationIntegrationHessian.set(startIdxI + 1, startIdxI + 1, c11);
         accelerationIntegrationHessian.set(startIdxI + 1, startIdxI + 2, c12);
         accelerationIntegrationHessian.set(startIdxI + 1, startIdxI + 3, c13);
         accelerationIntegrationHessian.set(startIdxI + 2, startIdxI, c02);
         accelerationIntegrationHessian.set(startIdxI + 2, startIdxI + 1, c12);
         accelerationIntegrationHessian.set(startIdxI + 2, startIdxI + 2, c22);
         accelerationIntegrationHessian.set(startIdxI + 2, startIdxI + 3, c23);
         accelerationIntegrationHessian.set(startIdxI + 3, startIdxI, c03);
         accelerationIntegrationHessian.set(startIdxI + 3, startIdxI + 1, c13);
         accelerationIntegrationHessian.set(startIdxI + 3, startIdxI + 2, c23);
         accelerationIntegrationHessian.set(startIdxI + 3, startIdxI + 3, c33);

         accelerationIntegrationGradient.set(startIdxI, 0, g0);
         accelerationIntegrationGradient.set(startIdxI + 1, 0, g1);
         accelerationIntegrationGradient.set(startIdxI + 2, 0, g2);
         accelerationIntegrationGradient.set(startIdxI + 3, 0, g3);

         for (int basisVectorIndexJ = basisVectorIndexI + 1; basisVectorIndexJ < numberOfBasisVectorsPerContactPoint; basisVectorIndexJ++)
         {
            FrameVector3DReadOnly basisVectorJ = basisVectors[basisVectorIndexJ];

            double basisDot = basisVectorI.dot(basisVectorJ);

            int startIdxJ = basisVectorIndexJ * MPCIndexHandler.coefficientsPerRho;

            accelerationIntegrationHessian.add(startIdxI, startIdxJ, basisDot * c00);
            accelerationIntegrationHessian.add(startIdxI, startIdxJ + 1, basisDot * c01);
            accelerationIntegrationHessian.add(startIdxI, startIdxJ + 2, basisDot * c02);
            accelerationIntegrationHessian.add(startIdxI, startIdxJ + 3, basisDot * c03);
            accelerationIntegrationHessian.add(startIdxI + 1, startIdxJ, basisDot * c01);
            accelerationIntegrationHessian.add(startIdxI + 1, startIdxJ + 1, basisDot * c11);
            accelerationIntegrationHessian.add(startIdxI + 1, startIdxJ + 2, basisDot * c12);
            accelerationIntegrationHessian.add(startIdxI + 1, startIdxJ + 3, basisDot * c13);
            accelerationIntegrationHessian.add(startIdxI + 2, startIdxJ, basisDot * c02);
            accelerationIntegrationHessian.add(startIdxI + 2, startIdxJ + 1, basisDot * c12);
            accelerationIntegrationHessian.add(startIdxI + 2, startIdxJ + 2, basisDot * c22);
            accelerationIntegrationHessian.add(startIdxI + 2, startIdxJ + 3, basisDot * c23);
            accelerationIntegrationHessian.add(startIdxI + 3, startIdxJ, basisDot * c03);
            accelerationIntegrationHessian.add(startIdxI + 3, startIdxJ + 1, basisDot * c13);
            accelerationIntegrationHessian.add(startIdxI + 3, startIdxJ + 2, basisDot * c23);
            accelerationIntegrationHessian.add(startIdxI + 3, startIdxJ + 3, basisDot * c33);

            // we know it's symmetric, and this way we can avoid iterating as much
            accelerationIntegrationHessian.add(startIdxJ, startIdxI, basisDot * c00);
            accelerationIntegrationHessian.add(startIdxJ, startIdxI + 1, basisDot * c01);
            accelerationIntegrationHessian.add(startIdxJ, startIdxI + 2, basisDot * c02);
            accelerationIntegrationHessian.add(startIdxJ, startIdxI + 3, basisDot * c03);
            accelerationIntegrationHessian.add(startIdxJ + 1, startIdxI, basisDot * c01);
            accelerationIntegrationHessian.add(startIdxJ + 1, startIdxI + 1, basisDot * c11);
            accelerationIntegrationHessian.add(startIdxJ + 1, startIdxI + 2, basisDot * c12);
            accelerationIntegrationHessian.add(startIdxJ + 1, startIdxI + 3, basisDot * c13);
            accelerationIntegrationHessian.add(startIdxJ + 2, startIdxI, basisDot * c02);
            accelerationIntegrationHessian.add(startIdxJ + 2, startIdxI + 1, basisDot * c12);
            accelerationIntegrationHessian.add(startIdxJ + 2, startIdxI + 2, basisDot * c22);
            accelerationIntegrationHessian.add(startIdxJ + 2, startIdxI + 3, basisDot * c23);
            accelerationIntegrationHessian.add(startIdxJ + 3, startIdxI, basisDot * c03);
            accelerationIntegrationHessian.add(startIdxJ + 3, startIdxI + 1, basisDot * c13);
            accelerationIntegrationHessian.add(startIdxJ + 3, startIdxI + 2, basisDot * c23);
            accelerationIntegrationHessian.add(startIdxJ + 3, startIdxI + 3, basisDot * c33);
         }
      }
   }

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
         int startIdxI = basisVectorIndexI * MPCIndexHandler.coefficientsPerRho;

         FrameVector3DReadOnly basisVectorI = basisVectors[basisVectorIndexI];

         jerkIntegrationHessian.set(startIdxI, startIdxI, c00);
         jerkIntegrationHessian.set(startIdxI, startIdxI + 1, c01);
         jerkIntegrationHessian.set(startIdxI, startIdxI + 2, c02);
         jerkIntegrationHessian.set(startIdxI + 1, startIdxI, c01);
         jerkIntegrationHessian.set(startIdxI + 1, startIdxI + 1, c11);
         jerkIntegrationHessian.set(startIdxI + 1, startIdxI + 2, c12);
         jerkIntegrationHessian.set(startIdxI + 2, startIdxI, c02);
         jerkIntegrationHessian.set(startIdxI + 2, startIdxI + 1, c12);
         jerkIntegrationHessian.set(startIdxI + 2, startIdxI + 2, c22);

         for (int basisVectorIndexJ = basisVectorIndexI + 1; basisVectorIndexJ < numberOfBasisVectorsPerContactPoint; basisVectorIndexJ++)
         {
            FrameVector3DReadOnly basisVectorJ = basisVectors[basisVectorIndexJ];

            double basisDot = basisVectorI.dot(basisVectorJ);

            int startIdxJ = basisVectorIndexJ * MPCIndexHandler.coefficientsPerRho;

            jerkIntegrationHessian.add(startIdxI, startIdxJ, basisDot * c00);
            jerkIntegrationHessian.add(startIdxI, startIdxJ + 1, basisDot * c01);
            jerkIntegrationHessian.add(startIdxI, startIdxJ + 2, basisDot * c02);
            jerkIntegrationHessian.add(startIdxI + 1, startIdxJ, basisDot * c01);
            jerkIntegrationHessian.add(startIdxI + 1, startIdxJ + 1, basisDot * c11);
            jerkIntegrationHessian.add(startIdxI + 1, startIdxJ + 2, basisDot * c12);
            jerkIntegrationHessian.add(startIdxI + 2, startIdxJ, basisDot * c02);
            jerkIntegrationHessian.add(startIdxI + 2, startIdxJ + 1, basisDot * c12);
            jerkIntegrationHessian.add(startIdxI + 2, startIdxJ + 2, basisDot * c22);

            jerkIntegrationHessian.add(startIdxJ, startIdxI, basisDot * c00);
            jerkIntegrationHessian.add(startIdxJ, startIdxI + 1, basisDot * c01);
            jerkIntegrationHessian.add(startIdxJ, startIdxI + 2, basisDot * c02);
            jerkIntegrationHessian.add(startIdxJ + 1, startIdxI, basisDot * c01);
            jerkIntegrationHessian.add(startIdxJ + 1, startIdxI + 1, basisDot * c11);
            jerkIntegrationHessian.add(startIdxJ + 1, startIdxI + 2, basisDot * c12);
            jerkIntegrationHessian.add(startIdxJ + 2, startIdxI, basisDot * c02);
            jerkIntegrationHessian.add(startIdxJ + 2, startIdxI + 1, basisDot * c12);
            jerkIntegrationHessian.add(startIdxJ + 2, startIdxI + 2, basisDot * c22);
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

//   private final SpatialForce unitSpatialForceVector = new SpatialForce();
//
//   public void computeWrenchJacobianInFrame(ReferenceFrame frame, DMatrixRMaj wrenchMatrixToPack, DMatrixRMaj forceMatrixToPack)
//   {
//      wrenchMatrixToPack.reshape(Wrench.SIZE, numberOfBasisVectorsPerContactPoint);
//      forceMatrixToPack.reshape(3, numberOfBasisVectorsPerContactPoint);
//      basisVectorOrigin.changeFrame(frame);
//
//      for (int rhoIndex = 0; rhoIndex < numberOfBasisVectorsPerContactPoint; rhoIndex++)
//      {
//         FrameVector3D basisVector = basisVectors[rhoIndex];
//         basisVector.changeFrame(frame);
//         unitSpatialForceVector.setIncludingFrame(null, basisVector, basisVectorOrigin);
//         unitSpatialForceVector.get(0, rhoIndex, wrenchMatrixToPack);
//         unitSpatialForceVector.getLinearPart().get(0, rhoIndex, forceMatrixToPack);
//      }
//   }

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

   public DMatrixRMaj getRhoMaxMatrix()
   {
      return rhoMaxMatrix;
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

   public void computeContactForceCoefficientMatrix(DMatrixRMaj solutionVector, int solutionStartIdx)
   {
      contactWrenchCoefficientMatrix.zero();

      int startIdx = solutionStartIdx;
      for (int rhoIndex = 0; rhoIndex < numberOfBasisVectorsPerContactPoint; rhoIndex++)
      {
         FrameVector3DReadOnly basisVector = basisVectors[rhoIndex];
         for (int coeffIdx = 0; coeffIdx < MPCIndexHandler.coefficientsPerRho; coeffIdx++)
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

   public FrameVector3DReadOnly getContactAcceleration()
   {
      return contactAcceleration;
   }
}
