package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class ContactPointHelper
{
   private final int numberOfBasisVectorsPerContactPoint;
   private final int coefficientsSize;
   private final FrameVector3D[] basisVectors;
   private final FramePoint3D[] basisVectorsOrigin;
   private final double basisVectorAngleIncrement;
   private final PoseReferenceFrame planeFrame;

   private final DMatrixRMaj rhoMaxMatrix;

   private double maxContactForce;
   private double timeOfContact = Double.NaN;

   private final DMatrixRMaj totalJacobianInWorldFrame;
   private final DMatrixRMaj linearJacobianInWorldFrame;

   private final RotationMatrix normalContactVectorRotationMatrix = new RotationMatrix();

   private final DMatrixRMaj positionJacobianMatrix;
   private final DMatrixRMaj velocityJacobianMatrix;
   private final DMatrixRMaj accelerationJacobianMatrix;
   private final DMatrixRMaj jerkJacobianMatrix;
   private final DMatrixRMaj accelerationIntegrationHessian;
   private final DMatrixRMaj accelerationIntegrationGradient;
   private final DMatrixRMaj jerkIntegrationHessian;

   public ContactPointHelper(int numberOfBasisVectorsPerContactPoint)
   {
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
      coefficientsSize = MPCIndexHandler.coefficientsPerRho * numberOfBasisVectorsPerContactPoint;

      basisVectorAngleIncrement = 2.0 * Math.PI / numberOfBasisVectorsPerContactPoint;

      rhoMaxMatrix = new DMatrixRMaj(this.numberOfBasisVectorsPerContactPoint, 1);

      maxContactForce = Double.POSITIVE_INFINITY;

      basisVectors = new FrameVector3D[this.numberOfBasisVectorsPerContactPoint];
      basisVectorsOrigin = new FramePoint3D[this.numberOfBasisVectorsPerContactPoint];
      planeFrame = new PoseReferenceFrame("ContactFrame", ReferenceFrame.getWorldFrame());

      totalJacobianInWorldFrame = new DMatrixRMaj(Wrench.SIZE, this.numberOfBasisVectorsPerContactPoint);
      linearJacobianInWorldFrame = new DMatrixRMaj(3, this.numberOfBasisVectorsPerContactPoint);

      positionJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      velocityJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      accelerationJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      jerkJacobianMatrix = new DMatrixRMaj(3, coefficientsSize);
      accelerationIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);
      accelerationIntegrationGradient = new DMatrixRMaj(coefficientsSize, 1);
      jerkIntegrationHessian = new DMatrixRMaj(coefficientsSize, coefficientsSize);

      for (int i = 0; i < this.numberOfBasisVectorsPerContactPoint; i++)
      {
         basisVectors[i] = new FrameVector3D(ReferenceFrame.getWorldFrame());
         basisVectorsOrigin[i] = new FramePoint3D(ReferenceFrame.getWorldFrame());
      }
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

   public FramePoint3DReadOnly getBasisVectorOrigin(int index)
   {
      return basisVectorsOrigin[index];
   }

   public void computeBasisVectors(Point2DReadOnly contactPointInPlaneFrame, FramePose3DReadOnly framePose, double rotationOffset, double mu)
   {
      planeFrame.setPoseAndUpdate(framePose);

      // Compute the orientation of the normal contact vector and the corresponding transformation matrix
      computeNormalContactVectorRotation(normalContactVectorRotationMatrix);

      rhoMaxMatrix.reshape(numberOfBasisVectorsPerContactPoint, 1);

      int rhoIndex = 0;

      // rotate each friction cone approximation to point one vector towards the center of the foot
      for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
      {
         FramePoint3D basisVectorOrigin = basisVectorsOrigin[rhoIndex];
         FrameVector3D basisVector = basisVectors[rhoIndex];

         basisVectorOrigin.setIncludingFrame(planeFrame, contactPointInPlaneFrame, 0.0);
         computeBasisVector(basisVectorIndex, rotationOffset, normalContactVectorRotationMatrix, basisVector, mu);

         rhoMaxMatrix.set(rhoIndex, 0, basisVector.getZ() * maxContactForce / numberOfBasisVectorsPerContactPoint);

         rhoIndex++;
      }

      computeWrenchJacobianInFrame(ReferenceFrame.getWorldFrame(), totalJacobianInWorldFrame, linearJacobianInWorldFrame);
   }

   public void computeJacobians(double time, double omega)
   {
      if (MathTools.epsilonEquals(time, timeOfContact, 1e-5))
         return;

      timeOfContact = time;

      positionJacobianMatrix.zero();
      velocityJacobianMatrix.zero();
      accelerationJacobianMatrix.zero();
      jerkJacobianMatrix.zero();

      double t2 = timeOfContact * timeOfContact;
      double t3 = timeOfContact * t2;
      double positiveExponential = Math.exp(omega * timeOfContact);
      double negativeExponential = 1.0 / positiveExponential;
      double firstVelocityCoefficient = omega * positiveExponential;
      double secondVelocityCoefficient = -omega * negativeExponential;
      double firstAccelerationCoefficient = omega * firstVelocityCoefficient;
      double secondAccelerationCoefficient = -omega * secondVelocityCoefficient;
      double firstJerkCoefficient = omega * firstAccelerationCoefficient;
      double secondJerkCoefficient = -omega * secondAccelerationCoefficient;
      boolean setTimeCoefficients = !MathTools.epsilonEquals(timeOfContact, 0.0, 1e-4);
      double thirdVelocityCoefficient = 3 * t2;
      double fourthVelocityCoefficient = 2 * timeOfContact;
      double thirdAccelerationCoefficient = 6 * timeOfContact;

      positionJacobianMatrix.zero();
      velocityJacobianMatrix.zero();
      accelerationJacobianMatrix.zero();
      jerkJacobianMatrix.zero();

      for (int basisVectorIndex = 0; basisVectorIndex < numberOfBasisVectorsPerContactPoint; basisVectorIndex++)
      {
         int startColumn = basisVectorIndex * MPCIndexHandler.coefficientsPerRho;
         FrameVector3DReadOnly basisVector = basisVectors[basisVectorIndex];

         for (int ordinal = 0; ordinal < 3; ordinal++)
         {
            positionJacobianMatrix.set(ordinal, startColumn, basisVector.getElement(ordinal) * positiveExponential);
            positionJacobianMatrix.set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * negativeExponential);

            velocityJacobianMatrix.set(ordinal, startColumn, basisVector.getElement(ordinal) * firstVelocityCoefficient);
            velocityJacobianMatrix.set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondVelocityCoefficient);

            accelerationJacobianMatrix.set(ordinal, startColumn, basisVector.getElement(ordinal) * firstAccelerationCoefficient);
            accelerationJacobianMatrix.set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondAccelerationCoefficient);
            accelerationJacobianMatrix.set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * 2.0);

            jerkJacobianMatrix.set(ordinal, startColumn, basisVector.getElement(ordinal) * firstJerkCoefficient);
            jerkJacobianMatrix.set(ordinal, startColumn + 1, basisVector.getElement(ordinal) * secondJerkCoefficient);
            jerkJacobianMatrix.set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * 6.0);

            if (setTimeCoefficients)
            {
               positionJacobianMatrix.set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * t3);
               positionJacobianMatrix.set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * t2);

               velocityJacobianMatrix.set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * thirdVelocityCoefficient);
               velocityJacobianMatrix.set(ordinal, startColumn + 3, basisVector.getElement(ordinal) * fourthVelocityCoefficient);

               accelerationJacobianMatrix.set(ordinal, startColumn + 2, basisVector.getElement(ordinal) * thirdAccelerationCoefficient);
            }
         }
      }

      timeOfContact = time;
   }

   private final FrameVector3D contactNormalVector = new FrameVector3D();

   private void computeNormalContactVectorRotation(RotationMatrix normalContactVectorRotationMatrixToPack)
   {
      contactNormalVector.setIncludingFrame(planeFrame, 0.0, 0.0, 1.0);
      EuclidGeometryTools.orientation3DFromZUpToVector3D(contactNormalVector, normalContactVectorRotationMatrixToPack);
   }

   public void clear()
   {
      for (int rhoIndex = 0; rhoIndex < basisVectorsOrigin.length; rhoIndex++)
      {
         FramePoint3D basisVectorOrigin = basisVectorsOrigin[rhoIndex];
         FrameVector3D basisVector = basisVectors[rhoIndex];

         basisVectorOrigin.setToZero(ReferenceFrame.getWorldFrame());
         basisVector.setToZero(ReferenceFrame.getWorldFrame());

         rhoMaxMatrix.set(rhoIndex, 0, Double.POSITIVE_INFINITY);
      }
   }

   private void computeBasisVector(int basisVectorIndex, double rotationOffset, RotationMatrix normalContactVectorRotationMatrix, FrameVector3D basisVectorToPack, double mu)
   {
      double angle = rotationOffset + basisVectorIndex * basisVectorAngleIncrement;

      // Compute the linear part considering a normal contact vector pointing z-up
      basisVectorToPack.setIncludingFrame(planeFrame, Math.cos(angle) * mu, Math.sin(angle) * mu, 1.0);

      // Transforming the result to consider the actual normal contact vector
      normalContactVectorRotationMatrix.transform(basisVectorToPack);
      basisVectorToPack.normalize();
   }

   private final SpatialForce unitSpatialForceVector = new SpatialForce();

   public void computeWrenchJacobianInFrame(ReferenceFrame frame, DMatrixRMaj wrenchMatrixToPack, DMatrixRMaj forceMatrixToPack)
   {
      wrenchMatrixToPack.reshape(Wrench.SIZE, numberOfBasisVectorsPerContactPoint);
      forceMatrixToPack.reshape(3, numberOfBasisVectorsPerContactPoint);
      for (int rhoIndex = 0; rhoIndex < numberOfBasisVectorsPerContactPoint; rhoIndex++)
      {
         FramePoint3D basisVectorOrigin = basisVectorsOrigin[rhoIndex];
         FrameVector3D basisVector = basisVectors[rhoIndex];
         basisVectorOrigin.changeFrame(frame);
         basisVector.changeFrame(frame);
         unitSpatialForceVector.setIncludingFrame(null, basisVector, basisVectorOrigin);
         unitSpatialForceVector.get(0, rhoIndex, wrenchMatrixToPack);
         unitSpatialForceVector.getLinearPart().get(0, rhoIndex, forceMatrixToPack);
      }
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
}
