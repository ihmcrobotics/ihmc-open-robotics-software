package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPointHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationDynamicsCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.Arrays;

public class OrientationDynamicsCommandCalculator
{
   final DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
   final DMatrixRMaj bodyVelocityToWorld = new DMatrixRMaj(3, 3);
   final DMatrixRMaj bodyAccelerationToWorld = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj rotationRateJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj rotationAccelerationJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj orientationJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj torqueJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj contactPointTorqueJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj tempInertia = new DMatrixRMaj(3, 3);
   final DMatrixRMaj inertiaInWorld = new DMatrixRMaj(3, 3);
   final DMatrixRMaj inertiaInWorldInverse = new DMatrixRMaj(3, 3);

   private final ContinuousMPCIndexHandler indexHandler;
   private final FrameVector3D contactLocation = new FrameVector3D();
   private final DMatrixRMaj contactLocationSkew = new DMatrixRMaj(3, 3);

   private final double mass;

   // TODO all this math is super sparse, and could be greatly sped up by some smarter block operations

   public OrientationDynamicsCommandCalculator(ContinuousMPCIndexHandler indexHandler, double mass)
   {
      this.indexHandler = indexHandler;
      this.mass = mass;

      bodyVelocityToWorld.set(2, 2, 1.0);
   }

   public DMatrixRMaj getRotationRateJacobian()
   {
      return rotationRateJacobian;
   }

   public DMatrixRMaj getRotationAccelerationJacobian()
   {
      return rotationAccelerationJacobian;
   }

   public DMatrixRMaj getOrientationJacobian()
   {
      return orientationJacobian;
   }

   public DMatrixRMaj getTorqueJacobian()
   {
      return torqueJacobian;
   }

   public void compute(OrientationDynamicsCommand command)
   {
      double yaw = command.getOrientationEstimate().getYaw();
      double pitch = command.getOrientationEstimate().getPitch();
      double roll = command.getOrientationEstimate().getRoll();
      double cosYaw = Math.cos(yaw);
      double sinYaw = Math.sin(yaw);
      double cosPitch = Math.cos(pitch);
      double sinPitch = Math.sin(pitch);
      double yawRate = command.getAngularVelocityEstimate().getZ();
      double pitchRate = command.getAngularVelocityEstimate().getY();

      convertYawPitchRollToMatrix(yaw, pitch, roll, rotationMatrix);

      bodyVelocityToWorld.set(0, 0, cosPitch * cosYaw);
      bodyVelocityToWorld.set(0, 1, -sinYaw);
      bodyVelocityToWorld.set(1, 0, cosPitch * sinYaw);
      bodyVelocityToWorld.set(1, 1, cosYaw);
      bodyVelocityToWorld.set(2, 1, -sinPitch);

      bodyAccelerationToWorld.set(0, 0, -pitchRate * sinPitch * cosYaw - yawRate * cosPitch * sinYaw);
      bodyAccelerationToWorld.set(0, 1, -yawRate * cosYaw);
      bodyAccelerationToWorld.set(1, 0, -pitchRate * sinPitch * sinYaw + yawRate * cosPitch * cosYaw);
      bodyAccelerationToWorld.set(1, 1, -yawRate * sinYaw);
      bodyAccelerationToWorld.set(2, 1, -pitchRate * cosPitch);

      int problemSize = indexHandler.getTotalProblemSize();
      rotationRateJacobian.reshape(3, problemSize);
      rotationAccelerationJacobian.reshape(3, problemSize);
      orientationJacobian.reshape(3, problemSize);
      torqueJacobian.reshape(3, problemSize);

      rotationRateJacobian.zero();
      rotationAccelerationJacobian.zero();
      orientationJacobian.zero();
      torqueJacobian.zero();

      computeOrientationJacobians(command.getOmega(), command.getTimeOfCommand(), command.getSegmentNumber());

      CommonOps_DDRM.mult(bodyAccelerationToWorld, rotationRateJacobian, orientationJacobian);
      CommonOps_DDRM.multAdd(bodyVelocityToWorld, rotationAccelerationJacobian, orientationJacobian);

      computeInertiaInWorld(command.getBodyInertia());

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         ContactPlaneHelper planeHelper = command.getContactPlaneHelper(i);
         planeHelper.computeJacobians(command.getTimeOfCommand(), command.getOmega());

         int columnStart = indexHandler.getRhoCoefficientStartIndex(command.getSegmentNumber());
         for (int contactPointIdx = 0; contactPointIdx < planeHelper.getNumberOfContactPoints(); contactPointIdx++)
         {
            ContactPointHelper contactPoint = planeHelper.getContactPointHelper(contactPointIdx);
            contactLocation.sub(contactPoint.getBasisVectorOrigin(), command.getComPositionEstimate());
            convertToSkewSymmetric(contactLocation, contactLocationSkew);

            contactPointTorqueJacobian.reshape(3, contactPoint.getCoefficientsSize());
            CommonOps_DDRM.mult(mass, contactLocationSkew, contactPoint.getLinearJacobian(2), contactPointTorqueJacobian);

            MatrixTools.multAddBlock(inertiaInWorldInverse, contactPointTorqueJacobian, torqueJacobian, 0, columnStart);

            columnStart += contactPoint.getCoefficientsSize();
         }
      }
   }

   private void computeOrientationJacobians(double omega, double time, int segmentId)
   {
      int rollStartIndex = indexHandler.getRollCoefficientsStartIndex(segmentId);
      int pitchStartIndex = indexHandler.getPitchCoefficientsStartIndex(segmentId);
      int yawStartIndex = indexHandler.getYawCoefficientsStartIndex(segmentId);

      OrientationCoefficientJacobianCalculator.calculateAngularVelocityJacobian(yawStartIndex, pitchStartIndex, rollStartIndex, omega, time, rotationRateJacobian, 1.0);
      OrientationCoefficientJacobianCalculator.calculateAngularAccelerationJacobian(yawStartIndex, pitchStartIndex, rollStartIndex, omega, time, rotationAccelerationJacobian, 1.0);
   }

   private void computeInertiaInWorld(SpatialInertiaReadOnly spatialInertia)
   {
      spatialInertia.getMomentOfInertia().get(inertia);
      CommonOps_DDRM.mult(rotationMatrix, inertia, tempInertia);
      CommonOps_DDRM.multTransB(tempInertia, rotationMatrix, inertiaInWorld);

      MatrixMissingTools.fast3x3Inverse(inertiaInWorld, inertiaInWorldInverse);
   }

   static void convertToSkewSymmetric(Tuple3DReadOnly tuple, DMatrixRMaj skewMatrix)
   {
      skewMatrix.set(0, 1, -tuple.getZ());
      skewMatrix.set(1, 0, tuple.getZ());
      skewMatrix.set(0, 2, tuple.getY());
      skewMatrix.set(2, 0, -tuple.getY());
      skewMatrix.set(1, 2, -tuple.getX());
      skewMatrix.set(2, 1, tuple.getX());
   }

   private static void convertYawPitchRollToMatrix(double yaw, double pitch, double roll, DMatrixRMaj matrixToPack)
   {
      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         Arrays.fill(matrixToPack.data, 0, matrixToPack.getNumElements(), 0.0);
         return;
      }

      if (YawPitchRollTools.isZero(yaw, pitch, roll, 1e-5))
      {
         CommonOps_DDRM.setIdentity(matrixToPack);
         return;
      }

      double cosc = EuclidCoreTools.cos(yaw);
      double sinc = EuclidCoreTools.sin(yaw);

      double cosb = EuclidCoreTools.cos(pitch);
      double sinb = EuclidCoreTools.sin(pitch);

      double cosa = EuclidCoreTools.cos(roll);
      double sina = EuclidCoreTools.sin(roll);

      // Introduction to Robotics, 2.64
      double m00 = cosc * cosb;
      double m01 = cosc * sinb * sina - sinc * cosa;
      double m02 = cosc * sinb * cosa + sinc * sina;
      double m10 = sinc * cosb;
      double m11 = sinc * sinb * sina + cosc * cosa;
      double m12 = sinc * sinb * cosa - cosc * sina;
      double m20 = -sinb;
      double m21 = cosb * sina;
      double m22 = cosb * cosa;
      matrixToPack.set(0, 0, m00);
      matrixToPack.set(0, 1, m01);
      matrixToPack.set(0, 2, m02);
      matrixToPack.set(1, 0, m10);
      matrixToPack.set(1, 1, m11);
      matrixToPack.set(1, 2, m12);
      matrixToPack.set(2, 0, m20);
      matrixToPack.set(2, 1, m21);
      matrixToPack.set(2, 2, m22);
   }
}
