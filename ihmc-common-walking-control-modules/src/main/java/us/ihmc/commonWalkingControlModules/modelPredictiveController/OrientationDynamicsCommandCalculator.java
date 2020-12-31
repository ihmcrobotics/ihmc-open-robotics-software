package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationDynamicsCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.MatrixMissingTools;

public class OrientationDynamicsCommandCalculator
{
   private final DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj rotationMatrixDot = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj rotationRateJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj rotationAccelerationJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj orientationJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj torqueJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj contactPointTorqueJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj tempInertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj inertiaInWorld = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj inertiaInWorldInverse = new DMatrixRMaj(3, 3);

   private final MPCIndexHandler indexHandler;
   private final Vector3D contactLocation = new Vector3D();
   private final DMatrixRMaj contactLocationSkew = new DMatrixRMaj(3, 3);

   private final double mass;

   // TODO all this math is super sparse, and could be greatly sped up by some smarter block operations

   public OrientationDynamicsCommandCalculator(MPCIndexHandler indexHandler, double mass)
   {
      this.indexHandler = indexHandler;
      this.mass = mass;

      rotationMatrix.set(2, 2, 1.0);
   }

   public DMatrixRMaj getRotationRateJacobian()
   {
      return rotationRateJacobian;
   }

   public DMatrixRMaj getRotationAccelerationJacobian()
   {
      return rotationAccelerationJacobian;
   }

   public DMatrixRMaj getTorqueJacobian()
   {
      return torqueJacobian;
   }

   public void compute(OrientationDynamicsCommand command)
   {
      double yaw = command.getOrientationEstimate().getYaw();
      double pitch = command.getOrientationEstimate().getPitch();
      double cosYaw = Math.cos(yaw);
      double sinYaw = Math.sin(yaw);
      double cosPitch = Math.cos(pitch);
      double sinPitch = Math.sin(pitch);
      double yawRate = command.getAngularVelocityEstimate().getZ();
      double pitchRate = command.getAngularVelocityEstimate().getY();

      rotationMatrix.set(0, 0, cosPitch * cosYaw);
      rotationMatrix.set(0, 1, -sinYaw);
      rotationMatrix.set(1, 0, cosPitch * sinYaw);
      rotationMatrix.set(1, 1,  cosYaw);

      rotationMatrixDot.set(0, 0, -pitchRate * sinPitch * cosYaw - yawRate * cosPitch * sinYaw);
      rotationMatrixDot.set(0, 1, -yawRate * cosYaw);
      rotationMatrixDot.set(1, 0, -pitchRate * sinPitch * sinYaw - yawRate * cosPitch * cosYaw);
      rotationMatrixDot.set(1, 1, -yawRate * sinYaw);

      int problemSize =indexHandler.getTotalProblemSize();
      rotationRateJacobian.reshape(3, problemSize);
      rotationAccelerationJacobian.reshape(3, problemSize);
      orientationJacobian.reshape(3, problemSize);
      torqueJacobian.reshape(3, problemSize);

      computeOrientationJacobians(command.getTimeOfCommand(), command.getSegmentNumber());

      CommonOps_DDRM.mult(rotationMatrixDot, rotationRateJacobian, orientationJacobian);
      CommonOps_DDRM.multAdd(rotationMatrix, rotationAccelerationJacobian, orientationJacobian);

      computeInertiaInWorld(command.getBodyInertia());

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         ContactPlaneHelper planeHelper = command.getContactPlaneHelper(i);
         planeHelper.computeJacobians(command.getTimeOfCommand(), command.getOmega());

         int columnStart = indexHandler.getRhoCoefficientStartIndex(command.getSegmentNumber());
         for (int contactPointIdx = 0; contactPointIdx < planeHelper.getNumberOfContactPoints(); contactPointIdx++)
         {
            ContactPointHelper contactPoint = planeHelper.getContactPointHelper(contactPointIdx);
            contactLocation.sub(command.getComPositionEstimate(), contactPoint.getBasisVectorOrigin());
            convertToSkewSymmetric(contactLocation, contactLocationSkew);

            contactPointTorqueJacobian.reshape(3, contactPoint.getCoefficientsSize());
            CommonOps_DDRM.mult(mass, contactLocationSkew, contactPoint.getLinearJacobian(2), contactPointTorqueJacobian);

            MatrixTools.multAddBlock(inertiaInWorldInverse, contactPointTorqueJacobian, torqueJacobian, 0, columnStart);

            columnStart += contactPoint.getCoefficientsSize();
         }
      }
   }

   private void computeOrientationJacobians(double time, int segmentId)
   {
      double c0Dot = 3.0 * time * time;
      double c1Dot = 2.0 * time;
      double c2Dot = 1.0;

      double c0Ddot = 6.0 * time;
      double c1Ddot = 2.0;

      int rollStartIndex = indexHandler.getRollCoefficientsStartIndex(segmentId);
      int pitchStartIndex = indexHandler.getPitchCoefficientsStartIndex(segmentId);
      int yawStartIndex = indexHandler.getYawCoefficientsStartIndex(segmentId);

//      OrientationCoefficientJacobianCalculator.calculateAngularVelocityJacobian(rollStartIndex, time, rotationRateJacobian, 1.0);
//      OrientationCoefficientJacobianCalculator.calculateAngularAccelerationJacobian(rollStartIndex, time, rotationAccelerationJacobian, 1.0);
      rotationRateJacobian.set(0, rollStartIndex, c0Dot);
      rotationRateJacobian.set(0, rollStartIndex + 1, c1Dot);
      rotationRateJacobian.set(0, rollStartIndex + 2, c2Dot);

      rotationRateJacobian.set(1, pitchStartIndex, c0Dot);
      rotationRateJacobian.set(1, pitchStartIndex + 1, c1Dot);
      rotationRateJacobian.set(1, pitchStartIndex + 2, c2Dot);

      rotationRateJacobian.set(2, yawStartIndex, c0Dot);
      rotationRateJacobian.set(2, yawStartIndex + 1, c1Dot);
      rotationRateJacobian.set(2, yawStartIndex + 2, c2Dot);

      rotationAccelerationJacobian.set(0, rollStartIndex, c0Ddot);
      rotationAccelerationJacobian.set(0, rollStartIndex + 1, c1Ddot);

      rotationAccelerationJacobian.set(1, pitchStartIndex, c0Ddot);
      rotationAccelerationJacobian.set(1, pitchStartIndex + 1, c1Ddot);

      rotationAccelerationJacobian.set(2, yawStartIndex, c0Ddot);
      rotationAccelerationJacobian.set(2, yawStartIndex + 1, c1Ddot);
   }

   private void computeInertiaInWorld(SpatialInertiaReadOnly spatialInertia)
   {
      spatialInertia.getMomentOfInertia().get(inertia);
      CommonOps_DDRM.mult(rotationMatrix, inertia, tempInertia);
      CommonOps_DDRM.multTransB(tempInertia, rotationMatrix, inertiaInWorld);

      MatrixMissingTools.fast3x3Inverse(inertiaInWorld, inertiaInWorldInverse);
   }

   private static void convertToSkewSymmetric(Tuple3DReadOnly tuple, DMatrixRMaj skewMatrix)
   {
      skewMatrix.set(0, 1, -tuple.getZ());
      skewMatrix.set(1, 0, tuple.getZ());
      skewMatrix.set(0, 2, tuple.getY());
      skewMatrix.set(2, 0, -tuple.getY());
      skewMatrix.set(1, 2, -tuple.getX());
      skewMatrix.set(2, 1, tuple.getX());
   }
}
