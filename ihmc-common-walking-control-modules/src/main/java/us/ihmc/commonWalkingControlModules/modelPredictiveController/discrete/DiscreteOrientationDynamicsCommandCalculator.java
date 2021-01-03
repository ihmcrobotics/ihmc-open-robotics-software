package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPointHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteOrientationDynamicsCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

import java.util.Arrays;

public class DiscreteOrientationDynamicsCommandCalculator
{
   final DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
   final DMatrixRMaj bodyVelocityToWorld = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj rotationRateJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj rotationAccelerationJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj orientationJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj angularAccelerationJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj contactPointTorqueJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj tempInertia = new DMatrixRMaj(3, 3);
   final DMatrixRMaj inertiaInWorld = new DMatrixRMaj(3, 3);
   final DMatrixRMaj inertiaInWorldInverse = new DMatrixRMaj(3, 3);

   private final DiscreteMPCIndexHandler indexHandler;
   private final FrameVector3D contactLocation = new FrameVector3D();
   private final DMatrixRMaj contactLocationSkew = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj Ad = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj Bd = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj fullMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj matrixExponential = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj stateTransitionMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj stateControlMatrix = new DMatrixRMaj(0, 0);

   private final MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(0);

   private final double mass;

   public DiscreteOrientationDynamicsCommandCalculator(DiscreteMPCIndexHandler indexHandler, double mass)
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

   public DMatrixRMaj getAngularAccelerationJacobian()
   {
      return angularAccelerationJacobian;
   }

   public void compute(DiscreteOrientationDynamicsCommand command)
   {
      double yaw = command.getOrientationEstimate().getYaw();
      double pitch = command.getOrientationEstimate().getPitch();
      double roll = command.getOrientationEstimate().getRoll();
      double cosYaw = Math.cos(yaw);
      double sinYaw = Math.sin(yaw);
      double cosPitch = Math.cos(pitch);
      double sinPitch = Math.sin(pitch);

      convertYawPitchRollToMatrix(yaw, pitch, roll, rotationMatrix);

      bodyVelocityToWorld.set(0, 0, cosPitch * cosYaw);
      bodyVelocityToWorld.set(0, 1, -sinYaw);
      bodyVelocityToWorld.set(1, 0, cosPitch * sinYaw);
      bodyVelocityToWorld.set(1, 1, cosYaw);
      bodyVelocityToWorld.set(2, 1, -sinPitch);

      // TODO may not need to do the intermediate step
      MatrixTools.setMatrixBlock(A, 0, 3, bodyVelocityToWorld, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(fullMatrix, 0, 3, bodyVelocityToWorld, 0, 0, 3, 3, 1.0);

      int problemSize = indexHandler.getTotalProblemSize();
      B.reshape(6, problemSize);
      Bd.reshape(6, problemSize);
      fullMatrix.reshape(6 + problemSize, 6 + problemSize);
      matrixExponential.reshape(6 + problemSize, 6 + problemSize);

      computeInertiaInWorld(command.getBodyInertia());

      matrixExponentialCalculator.reshape(6 + problemSize);

      stateTransitionMatrix.reshape(indexHandler.getTotalOrientationTicks(), indexHandler.getTotalOrientationTicks());
      stateControlMatrix.reshape(indexHandler.getTotalOrientationTicks(), problemSize);

      int tick = 0;
      double timeInHorizon = 0.0;
      for (int segment = 0; segment < command.getNumberOfSegments(); segment++)
      {
         for (int i = 0; i < command.getNumberOfContacts(segment); i++)
         {
            ContactPlaneHelper planeHelper = command.getContactPlaneHelper(segment, i);
            // TODO should deal with the contact points and acceleration jacobians individually, as there are a lot of repeated and extra calculations in here.
            for (; tick < tick + indexHandler.getOrientationTicksInSegment(segment); tick++)
            {
               double time = tick * DiscreteMPCIndexHandler.orientationDt;

               planeHelper.computeJacobians(time - timeInHorizon, command.getOmega());
               angularAccelerationJacobian.zero();

               int columnStart = indexHandler.getRhoCoefficientStartIndex(segment);
               for (int contactPointIdx = 0; contactPointIdx < planeHelper.getNumberOfContactPoints(); contactPointIdx++)
               {
                  ContactPointHelper contactPoint = planeHelper.getContactPointHelper(contactPointIdx);
                  contactLocation.sub(contactPoint.getBasisVectorOrigin(), command.getComPositionEstimate());
                  convertToSkewSymmetric(contactLocation, contactLocationSkew);

                  contactPointTorqueJacobian.reshape(3, contactPoint.getCoefficientsSize());
                  CommonOps_DDRM.mult(mass, contactLocationSkew, contactPoint.getLinearJacobian(2), contactPointTorqueJacobian);

                  MatrixTools.multAddBlock(inertiaInWorldInverse, contactPointTorqueJacobian, angularAccelerationJacobian, 0, columnStart);

                  columnStart += contactPoint.getCoefficientsSize();
               }

               MatrixTools.setMatrixBlock(B, 3, 0, angularAccelerationJacobian, 0, 0, 3, problemSize, 1.0);

               MatrixTools.setMatrixBlock(fullMatrix, 0, 6, B, 0, 0, 6, problemSize, 1.0);
               matrixExponentialCalculator.compute(matrixExponential, fullMatrix);

               MatrixTools.setMatrixBlock(Ad, 0, 0, matrixExponential, 0, 0, 6, 6, 1.0);
               MatrixTools.setMatrixBlock(Bd, 0, 0, matrixExponential, 6, 0, 6, problemSize, 1.0);

               int element = 6 * tick;
               int nextElement = 6 * tick;
               MatrixTools.setMatrixBlock(stateTransitionMatrix, nextElement, element, Ad, 0, 0, 6, 6, 1.0);
               // FIXME
               MatrixTools.setMatrixBlock(stateControlMatrix, nextElement, element, Bd, 0, 0, 6, problemSize, 1.0);
            }

            timeInHorizon += command.getSegmentDuration(segment);
         }
      }

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
