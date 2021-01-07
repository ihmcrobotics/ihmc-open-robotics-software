package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPointHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCIndexHandler;
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
   private final DMatrixRMaj angularTorqueJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj angularAccelerationJacobian = new DMatrixRMaj(3, 0);
   private final DMatrixRMaj contactPointTorqueJacobian = new DMatrixRMaj(3, 0);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj tempInertia = new DMatrixRMaj(3, 3);
   final DMatrixRMaj inertiaInWorld = new DMatrixRMaj(3, 3);
   final DMatrixRMaj inertiaInWorldInverse = new DMatrixRMaj(3, 3);

   private final DiscreteMPCIndexHandler indexHandler;
   private final FrameVector3D contactLocation = new FrameVector3D();
   private final DMatrixRMaj contactLocationSkew = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj B = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj Ad = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj Bd = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj fullMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj matrixExponential = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj rhsJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj lhsJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj identity = CommonOps_DDRM.identity(6);
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

      int problemSize = indexHandler.getTotalProblemSize();

      computeInertiaInWorld(command.getBodyInertia());

      matrixExponentialCalculator.reshape(6 + problemSize);

      rhsJacobian.reshape(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), problemSize);
      lhsJacobian.reshape(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), problemSize);

      int tick = 0;
      double timeInHorizon = 0.0;
      for (int segment = 0; segment < command.getNumberOfSegments(); segment++)
      {
         int rhoSize = indexHandler.getRhoCoefficientsInSegment(segment);
         matrixExponentialCalculator.reshape(6 + indexHandler.getTotalProblemSize());

         angularAccelerationJacobian.reshape(3, indexHandler.getRhoCoefficientsInSegment(segment));

         B.reshape(6, rhoSize);
         Bd.reshape(6, indexHandler.getTotalProblemSize());
         fullMatrix.reshape(6 + indexHandler.getTotalProblemSize(), 6 + indexHandler.getTotalProblemSize());
         matrixExponential.reshape(6 + indexHandler.getTotalProblemSize(), 6 + indexHandler.getTotalProblemSize());

         B.zero();
         Bd.zero();
         fullMatrix.zero();
         matrixExponential.zero();


         // TODO should deal with the contact points and acceleration jacobians individually, as there are a lot of repeated and extra calculations in here.
         int lastTick = Math.min(tick + indexHandler.getOrientationTicksInSegment(segment), indexHandler.getTotalOrientationTicks() - 1);

         for (; tick < lastTick; tick++)
         {
            double time = tick * indexHandler.getOrientationDt();

            int ticksIntoSegment = tick - indexHandler.getOrientationTicksBeforeSegment(segment);
            boolean isLastTickInSegment = ticksIntoSegment == indexHandler.getOrientationTicksInSegment(segment);


            angularAccelerationJacobian.zero();

            for (int i = 0; i < command.getNumberOfContacts(segment); i++)
            {
               ContactPlaneHelper planeHelper = command.getContactPlaneHelper(segment, i);

               planeHelper.computeJacobians(time - timeInHorizon, command.getOmega());

               int columnStart = 0;
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

               MatrixTools.addMatrixBlock(B, 3, 0, angularAccelerationJacobian, 0, 0, 3, rhoSize, 1.0);
            }

            fullMatrix.zero();
            MatrixTools.setMatrixBlock(fullMatrix, 0, 3, bodyVelocityToWorld, 0, 0, 3, 3, 1.0);
            MatrixTools.setMatrixBlock(fullMatrix, 0, 6 + indexHandler.getRhoCoefficientStartIndex(segment), B, 0, 0, 6, rhoSize, 1.0);
            CommonOps_DDRM.scale(indexHandler.getOrientationDt(), fullMatrix);
            matrixExponentialCalculator.compute(matrixExponential, fullMatrix);

            MatrixTools.setMatrixBlock(Ad, 0, 0, matrixExponential, 0, 0, 6, 6, 1.0);
            MatrixTools.setMatrixBlock(Bd, 0, 0, matrixExponential, 0, 6, 6, indexHandler.getTotalProblemSize(), 1.0);

            MatrixTools.addMatrixBlock(rhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, 0, Bd, 0, 0, 6, indexHandler.getTotalProblemSize(), 1.0);

            int element = DiscreteMPCIndexHandler.orientationVariablesPerTick * ticksIntoSegment + indexHandler.getOrientationStart(segment);
            int nextElement = isLastTickInSegment ?
                  indexHandler.getOrientationStart(segment + 1) :
                  element + DiscreteMPCIndexHandler.orientationVariablesPerTick;

            if (tick > 0)
               MatrixTools.setMatrixBlock(rhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, element, Ad, 0, 0, 6, 6, 1.0);
            MatrixTools.setMatrixBlock(lhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, nextElement, identity, 0, 0, 6, 6, 1.0);

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

   public static void convertToSkewSymmetric(Tuple3DReadOnly tuple, DMatrixRMaj skewMatrix)
   {
      skewMatrix.set(0, 1, -tuple.getZ());
      skewMatrix.set(1, 0, tuple.getZ());
      skewMatrix.set(0, 2, tuple.getY());
      skewMatrix.set(2, 0, -tuple.getY());
      skewMatrix.set(1, 2, -tuple.getX());
      skewMatrix.set(2, 1, tuple.getX());
   }

   public DMatrixRMaj getRhsJacobian()
   {
      return rhsJacobian;
   }

   public DMatrixRMaj getLhsJacobian()
   {
      return lhsJacobian;
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
