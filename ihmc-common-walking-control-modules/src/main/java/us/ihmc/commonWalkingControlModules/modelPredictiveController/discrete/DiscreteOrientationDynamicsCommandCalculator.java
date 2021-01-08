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
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

import java.util.Arrays;

public class DiscreteOrientationDynamicsCommandCalculator
{
   private static final boolean shouldPrint = false;
   private static final int tickToPrint = 1;

   final DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
   final DMatrixRMaj eulerVelocityToWorld = new DMatrixRMaj(3, 3);
   final DMatrixRMaj worldVelocityToEuler = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj angularVelocity = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj tempInertia = new DMatrixRMaj(3, 3);
   final DMatrixRMaj inertiaInWorld = new DMatrixRMaj(3, 3);
   final DMatrixRMaj inertiaInWorldInverse = new DMatrixRMaj(3, 3);

   private final DiscreteMPCIndexHandler indexHandler;
   private final FrameVector3D contactLocation = new FrameVector3D();
   private final DMatrixRMaj contactLocationSkew = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj Ircross = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj Ad = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj Bd = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj fullMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj matrixExponential = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj rhsJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj rhsConstant = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj lhsJacobian = new DMatrixRMaj(0, 0);

   private final MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(0);

   private final double mass;

   public DiscreteOrientationDynamicsCommandCalculator(DiscreteMPCIndexHandler indexHandler, double mass)
   {
      this.indexHandler = indexHandler;
      this.mass = mass;

      eulerVelocityToWorld.set(2, 2, 1.0);
   }

   public void compute(DiscreteOrientationDynamicsCommand command)
   {
      double yaw = command.getOrientationEstimate().getYaw();
      double pitch = command.getOrientationEstimate().getPitch();
      double roll = command.getOrientationEstimate().getRoll();
      command.getAngularVelocityEstimate().get(angularVelocity);

      convertYawPitchRollToMatrix(yaw, pitch, roll, rotationMatrix);

      computeEulerVelocityToWorldVelocity(pitch, yaw, eulerVelocityToWorld);

      NativeCommonOps.invert(eulerVelocityToWorld, worldVelocityToEuler);

      int problemSize = indexHandler.getTotalProblemSize();

      computeInertiaInWorld(command.getBodyInertia());

      rhsJacobian.reshape(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), problemSize);
      rhsConstant.reshape(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), 1);
      lhsJacobian.reshape(DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getTotalOrientationTicks(), problemSize);

      rhsJacobian.zero();
      rhsConstant.zero();
      lhsJacobian.zero();

      MatrixTools.multAddBlock(worldVelocityToEuler, angularVelocity, rhsConstant, 0, 0);

      int tick = 0;
      double timeInHorizon = 0.0;
      for (int segment = 0; segment < command.getNumberOfSegments(); segment++)
      {
         int rhoSize = indexHandler.getRhoCoefficientsInSegment(segment);
         matrixExponentialCalculator.reshape(6 + rhoSize);
         Bd.reshape(6, rhoSize);
         fullMatrix.reshape(6 + rhoSize, 6 + rhoSize);
         matrixExponential.reshape(6 + rhoSize, 6 + rhoSize);

         int lastTick = Math.min(tick + indexHandler.getOrientationTicksInSegment(segment), indexHandler.getTotalOrientationTicks() - 1);

         for (; tick < lastTick; tick++)
         {
            Bd.zero();
            fullMatrix.zero();
            matrixExponential.zero();

            double time = tick * indexHandler.getOrientationDt();

            int ticksIntoSegment = tick - indexHandler.getOrientationTicksBeforeSegment(segment);
            boolean isLastTickInSegment = ticksIntoSegment == indexHandler.getOrientationTicksInSegment(segment);

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

                  CommonOps_DDRM.mult(inertiaInWorldInverse, contactLocationSkew, Ircross);

                  MatrixTools.multAddBlock(mass * indexHandler.getOrientationDt(), Ircross, contactPoint.getLinearJacobian(2), fullMatrix, 3, 6 + columnStart);

                  columnStart += contactPoint.getCoefficientsSize();
               }
            }

            MatrixTools.setMatrixBlock(fullMatrix, 0, 3, worldVelocityToEuler, 0, 0, 3, 3, indexHandler.getOrientationDt());
            matrixExponentialCalculator.compute(matrixExponential, fullMatrix);

            MatrixTools.setMatrixBlock(Ad, 0, 0, matrixExponential, 0, 0, 6, 6, 1.0);
            MatrixTools.setMatrixBlock(Bd, 0, 0, matrixExponential, 0, 6, 6, rhoSize, 1.0);

            MatrixTools.addMatrixBlock(rhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, indexHandler.getRhoCoefficientStartIndex(segment), Bd, 0, 0, 6, rhoSize, 1.0);

            if (shouldPrint && tick == tickToPrint)
               printInfo(segment);

            int element = DiscreteMPCIndexHandler.orientationVariablesPerTick * ticksIntoSegment + indexHandler.getOrientationStart(segment);
            int nextElement = isLastTickInSegment ?
                  indexHandler.getOrientationStart(segment + 1) :
                  element + DiscreteMPCIndexHandler.orientationVariablesPerTick;

            if (tick > 0)
               MatrixTools.setMatrixBlock(rhsJacobian, DiscreteMPCIndexHandler.orientationVariablesPerTick * tick, element, Ad, 0, 0, 6, 6, 1.0);

            for (int i = 0; i < 6; i++)
               lhsJacobian.set(DiscreteMPCIndexHandler.orientationVariablesPerTick * tick + i, nextElement + i, 1.0);
         }

         timeInHorizon += command.getSegmentDuration(segment);
      }
   }

   private void printInfo(int segment)
   {
      DMatrixRMaj nonZeroB = new DMatrixRMaj(6, indexHandler.getRhoCoefficientsInSegment(segment));
      DMatrixRMaj nonZeroA = new DMatrixRMaj(6, 6);
      MatrixTools.setMatrixBlock(nonZeroB, 3, 0, fullMatrix, 3, 6, 3, indexHandler.getRhoCoefficientsInSegment(segment), 1.0);
      MatrixTools.setMatrixBlock(nonZeroA, 0, 0, fullMatrix, 0, 0, 6, 6, 1.0 / indexHandler.getOrientationDt());

      LogTools.info("Actual A : " + nonZeroA);
      LogTools.info("Actual B : " + nonZeroB);

      DMatrixRMaj nonZeroBd = new DMatrixRMaj(6, indexHandler.getRhoCoefficientsInSegment(segment));
      MatrixTools.setMatrixBlock(nonZeroBd, 0, 0, Bd, 0, 0, 6, indexHandler.getRhoCoefficientsInSegment(segment), 1.0);

      LogTools.info("Actual Ad : " + Ad);
      LogTools.info("Actual Bd : " + nonZeroBd);
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

   public static void computeEulerVelocityToWorldVelocity(double pitch, double yaw, DMatrixRMaj matrixToPack)
   {
      double cosYaw = Math.cos(yaw);
      double sinYaw = Math.sin(yaw);
      double cosPitch = Math.cos(pitch);
      double sinPitch = Math.sin(pitch);

      matrixToPack.set(0, 0, cosPitch * cosYaw);
      matrixToPack.set(0, 1, -sinYaw);
      matrixToPack.set(1, 0, cosPitch * sinYaw);
      matrixToPack.set(1, 1, cosYaw);
      matrixToPack.set(2, 0, -sinPitch);
   }

   public static void computeWorldVelocityToEulerVelocity(double pitch, double yaw, DMatrixRMaj matrixToPack)
   {}


   public DMatrixRMaj getRhsJacobian()
   {
      return rhsJacobian;
   }

   public DMatrixRMaj getRhsConstant()
   {
      return rhsConstant;
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
