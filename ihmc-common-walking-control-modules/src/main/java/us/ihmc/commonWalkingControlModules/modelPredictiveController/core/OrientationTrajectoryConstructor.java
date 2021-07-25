package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.LinearMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.OrientationMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowSegment;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.List;

public class OrientationTrajectoryConstructor
{
   private static final boolean debug = true;

   private final OrientationDynamicsCalculator dynamicsCalculator;

   private final RecyclingArrayList<OrientationTrajectoryCommand> trajectoryCommandsForSegments = new RecyclingArrayList<>(OrientationTrajectoryCommand::new);

   private final FrameVector3D referenceBodyAngularVelocityInBodyFrame = new FrameVector3D();
   private final Vector3D desiredInternalAngularMomentumRate = new Vector3D();
   private final Vector3D desiredNetAngularMomentumRate = new Vector3D();

   private final Vector3D gravityVector = new Vector3D();
   private final SE3MPCIndexHandler indexHandler;
   private final DoubleProvider orientationAngleTrackingWeight;
   private final DoubleProvider orientationVelocityTrackingWeight;

   private final DoubleProvider omega;
   private final double gravityZ;

   public OrientationTrajectoryConstructor(SE3MPCIndexHandler indexHandler,
                                           DoubleProvider orientationAngleTrackingWeight,
                                           DoubleProvider orientationVelocityTrackingWeight,
                                           DoubleProvider omega,
                                           double mass,
                                           double gravity)
   {
      this.indexHandler = indexHandler;
      this.orientationAngleTrackingWeight = orientationAngleTrackingWeight;
      this.orientationVelocityTrackingWeight = orientationVelocityTrackingWeight;
      this.omega = omega;
      this.gravityZ = gravity;

      gravityVector.setZ(-Math.abs(gravityZ));

      dynamicsCalculator = new OrientationDynamicsCalculator(mass, gravity);
   }

   public List<OrientationTrajectoryCommand> getOrientationTrajectoryCommands()
   {
      return trajectoryCommandsForSegments;
   }

   public void compute(List<PreviewWindowSegment> previewWindowContactSequence,
                       Matrix3DReadOnly momentOfInertia,
                       LinearMPCTrajectoryHandler linearTrajectoryHandler,
                       OrientationMPCTrajectoryHandler orientationTrajectoryHandler,
                       List<? extends List<MPCContactPlane>> contactPlaneHelpers)
   {
      dynamicsCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);

      trajectoryCommandsForSegments.clear();

      double trajectoryStartTime = previewWindowContactSequence.get(0).getStartTime();

      for (int segmentNumber = 0; segmentNumber < previewWindowContactSequence.size(); segmentNumber++)
      {
         OrientationTrajectoryCommand command = trajectoryCommandsForSegments.add();
         command.reset();
         command.setSegmentNumber(segmentNumber);

         int ticksInSegment = indexHandler.getTicksInSegment(segmentNumber);
         double tickDuration = indexHandler.getTickDuration(segmentNumber);
         double timeInSegment = 0.0;

         command.setAngleErrorMinimizationWeight(tickDuration * orientationAngleTrackingWeight.getValue());
         command.setVelocityErrorMinimizationWeight(tickDuration * orientationVelocityTrackingWeight.getValue());

         for (int tick = 0; tick < ticksInSegment; tick++)
         {
            linearTrajectoryHandler.compute(trajectoryStartTime);
            orientationTrajectoryHandler.computeDiscretizedReferenceTrajectory(trajectoryStartTime);

            FrameOrientation3DReadOnly referenceOrientation = orientationTrajectoryHandler.getReferenceBodyOrientation();
            // angular velocity in body frame
            referenceBodyAngularVelocityInBodyFrame.set(orientationTrajectoryHandler.getReferenceBodyVelocity());
            referenceOrientation.transform(referenceBodyAngularVelocityInBodyFrame);

            if (false)//orientationTrajectoryHandler.hasInternalAngularMomentum())
            {
               desiredInternalAngularMomentumRate.set(orientationTrajectoryHandler.getDesiredInternalAngularMomentumRate());
               if (contactPlaneHelpers.get(segmentNumber).size() > 0)
                  desiredNetAngularMomentumRate.set(orientationTrajectoryHandler.getDesiredInternalAngularMomentumRate());
               else
                  desiredNetAngularMomentumRate.setToZero();
            }
            else
            {
               desiredNetAngularMomentumRate.setToZero();
               desiredInternalAngularMomentumRate.setToZero();
            }

            FrameVector3D acceleration = new FrameVector3D(linearTrajectoryHandler.getDesiredCoMAcceleration());
            if (debug && contactPlaneHelpers.get(segmentNumber).size() == 0)
            {
               acceleration.set(gravityVector);
            }

            dynamicsCalculator.compute(linearTrajectoryHandler.getDesiredCoMPosition(),
                                       acceleration,
                                       referenceOrientation,
                                       referenceBodyAngularVelocityInBodyFrame,
                                       desiredNetAngularMomentumRate,
                                       desiredInternalAngularMomentumRate,
                                       contactPlaneHelpers.get(segmentNumber),
                                       timeInSegment,
                                       tickDuration,
                                       omega.getValue());

            DMatrixRMaj previousA = command.getLastAMatrix();
            DMatrixRMaj previousB = command.getLastBMatrix();
            DMatrixRMaj previousC = command.getLastCMatrix();

            DMatrixRMaj nextA = command.addAMatrix();
            DMatrixRMaj nextB = command.addBMatrix();
            DMatrixRMaj nextC = command.addCMatrix();

            nextB.set(dynamicsCalculator.getDiscreteBMatrix());
            nextC.set(dynamicsCalculator.getDiscreteCMatrix());

            if (tick > 0)
            {
               CommonOps_DDRM.mult(dynamicsCalculator.getDiscreteAMatrix(), previousA, nextA);
               CommonOps_DDRM.multAdd(dynamicsCalculator.getDiscreteAMatrix(), previousB, nextB);
               CommonOps_DDRM.multAdd(dynamicsCalculator.getDiscreteAMatrix(), previousC, nextC);
            }
            else
            {
               nextA.set(dynamicsCalculator.getDiscreteAMatrix());
            }

            if (debug && contactPlaneHelpers.get(segmentNumber).size() == 0)
            {
               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB0()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. B0 should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB0()));
               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB1()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. B1 should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB1()));
               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB2()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. B2 should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB2()));
               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB3()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. B3 should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB3()));
               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB4()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. B4 should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getB4()));

               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getContinuousBMatrix()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. Continuous B should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getContinuousBMatrix()));
               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getContinuousCMatrix()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. Continuous C should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getContinuousCMatrix()));

               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getDiscreteBMatrix()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. Discrete B should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getDiscreteBMatrix()));
               if (!MathTools.epsilonEquals(CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getDiscreteCMatrix()), 0.0, 1e-3))
                  throw new RuntimeException("Poopy. Discrete C should be zero. Actually " + CommonOps_DDRM.elementMaxAbs(dynamicsCalculator.getDiscreteCMatrix()));
            }

            trajectoryStartTime += tickDuration;
            timeInSegment += tickDuration;
         }
      }
   }
}
