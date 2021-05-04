package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.LinearMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.OrientationMPCTrajectoryHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.List;

public class OrientationTrajectoryConstructor
{
   private final OrientationDynamicsCalculator dynamicsCalculator;

   private final RecyclingArrayList<OrientationTrajectoryCommand> trajectoryCommandsForSegments = new RecyclingArrayList<>(OrientationTrajectoryCommand::new);

   private final FrameVector3D referenceBodyAngularVelocityInBodyFrame = new FrameVector3D();
   private final Vector3D desiredInternalAngularMomentumRate = new Vector3D();
   private final Vector3D desiredNetAngularMomentumRate = new Vector3D();

   private final SE3MPCIndexHandler indexHandler;
   private final DoubleProvider orientationAngleTrackingWeight;
   private final DoubleProvider orientationVelocityTrackingWeight;

   private final DoubleProvider omega;

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

      dynamicsCalculator = new OrientationDynamicsCalculator(mass, gravity);
   }

   public List<OrientationTrajectoryCommand> getOrientationTrajectoryCommands()
   {
      return trajectoryCommandsForSegments;
   }

   public void compute(List<ContactPlaneProvider> previewWindowContactSequence,
                       Matrix3DReadOnly momentOfInertia,
                       LinearMPCTrajectoryHandler linearTrajectoryHandler,
                       OrientationMPCTrajectoryHandler orientationTrajectoryHandler,
                       List<? extends List<MPCContactPlane>> contactPlaneHelpers)
   {
      dynamicsCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);

      trajectoryCommandsForSegments.clear();

      double trajectoryStartTime = previewWindowContactSequence.get(0).getTimeInterval().getStartTime();

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

            if (orientationTrajectoryHandler.hasInternalAngularMomentum())
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

            dynamicsCalculator.compute(linearTrajectoryHandler.getDesiredCoMPosition(),
                                       linearTrajectoryHandler.getDesiredCoMAcceleration(),
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

            trajectoryStartTime += tickDuration;
            timeInSegment += tickDuration;
         }
      }
   }
}
