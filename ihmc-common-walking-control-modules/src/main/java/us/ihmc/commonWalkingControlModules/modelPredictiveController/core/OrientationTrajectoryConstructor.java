package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.ImplicitOrientationMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.LinearMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
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

   private final RecyclingArrayList<OrientationTrajectoryCommand> commandsForSegments = new RecyclingArrayList<>(OrientationTrajectoryCommand::new);

   private final FrameVector3D referenceBodyAngularVelocityInBodyFrame = new FrameVector3D();
   private final Vector3D desiredInternalAngularMomentumRate = new Vector3D();
   private final Vector3D desiredNetAngularMomentumRate = new Vector3D();

   private final ImplicitSE3MPCIndexHandler indexHandler;
   private final DoubleProvider orientationAngleTrackingWeight;
   private final DoubleProvider orientationVelocityTrackingWeight;

   public OrientationTrajectoryConstructor(ImplicitSE3MPCIndexHandler indexHandler,
                                           DoubleProvider orientationAngleTrackingWeight,
                                           DoubleProvider orientationVelocityTrackingWeight,
                                           double mass, double gravity)
   {
      this.indexHandler = indexHandler;
      this.orientationAngleTrackingWeight = orientationAngleTrackingWeight;
      this.orientationVelocityTrackingWeight = orientationVelocityTrackingWeight;
      dynamicsCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravity);
   }

   public List<OrientationTrajectoryCommand> getOrientationTrajectoryCommands()
   {
      return commandsForSegments;
   }

   // FIXME need to start from the current time in state
   public void compute(double currentTimeInState,
                       List<ContactPlaneProvider> previewWindowContactSequence,
                       Matrix3DReadOnly momentOfInertia,
                       LinearMPCTrajectoryHandler linearTrajectoryHandler,
                       ImplicitOrientationMPCTrajectoryHandler orientationTrajectoryHandler,
                       List<? extends List<MPCContactPlane>> contactPlaneHelpers,
                       DMatrixRMaj initialOrientationError,
                       double omega)
   {
      dynamicsCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);

      commandsForSegments.clear();

      // FIXME this time still may be wrong
      double segmentGlobalStartTime = currentTimeInState - previewWindowContactSequence.get(0).getTimeInterval().getStartTime();
      for (int segmentNumber = 0; segmentNumber < previewWindowContactSequence.size(); segmentNumber++)
      {
         OrientationTrajectoryCommand command = commandsForSegments.add();
         command.reset();
         command.setSegmentNumber(segmentNumber);
         if (segmentNumber == 0)
            command.setInitialOrientationError(initialOrientationError);

         int ticksInSegment = indexHandler.getTicksInSegment(segmentNumber);
         double tickDuration = indexHandler.getTickDuration(segmentNumber);
         double timeInSegment = 0;

         command.setAngleErrorMinimizationWeight(tickDuration * orientationAngleTrackingWeight.getValue());
         command.setVelocityErrorMinimizationWeight(tickDuration * orientationVelocityTrackingWeight.getValue());

         for (int tick = 0; tick < ticksInSegment; tick++)
         {
            linearTrajectoryHandler.compute(segmentGlobalStartTime);
            // TODO make this work in the correct time frame
            orientationTrajectoryHandler.computeDiscretizedReferenceTrajectory(segmentGlobalStartTime);

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

            dynamicsCalculator.compute(segmentNumber,
                                       linearTrajectoryHandler.getDesiredCoMPosition(),
                                       linearTrajectoryHandler.getDesiredCoMAcceleration(),
                                       referenceOrientation,
                                       referenceBodyAngularVelocityInBodyFrame,
                                       desiredNetAngularMomentumRate,
                                       desiredInternalAngularMomentumRate,
                                       contactPlaneHelpers.get(segmentNumber),
                                       timeInSegment,
                                       tickDuration,
                                       omega);

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

            segmentGlobalStartTime += tickDuration;
            timeInSegment += tickDuration;
         }
      }
   }
}
