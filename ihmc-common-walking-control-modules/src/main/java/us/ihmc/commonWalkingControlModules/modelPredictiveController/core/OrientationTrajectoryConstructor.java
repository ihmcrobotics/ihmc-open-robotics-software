package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.ImplicitOrientationMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.LinearMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.OrientationMPCTrajectoryHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.List;

public class OrientationTrajectoryConstructor
{
   private static final double intermediateDt = 0.01;

   private final OrientationDynamicsCalculator dynamicsCalculator;

   private final RecyclingArrayList<OrientationTrajectoryCommand> commandsForSegments = new RecyclingArrayList<>(OrientationTrajectoryCommand::new);

   private final FrameVector3D desiredBodyAngularVelocityInBodyFrame = new FrameVector3D();
   private final Vector3D desiredInternalAngularMomentumRate = new Vector3D();
   private final Vector3D desiredNetAngularMomentumRate = new Vector3D();

   private final TIntArrayList ticksInSegment = new TIntArrayList();
   private final TDoubleArrayList tickDurations = new TDoubleArrayList();

   public OrientationTrajectoryConstructor(LinearMPCIndexHandler indexHandler, double mass, double gravity)
   {
      dynamicsCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravity);
   }

   public List<OrientationTrajectoryCommand> getOrientationTrajectoryCommands()
   {
      return commandsForSegments;
   }

   public int getTicksInSegment(int segment)
   {
      return ticksInSegment.get(segment);
   }

   public double getTickDuration(int segment)
   {
      return tickDurations.get(segment);
   }

   // FIXME need to start from the current time in state
   public void compute(List<ContactPlaneProvider> previewWindowContactSequence,
                       Matrix3DReadOnly momentOfInertia,
                       LinearMPCTrajectoryHandler linearTrajectoryHandler,
                       ImplicitOrientationMPCTrajectoryHandler orientationTrajectoryHandler,
                       List<? extends List<MPCContactPlane>> contactPlaneHelpers,
                       DMatrixRMaj initialOrientationError,
                       double omega)
   {
      dynamicsCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);

      commandsForSegments.clear();
      tickDurations.reset();
      ticksInSegment.reset();

      double globalTime = 0.0;
      for (int segmentNumber = 0; segmentNumber < previewWindowContactSequence.size(); segmentNumber++)
      {
         OrientationTrajectoryCommand command = commandsForSegments.add();
         command.reset();
         command.setSegmentNumber(segmentNumber);
         if (segmentNumber == 0)
            command.setInitialOrientationError(initialOrientationError);

         double segmentDuration = previewWindowContactSequence.get(segmentNumber).getTimeInterval().getDuration();
         int ticksInSegment = computeTicksInSegment(segmentDuration);
         double tickDuration = segmentDuration / ticksInSegment;

         this.ticksInSegment.add(ticksInSegment);
         this.tickDurations.add(tickDuration);

         for (int tick = 0; tick < ticksInSegment; tick++)
         {
            globalTime += tickDuration;

            linearTrajectoryHandler.compute(globalTime);
            orientationTrajectoryHandler.compute(globalTime);

            FrameOrientation3DReadOnly desiredOrientation = orientationTrajectoryHandler.getDesiredBodyOrientationOutsidePreview();
            // angular velocity in body frame
            desiredBodyAngularVelocityInBodyFrame.set(orientationTrajectoryHandler.getDesiredBodyVelocityOutsidePreview());
            desiredOrientation.transform(desiredBodyAngularVelocityInBodyFrame);

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
                                       desiredOrientation,
                                       desiredBodyAngularVelocityInBodyFrame,
                                       desiredNetAngularMomentumRate,
                                       desiredInternalAngularMomentumRate,
                                       contactPlaneHelpers.get(segmentNumber),
                                       globalTime,
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
         }
      }
   }

   private static int computeTicksInSegment(double segmentDuration)
   {
      if (segmentDuration > 0.0 && segmentDuration < intermediateDt)
         return 1;
      else
         return (int) Math.floor(segmentDuration / intermediateDt);
   }
}
