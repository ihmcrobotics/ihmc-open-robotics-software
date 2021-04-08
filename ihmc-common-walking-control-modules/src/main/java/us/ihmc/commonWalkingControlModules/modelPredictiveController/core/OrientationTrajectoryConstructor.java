package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
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

   private final RecyclingArrayList<RecyclingArrayList<DMatrixRMaj>> AMatricesAllSegments = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(() -> new DMatrixRMaj(
         6,
         6)));
   private final RecyclingArrayList<RecyclingArrayList<DMatrixRMaj>> BMatricesAllSegments = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(() -> new DMatrixRMaj(
         6,
         0)));
   private final RecyclingArrayList<RecyclingArrayList<DMatrixRMaj>> CMatricesAllSegments = new RecyclingArrayList<>(() -> new RecyclingArrayList<>(() -> new DMatrixRMaj(
         6,
         1)));

   private final FrameVector3D desiredBodyAngularVelocityInBodyFrame = new FrameVector3D();
   private final Vector3D desiredInternalAngularMomentumRate = new Vector3D();
   private final Vector3D desiredNetAngularMomentumRate = new Vector3D();

   public OrientationTrajectoryConstructor(SE3MPCIndexHandler indexHandler, double mass, double gravity)
   {
      dynamicsCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravity);
   }

   public void compute(List<ContactPlaneProvider> previewWindowContactSequence,
                       Matrix3DReadOnly momentOfInertia,
                       LinearMPCTrajectoryHandler linearTrajectoryHandler,
                       OrientationMPCTrajectoryHandler orientationTrajectoryHandler,
                       List<? extends List<MPCContactPlane>> contactPlaneHelpers,
                       double omega)
   {
      dynamicsCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);

      AMatricesAllSegments.clear();
      BMatricesAllSegments.clear();
      CMatricesAllSegments.clear();

      double globalTime = 0.0;
      for (int segmentNumber = 0; segmentNumber < previewWindowContactSequence.size(); segmentNumber++)
      {
         RecyclingArrayList<DMatrixRMaj> AMatricesInSegment = AMatricesAllSegments.add();
         RecyclingArrayList<DMatrixRMaj> BMatricesInSegment = BMatricesAllSegments.add();
         RecyclingArrayList<DMatrixRMaj> CMatricesInSegment = CMatricesAllSegments.add();

         AMatricesInSegment.clear();
         BMatricesInSegment.clear();
         CMatricesInSegment.clear();

         double segmentDuration = previewWindowContactSequence.get(segmentNumber).getTimeInterval().getDuration();
         int ticksInSegment = computeTicksInSegment(segmentDuration);
         double tickDuration = segmentDuration / ticksInSegment;

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

            DMatrixRMaj nextA = AMatricesInSegment.add();
            DMatrixRMaj nextB = BMatricesInSegment.add();
            DMatrixRMaj nextC = CMatricesInSegment.add();

            nextB.set(dynamicsCalculator.getDiscreteBMatrix());
            nextC.set(dynamicsCalculator.getDiscreteCMatrix());

            if (tick > 0)
            {
               DMatrixRMaj previousA = AMatricesInSegment.getLast();
               DMatrixRMaj previousB = BMatricesInSegment.getLast();
               DMatrixRMaj previousC = CMatricesInSegment.getLast();

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
