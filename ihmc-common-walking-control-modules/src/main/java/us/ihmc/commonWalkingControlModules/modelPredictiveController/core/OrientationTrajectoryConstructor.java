package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.CMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
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
   private final OrientationDynamicsCalculator dynamicsCalculator;
   private final SE3MPCIndexHandler indexHandler;

   private final RecyclingArrayList<DMatrixRMaj> AMatrices = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 6));
   private final RecyclingArrayList<DMatrixRMaj> BMatrices = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 0));
   private final RecyclingArrayList<DMatrixRMaj> CMatrices = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 1));

   private final FrameVector3D desiredBodyAngularVelocityInBodyFrame = new FrameVector3D();
   private final Vector3D desiredInternalAngularMomentumRate = new Vector3D();
   private final Vector3D desiredNetAngularMomentumRate = new Vector3D();

   public OrientationTrajectoryConstructor(SE3MPCIndexHandler indexHandler, double mass, double gravity)
   {
      this.indexHandler = indexHandler;
      dynamicsCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravity);
   }

   public void compute(Matrix3DReadOnly momentOfInertia,
                       LinearMPCTrajectoryHandler linearTrajectoryHandler,
                       OrientationMPCTrajectoryHandler orientationTrajectoryHandler,
                       List<? extends List<MPCContactPlane>> contactPlaneHelpers,
                       double omega)
   {
      dynamicsCalculator.setMomentumOfInertiaInBodyFrame(momentOfInertia);

      AMatrices.clear();
      BMatrices.clear();
      CMatrices.clear();

      CommonOps_DDRM.setIdentity(AMatrices.add());
      DMatrixRMaj BStart = BMatrices.add();
      BStart.reshape(6, indexHandler.getTotalProblemSize());
      BStart.zero();
      CMatrices.add().zero();

      double globalTime = 0.0;
      for (int segmentNumber = 0; segmentNumber < indexHandler.getNumberOfSegments(); segmentNumber++)
      {
         for (int tick = 0; tick < indexHandler.getOrientationTicksInSegment(segmentNumber); tick++)
         {
            double tickDuration = indexHandler.getOrientationTickDuration(segmentNumber);
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

            DMatrixRMaj previousA = AMatrices.getLast();
            DMatrixRMaj previousB = BMatrices.getLast();
            DMatrixRMaj previousC = CMatrices.getLast();

            DMatrixRMaj nextA = AMatrices.add();
            DMatrixRMaj nextB = BMatrices.add();
            DMatrixRMaj nextC = CMatrices.add();

            nextB.reshape(6, indexHandler.getTotalProblemSize());

            CommonOps_DDRM.mult(dynamicsCalculator.getDiscreteAMatrix(), previousA, nextA);

            CommonOps_DDRM.mult(dynamicsCalculator.getDiscreteAMatrix(), previousB, nextB);
            CommonOps_DDRM.addEquals(nextB, dynamicsCalculator.getDiscreteBMatrix()); // FIXME this is an inaccurate block matrix

            CommonOps_DDRM.mult(dynamicsCalculator.getDiscreteAMatrix(), previousC, nextC);
            CommonOps_DDRM.addEquals(nextC, dynamicsCalculator.getDiscreteCMatrix());
         }
      }
   }
}
