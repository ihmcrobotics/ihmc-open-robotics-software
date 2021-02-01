package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLongTime;

public class TrajectoryAndCornerPointCalculator
{
   private SegmentPointViewer viewer = null;

   private final FramePoint3D vrpStartPosition = new FramePoint3D();
   private final FrameVector3D vrpStartVelocity = new FrameVector3D();
   private final FramePoint3D vrpEndPosition = new FramePoint3D();
   private final FrameVector3D vrpEndVelocity = new FrameVector3D();
   private final FramePoint3D dcmStartPosition = new FramePoint3D();
   private final FramePoint3D dcmEndPosition = new FramePoint3D();
   private final FramePoint3D comStartPosition = new FramePoint3D();
   private final FramePoint3D comEndPosition = new FramePoint3D();

   private final RecyclingArrayList<LineSegment3D> dcmSegments = new RecyclingArrayList<>(LineSegment3D::new);
   private final RecyclingArrayList<LineSegment3D> comSegments = new RecyclingArrayList<>(LineSegment3D::new);
   private final RecyclingArrayList<LineSegment3D> vrpSegments = new RecyclingArrayList<>(LineSegment3D::new);

   private final RecyclingArrayList<Polynomial3DBasics> vrpTrajectoryPool = new RecyclingArrayList<>(() -> new Polynomial3D(4));
   private final List<Polynomial3DBasics> vrpTrajectories = new ArrayList<>();

   public void setViewer(SegmentPointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void updateCornerPoints(CoMTrajectoryModelPredictiveController mpc,
                                  int previewWindowLength,
                                  List<? extends ContactStateProvider> contactSequence,
                                  int maxCapacity,
                                  double omega)
   {
      vrpTrajectoryPool.clear();
      vrpTrajectories.clear();

      comSegments.clear();
      dcmSegments.clear();
      vrpSegments.clear();

      int segmentId = 0;
      for (; segmentId < Math.min(previewWindowLength, maxCapacity + 1); segmentId++)
      {
         double duration = contactSequence.get(segmentId).getTimeInterval().getDuration();

         duration = Math.min(duration, sufficientlyLongTime);
         mpc.trajectoryHandler.computeInPlanningWindow(segmentId, 0.0, omega);
         comStartPosition.set(mpc.trajectoryHandler.getDesiredCoMPosition());
         dcmStartPosition.set(mpc.trajectoryHandler.getDesiredDCMPosition());
         vrpStartPosition.set(mpc.trajectoryHandler.getDesiredVRPPosition());
         vrpStartVelocity.set(mpc.trajectoryHandler.getDesiredVRPVelocity());

         mpc.trajectoryHandler.computeInPlanningWindow(segmentId, duration, omega);
         comEndPosition.set(mpc.trajectoryHandler.getDesiredCoMPosition());
         dcmEndPosition.set(mpc.trajectoryHandler.getDesiredDCMPosition());
         vrpEndPosition.set(mpc.trajectoryHandler.getDesiredVRPPosition());
         vrpEndVelocity.set(mpc.trajectoryHandler.getDesiredVRPVelocity());

         Polynomial3DBasics trajectory3D = vrpTrajectoryPool.add();
//         trajectory3D.setLinear(0.0, duration, vrpStartPosition,  vrpEndPosition);
         trajectory3D.setCubic(0.0, duration, vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
         vrpTrajectories.add(trajectory3D);

         comSegments.add().set(comStartPosition, comEndPosition);
         dcmSegments.add().set(dcmStartPosition, dcmEndPosition);
         vrpSegments.add().set(vrpStartPosition, vrpEndPosition);
      }

      for (; segmentId < Math.min(contactSequence.size(), maxCapacity + 1); segmentId++)
      {
         double startTime = contactSequence.get(segmentId).getTimeInterval().getStartTime();
         double duration = contactSequence.get(segmentId).getTimeInterval().getDuration();
         duration = Math.min(duration, sufficientlyLongTime);

         mpc.trajectoryHandler.compute(startTime, omega);
         comStartPosition.set(mpc.trajectoryHandler.getDesiredCoMPosition());
         dcmStartPosition.set(mpc.trajectoryHandler.getDesiredDCMPosition());
         vrpStartPosition.set(mpc.trajectoryHandler.getDesiredVRPPosition());
         vrpStartVelocity.set(mpc.trajectoryHandler.getDesiredVRPVelocity());

         mpc.trajectoryHandler.compute(startTime + duration, omega);
         comEndPosition.set(mpc.trajectoryHandler.getDesiredCoMPosition());
         dcmEndPosition.set(mpc.trajectoryHandler.getDesiredDCMPosition());
         vrpEndPosition.set(mpc.trajectoryHandler.getDesiredVRPPosition());
         vrpEndVelocity.set(mpc.trajectoryHandler.getDesiredVRPVelocity());

         Polynomial3DBasics trajectory3D = vrpTrajectoryPool.add();
//         trajectory3D.setLinear(0.0, duration, vrpStartPosition,  vrpEndPosition);
                  trajectory3D.setCubic(0.0, duration, vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
         vrpTrajectories.add(trajectory3D);

         comSegments.add().set(comStartPosition, comEndPosition);
         dcmSegments.add().set(dcmStartPosition, dcmEndPosition);
         vrpSegments.add().set(vrpStartPosition, vrpEndPosition);
      }

      // TODO the segment outside of the preview window

      if (viewer != null)
      {
         viewer.updateDCMCornerPoints(dcmSegments);
         viewer.updateCoMCornerPoints(comSegments);
         viewer.updateVRPWaypoints(vrpSegments);
      }
   }

   public List<? extends Polynomial3DReadOnly> getVrpTrajectories()
   {
      return vrpTrajectories;
   }
}
