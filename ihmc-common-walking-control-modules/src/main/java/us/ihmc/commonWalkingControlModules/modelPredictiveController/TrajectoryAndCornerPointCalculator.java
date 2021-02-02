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

   public void setViewer(SegmentPointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void updateCornerPoints(LinearMPCTrajectoryHandler trajectoryHandler,
                                  List<? extends ContactStateProvider> contactSequence,
                                  int maxCapacity,
                                  double omega)
   {
      comSegments.clear();
      dcmSegments.clear();
      vrpSegments.clear();

      int segmentId = 0;
      for (; segmentId < Math.min(contactSequence.size(), maxCapacity + 1); segmentId++)
      {
         double startTime = contactSequence.get(segmentId).getTimeInterval().getStartTime();
         double endTime = Math.min(contactSequence.get(segmentId).getTimeInterval().getEndTime(), startTime + sufficientlyLongTime);

         startTime += 1e-6;
         endTime -= 1e-6;

         trajectoryHandler.compute(startTime);
         comStartPosition.set(trajectoryHandler.getDesiredCoMPosition());
         dcmStartPosition.set(trajectoryHandler.getDesiredDCMPosition());
         vrpStartPosition.set(trajectoryHandler.getDesiredVRPPosition());
         vrpStartVelocity.set(trajectoryHandler.getDesiredVRPVelocity());

         trajectoryHandler.compute(endTime);
         comEndPosition.set(trajectoryHandler.getDesiredCoMPosition());
         dcmEndPosition.set(trajectoryHandler.getDesiredDCMPosition());
         vrpEndPosition.set(trajectoryHandler.getDesiredVRPPosition());
         vrpEndVelocity.set(trajectoryHandler.getDesiredVRPVelocity());

         comSegments.add().set(comStartPosition, comEndPosition);
         dcmSegments.add().set(dcmStartPosition, dcmEndPosition);
         vrpSegments.add().set(vrpStartPosition, vrpEndPosition);
      }

      if (viewer != null)
      {
         viewer.updateDCMCornerPoints(dcmSegments);
         viewer.updateCoMCornerPoints(comSegments);
         viewer.updateVRPWaypoints(vrpSegments);
      }
   }
}
