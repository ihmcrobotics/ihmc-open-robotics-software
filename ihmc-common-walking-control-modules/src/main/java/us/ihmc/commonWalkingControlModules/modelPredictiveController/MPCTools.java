package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowSegment;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.minDuration;

public class MPCTools
{
   public static void computeVRPWaypoints(double nominalCoMHeight,
                                          double gravityZ,
                                          double omega,
                                          FrameVector3DReadOnly currentCoMVelocity,
                                          List<PreviewWindowSegment> contactSequence,
                                          RecyclingArrayList<RecyclingArrayList<FramePoint3D>> startVRPPositionsToPack,
                                          RecyclingArrayList<RecyclingArrayList<FramePoint3D>> endVRPPositionsToPack,
                                          boolean adjustWaypointHeightForHeightChange)
   {
      startVRPPositionsToPack.clear();
      endVRPPositionsToPack.clear();

      double initialHeightVelocity = currentCoMVelocity.getZ();
      double finalHeightVelocity;

      for (int segmentId = 0; segmentId < contactSequence.size() - 1; segmentId++)
      {
         PreviewWindowSegment contactStateProvider = contactSequence.get(segmentId);
         boolean finalContact = segmentId == contactSequence.size() - 1;
         PreviewWindowSegment nextContactStateProvider = null;
         if (!finalContact)
            nextContactStateProvider = contactSequence.get(segmentId + 1);

         double offset = 0.0;
         if (adjustWaypointHeightForHeightChange)
         {
            double duration = contactStateProvider.getDuration();
            duration = Math.signum(duration) * Math.max(Math.abs(duration), minDuration);
            if (!contactStateProvider.getContactState().isLoadBearing())
            {
               finalHeightVelocity = initialHeightVelocity - gravityZ * duration;
            }
            else
            {
               if (!finalContact && !nextContactStateProvider.getContactState().isLoadBearing())
               { // next is a jump, current one is load bearing
                  PreviewWindowSegment nextNextContactStateProvider = contactSequence.get(segmentId + 2);
                  double heightBeforeJump = contactStateProvider.getContactPhase(contactStateProvider.getNumberOfContactPhasesInSegment() - 1).getECMPEndPosition().getZ();
                  double finalHeightAfterJump = nextNextContactStateProvider.getContactPhase(0).getECMPStartPosition().getZ();

                  double heightChangeWhenJumping = finalHeightAfterJump - heightBeforeJump;
                  double durationOfJump = nextContactStateProvider.getDuration();

                  /* delta z = v0 T - 0.5 g T^2
                   * v0 =  delta z / T + 0.5 g T**/
                  finalHeightVelocity = heightChangeWhenJumping / durationOfJump + 0.5 * gravityZ * durationOfJump;
               }
               else
               { // next is is load bearing, current is load bearing.
                  finalHeightVelocity = 0.0;
               }

               initialHeightVelocity = finalHeightVelocity;
            }

            // offset the height VRP waypoint based on the desired velocity change
            double heightVelocityChange = finalHeightVelocity - initialHeightVelocity;
            offset = heightVelocityChange / (MathTools.square(omega) * duration);
         }
         RecyclingArrayList<FramePoint3D> starts = startVRPPositionsToPack.add();
         RecyclingArrayList<FramePoint3D> ends = endVRPPositionsToPack.add();
         starts.clear();
         ends.clear();

         for (int phaseId = 0; phaseId < contactStateProvider.getNumberOfContactPhasesInSegment(); phaseId++)
         {
            FramePoint3D start = starts.add();
            FramePoint3D end = ends.add();

            start.set(contactStateProvider.getContactPhase(phaseId).getECMPStartPosition());
            start.addZ(nominalCoMHeight);
            end.set(contactStateProvider.getContactPhase(phaseId).getECMPEndPosition());
            end.addZ(nominalCoMHeight);

            start.subZ(offset);
            end.subZ(offset);

         }
      }

      PreviewWindowSegment contactStateProvider = contactSequence.get(contactSequence.size() - 1);
      RecyclingArrayList<FramePoint3D> starts = startVRPPositionsToPack.add();
      RecyclingArrayList<FramePoint3D> ends = endVRPPositionsToPack.add();

      for (int phaseId = 0; phaseId < contactStateProvider.getNumberOfContactPhasesInSegment(); phaseId++)
      {
         FramePoint3D start = starts.add();
         FramePoint3D end = ends.add();

         start.set(contactStateProvider.getContactPhase(phaseId).getECMPStartPosition());
         start.addZ(nominalCoMHeight);
         end.set(contactStateProvider.getContactPhase(phaseId).getECMPEndPosition());
         end.addZ(nominalCoMHeight);
      }
   }

   public static void computeVRPVelocites(List<PreviewWindowSegment> contactSequence,
                                          RecyclingArrayList<RecyclingArrayList<FrameVector3D>> startVRPVelocitiesToPack,
                                          RecyclingArrayList<RecyclingArrayList<FrameVector3D>> endVRPVelocitiesToPack)
   {
      startVRPVelocitiesToPack.clear();
      endVRPVelocitiesToPack.clear();

      for (int segmentId = 0; segmentId < contactSequence.size(); segmentId++)
      {
         PreviewWindowSegment contactStateProvider = contactSequence.get(segmentId);

         RecyclingArrayList<FrameVector3D> starts = startVRPVelocitiesToPack.add();
         RecyclingArrayList<FrameVector3D> ends = endVRPVelocitiesToPack.add();
         starts.clear();
         ends.clear();

         for (int phaseId = 0; phaseId < contactStateProvider.getNumberOfContactPhasesInSegment(); phaseId++)
         {
            FrameVector3D start = starts.add();
            FrameVector3D end = ends.add();

            start.set(contactStateProvider.getContactPhase(phaseId).getECMPStartVelocity());
            end.set(contactStateProvider.getContactPhase(phaseId).getECMPEndVelocity());
         }
      }
   }
}
