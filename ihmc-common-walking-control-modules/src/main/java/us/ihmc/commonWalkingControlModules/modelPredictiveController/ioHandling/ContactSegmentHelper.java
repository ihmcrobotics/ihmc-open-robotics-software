package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateBasics;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

public class ContactSegmentHelper
{
   private final FramePoint3D modifiedCoPLocation = new FramePoint3D();
   private final FrameVector3D modifiedCoPVelocity = new FrameVector3D();
   private final ContactPlaneProvider splitSegmentRemaining = new ContactPlaneProvider();

   // TODO need to account for the velocity, as well.
   public void cropInitialSegmentLength(ContactPlaneProvider contact, double timeAtStartOfWindow)
   {
      TimeIntervalBasics timeInterval = contact.getTimeInterval();
      if (timeAtStartOfWindow > timeInterval.getEndTime())
         throw new IllegalArgumentException("Bad initial segment.");

      double segmentDuration = Math.min(timeInterval.getDuration(), 10.0);
      double alphaIntoSegment = (timeAtStartOfWindow - timeInterval.getStartTime()) / segmentDuration;
      modifiedCoPLocation.interpolate(contact.getECMPStartPosition(), contact.getECMPEndPosition(), alphaIntoSegment);

      timeInterval.setStartTime(timeAtStartOfWindow);
      contact.setStartECMPPosition(modifiedCoPLocation);
   }

   public void cropInitialSegmentLength(PreviewWindowSegment segment, double timeAtStartOfWindow)
   {
      if (timeAtStartOfWindow > segment.getEndTime())
         throw new IllegalArgumentException("Bad initial segment.");

      int phaseId = 0;
      while ( phaseId < segment.getNumberOfContactPhasesInSegment())
      {
         TimeIntervalReadOnly timeInterval = segment.getTimeInterval(phaseId);
         if (timeAtStartOfWindow > timeInterval.getEndTime())
         {
            segment.removeContactPhaseFromSegment(phaseId);
            continue;
         }

         double segmentDuration = Math.min(timeInterval.getDuration(), 10.0);
         double alphaIntoSegment = (timeAtStartOfWindow - timeInterval.getStartTime()) / segmentDuration;
         cubicInterpolateStartOfSegment(segment.getContactPhase(phaseId), alphaIntoSegment);
         break;
      }
      segment.setStartTime(timeAtStartOfWindow);
   }

   public ContactPlaneProvider trimFinalSegmentLength(ContactPlaneProvider contact, double timeAtStartOfSegment, double maxEndTime)
   {
      TimeIntervalBasics timeInterval = contact.getTimeInterval();
      double maxSegmentDuration = maxEndTime - timeAtStartOfSegment;
      if (maxSegmentDuration > timeInterval.getDuration())
         return null;

      double segmentDuration = Math.min(timeInterval.getDuration(), 10.0);
      double alphaIntoSegment = maxSegmentDuration / segmentDuration;
      modifiedCoPLocation.interpolate(contact.getECMPStartPosition(), contact.getECMPEndPosition(), alphaIntoSegment);

      splitSegmentRemaining.set(contact);

      double splitTime = maxSegmentDuration + timeInterval.getStartTime();

      contact.setEndTime(splitTime);
      contact.setEndECMPPosition(modifiedCoPLocation);

      splitSegmentRemaining.setStartTime(splitTime);
      splitSegmentRemaining.setStartECMPPosition(modifiedCoPLocation);

      return splitSegmentRemaining;
   }

   public void cubicInterpolateStartOfSegment(ContactStateBasics<?> contact, double alpha)
   {
      double originalDuration = contact.getTimeInterval().getDuration();
      cubicInterpolatePosition(modifiedCoPLocation, contact.getECMPStartPosition(), contact.getECMPStartVelocity(),
                               contact.getECMPEndPosition(), contact.getECMPEndVelocity(), alpha, originalDuration);
      cubicInterpolateVelocity(modifiedCoPVelocity, contact.getECMPStartPosition(), contact.getECMPStartVelocity(),
                               contact.getECMPEndPosition(), contact.getECMPEndVelocity(), alpha, originalDuration);
      contact.setStartECMPPosition(modifiedCoPLocation);
      contact.setStartECMPVelocity(modifiedCoPVelocity);
   }

   public void cubicInterpolateEndOfSegment(ContactStateBasics<?> contact, double alpha)
   {
      double originalDuration = contact.getTimeInterval().getDuration();
      cubicInterpolatePosition(modifiedCoPLocation, contact.getECMPStartPosition(), contact.getECMPStartVelocity(),
                               contact.getECMPEndPosition(), contact.getECMPEndVelocity(), alpha, originalDuration);
      cubicInterpolateVelocity(modifiedCoPVelocity, contact.getECMPStartPosition(), contact.getECMPStartVelocity(),
                               contact.getECMPEndPosition(), contact.getECMPEndVelocity(), alpha, originalDuration);
      contact.setEndECMPPosition(modifiedCoPLocation);
      contact.setEndECMPVelocity(modifiedCoPVelocity);
   }

   private static void cubicInterpolatePosition(FramePoint3DBasics positionToPack,
                                                FramePoint3DReadOnly startPosition,
                                                FrameVector3DReadOnly startVelocity,
                                                FramePoint3DReadOnly endPosition,
                                                FrameVector3DReadOnly endVelocity,
                                                double alpha,
                                                double originalDuration)
   {
      double a2 = alpha * alpha;
      double a3 = alpha * a2;

      positionToPack.setAndScale((a3 - a2) * originalDuration, endVelocity);
      positionToPack.scaleAdd((a3 - 2.0 * a2 + alpha) * originalDuration, startVelocity, positionToPack);
      positionToPack.scaleAdd(3.0 * a2 - 2.0 * a3, endPosition, positionToPack);
      positionToPack.scaleAdd(2.0 * a3 - 3.0 * a2 + 1, startPosition, positionToPack);
   }

   private static void cubicInterpolateVelocity(FrameVector3DBasics velocityToPack,
                                                FramePoint3DReadOnly startPosition,
                                                FrameVector3DReadOnly startVelocity,
                                                FramePoint3DReadOnly endPosition,
                                                FrameVector3DReadOnly endVelocity,
                                                double alpha,
                                                double originalDuration)
   {
      double a2 = alpha * alpha;

      velocityToPack.sub(startPosition, endPosition);
      velocityToPack.scale(6.0 / originalDuration * (a2 - alpha));
      velocityToPack.scaleAdd(3.0 * a2 - 2.0 * alpha, endVelocity, velocityToPack);
      velocityToPack.scaleAdd(3.0 * a2 - 4.0 * alpha + 1.0, startVelocity, velocityToPack);
   }
}
