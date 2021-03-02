package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.time.TimeIntervalBasics;

public class ContactSegmentHelper
{
   private final FramePoint3D modifiedCoPLocation = new FramePoint3D();
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
}
