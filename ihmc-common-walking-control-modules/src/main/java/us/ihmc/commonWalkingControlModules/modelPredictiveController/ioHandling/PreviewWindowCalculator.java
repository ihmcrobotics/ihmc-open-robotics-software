package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

/**
 * Class meant to calculate the preview window over which the model predictive controller operates. It is very unlikely that the MPC will calculate over the
 * full time horizon of steps. That means that some subset over a shorter horizon is needed to be computed for that. This class is used to compute what the
 * contact sequence is over that preview window.
 */
public class PreviewWindowCalculator
{
   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger activeSegment = new YoInteger("activeSegmentInWindow", registry);
   private final YoBoolean activeSegmentChanged = new YoBoolean("activeSegmentChanged", registry);

   private final YoDouble nominalSegmentDuration = new YoDouble("nominalSegmentDuration", registry);
   private final YoDouble maximumPreviewWindowDuration = new YoDouble("maximumPreviewWindowDuration", registry);
   private final YoInteger maximumPreviewWindowSegments = new YoInteger("maximumPreviewWindowSegments", registry);
   private final YoInteger minimumPreviewWindowSegments = new YoInteger("minimumPreviewWindowSegments", registry);

   private final YoInteger segmentsInPreviewWindow = new YoInteger("segmentsInPreviewWindow", registry);
   private final YoDouble previewWindowDuration = new YoDouble("previewWindowDuration", registry);

   private final RecyclingArrayList<PreviewWindowSegment> previewWindowContacts = new RecyclingArrayList<>(PreviewWindowSegment::new);
   private final RecyclingArrayList<ContactPlaneProvider> fullContactSet = new RecyclingArrayList<>(ContactPlaneProvider::new);

   private final ContactSegmentHelper contactSegmentHelper = new ContactSegmentHelper();
   private final List<ContactPlaneProvider> phasesInInterval = new ArrayList<>();

   public PreviewWindowCalculator(YoRegistry parentRegistry)
   {
      activeSegment.set(-1);
      this.nominalSegmentDuration.set(0.15);
      this.maximumPreviewWindowDuration.set(0.75);
      this.maximumPreviewWindowSegments.set(5);
      this.minimumPreviewWindowSegments.set(2);

      parentRegistry.addChild(registry);
   }

   /**
    * Computes the preview window contacts from the full contact sequence.
    *
    * @param fullContactSequence entire contact sequence. It may start before {@param timeAtStartOfWindow}
    * @param timeAtStartOfWindow time at the start of the preview window
    */
   public void compute(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      previewWindowContacts.clear();
      double previewWindowLength = computePlanningHorizon(fullContactSequence, timeAtStartOfWindow);

      this.previewWindowDuration.set(previewWindowLength);
      segmentsInPreviewWindow.set(previewWindowContacts.size());
   }

   /**
    * Gets the total duration that the preview window consists of.
    * @return duration of preview window
    */
   public double getPreviewWindowDuration()
   {
      return previewWindowDuration.getDoubleValue();
   }

   private double computePlanningHorizon(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      int activeSegment = findTheActiveSegmentInTheContactSequence(fullContactSequence, timeAtStartOfWindow);
      activeSegmentChanged.set(false);

      if (this.activeSegment.getIntegerValue() != activeSegment)
      {
         activeSegmentChanged.set(true);
         this.activeSegment.set(activeSegment);
      }

      previewWindowContacts.clear();

      double timeUntilTakeOff = computeTimeUntilTakeOff(fullContactSequence, timeAtStartOfWindow);

      int segmentsInWindow = (int) Math.floor(Math.min(timeUntilTakeOff, maximumPreviewWindowDuration.getDoubleValue()) / nominalSegmentDuration.getDoubleValue());
      segmentsInWindow = Math.max(segmentsInWindow, minimumPreviewWindowSegments.getIntegerValue());
      double nominalSegmentDuration = timeUntilTakeOff / segmentsInWindow;

      double startTime = timeAtStartOfWindow;
      for (int segmentIndex = activeSegment; segmentIndex < segmentsInWindow; segmentIndex++)
      {
         PreviewWindowSegment segment = previewWindowContacts.add();
         segment.reset();

         double endTime = startTime + nominalSegmentDuration;
         getPhasesSpanningInterval(startTime, endTime, fullContactSequence, phasesInInterval);
         for (int phaseIndex = 0; phaseIndex < phasesInInterval.size(); phaseIndex++)
         {
            ContactPlaneProvider contact = phasesInInterval.get(phaseIndex);
            segment.addContactPhaseInSegment(contact, contact.getTimeInterval().getStartTime(), contact.getTimeInterval().getEndTime());
         }

         double alphaThroughStart = computeTheFractionThroughTheTimeInterval(startTime, phasesInInterval.get(0).getTimeInterval());
         double alphaThroughEnd = computeTheFractionThroughTheTimeInterval(endTime, phasesInInterval.get(phasesInInterval.size() - 1).getTimeInterval());
         contactSegmentHelper.cubicInterpolateStartOfSegment(segment.getContactPhase(0), alphaThroughStart);
         contactSegmentHelper.cubicInterpolateEndOfSegment(segment.getContactPhase(phasesInInterval.size() - 1), alphaThroughEnd);

         for (int contactId = 0; contactId < phasesInInterval.get(0).getNumberOfContactPlanes(); contactId++)
            segment.addContact(phasesInInterval.get(0).getContactPose(contactId), phasesInInterval.get(0).getContactsInBodyFrame(contactId));
      }

      double startAlpha = (timeAtStartOfWindow - fullContactSequence.get(activeSegment).getTimeInterval().getStartTime()) / fullContactSequence.get(activeSegment).getTimeInterval().getDuration();
      contactSegmentHelper.cubicInterpolateStartOfSegment(previewWindowContacts.get(0).getContactPhase(0), startAlpha);
      previewWindowContacts.get(0).setStartTime(timeAtStartOfWindow);

      double previewWindowLength = 0.0;
      double flightDuration = 0.0;
      for (int i = 0; i < previewWindowContacts.size() - 1; i++)
      {
         double duration = previewWindowContacts.get(i).getDuration();
         if (previewWindowContacts.get(i).getContactState().isLoadBearing())
            previewWindowLength += duration;
         else
            flightDuration += duration;
      }

      double desiredFinalSegmentDuration = maximumPreviewWindowDuration.getDoubleValue() - previewWindowLength;
      PreviewWindowSegment lastSegment = previewWindowContacts.getLast();
      ContactStateBasics<?> lastPhase = lastSegment.getContactPhase(lastSegment.getNumberOfContactPhasesInSegment() - 1);
      double lastPhaseDuration = Math.min(lastPhase.getTimeInterval().getDuration(), 10.0);
      double alpha = Math.min(desiredFinalSegmentDuration / lastPhaseDuration, 1.0);
      if (alpha < 1.0)
      {
         double startTime = lastPhase.getTimeInterval().getStartTime();
         double newEndTime = alpha * lastPhaseDuration + startTime;
         contactSegmentHelper.cubicInterpolateEndOfSegment(lastPhase, alpha);
         lastSegment.setEndTime(newEndTime);
      }
      previewWindowLength += lastSegment.getDuration();

      populateTheFullContactSet(fullContactSequence);

      if (!checkContactSequenceIsValid(previewWindowContacts))
         throw new IllegalArgumentException("The preview window is not valid.");
      if (!ContactStateProviderTools.checkContactSequenceIsValid(fullContactSet))
         throw new IllegalArgumentException("The full contact sequence is not valid.");

      return previewWindowLength + flightDuration;
   }

   private static double computeTimeUntilTakeOff(List<ContactPlaneProvider> fullContactSequence, double currentTime)
   {
      double totalWindow = fullContactSequence.get(0).getTimeInterval().getStartTime() - currentTime;
      for (int i = 0; i < fullContactSequence.size(); i++)
      {
         if (i > 0 && fullContactSequence.get(i).getContactState().isLoadBearing())
            return totalWindow;

         totalWindow += fullContactSequence.get(i).getTimeInterval().getDuration();
      }

      return totalWindow;
   }

   private void populateTheFullContactSet(List<ContactPlaneProvider> fullContactSequence)
   {
      fullContactSet.clear();
      for (int i = 0; i < previewWindowContacts.size(); i++)
         setPlaneProviderFromPreviewWindowSegment(fullContactSet.add(), previewWindowContacts.get(i));

      double previewWindowEndTime = previewWindowContacts.getLast().getEndTime();
      if (previewWindowEndTime < fullContactSequence.get(fullContactSequence.size() - 1).getTimeInterval().getEndTime())
      {
         int activeSegmentIdx = findTheActiveSegmentInTheContactSequence(fullContactSequence, previewWindowEndTime);

         ContactPlaneProvider activeSegment = fullContactSequence.get(activeSegmentIdx);
         ContactPlaneProvider trimmedFinalSegment = fullContactSet.add();
         trimmedFinalSegment.set(activeSegment);
         trimmedFinalSegment.setStartTime(previewWindowEndTime);
         double alpha = computeTheFractionThroughTheTimeInterval(previewWindowEndTime, activeSegment.getTimeInterval());
         contactSegmentHelper.cubicInterpolateStartOfSegment(trimmedFinalSegment, alpha);

         for (int i = activeSegmentIdx + 1; i < fullContactSequence.size(); i++)
            fullContactSet.add().set(fullContactSequence.get(i));
      }
   }

   private static void getPhasesSpanningInterval(double starTime, double endTime, List<ContactPlaneProvider> fullContactSequenece, List<ContactPlaneProvider> segmentsToPack)
   {
      segmentsToPack.clear();
      int startSegment = findTheActiveSegmentInTheContactSequence(fullContactSequenece, starTime);
      int endSegment = findTheActiveSegmentInTheContactSequence(fullContactSequenece, endTime, startSegment);

      for (int i = startSegment; i <= endSegment; i++)
         segmentsToPack.add(fullContactSequenece.get(i));
   }

   private static int findTheActiveSegmentInTheContactSequence(List<ContactPlaneProvider> fullContactSequence, double time)
   {
      return findTheActiveSegmentInTheContactSequence(fullContactSequence, time, 0);
   }

   private static int findTheActiveSegmentInTheContactSequence(List<ContactPlaneProvider> fullContactSequence, double time, int startSegment)
   {
      int activeSegment = -1;
      for (int i = 0; i < fullContactSequence.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = fullContactSequence.get(i).getTimeInterval();
         boolean segmentIsValid = timeInterval.intervalContains(time);
         boolean notAtEndOfSegment = i >= fullContactSequence.size() - 1 || time < timeInterval.getEndTime();
         if (segmentIsValid && notAtEndOfSegment)
         {
            activeSegment = i;
            break;
         }
      }

      return activeSegment;
   }

   private static double computeTheFractionThroughTheTimeInterval(double time, TimeIntervalReadOnly timeInterval)
   {
      return (time - timeInterval.getStartTime()) / Math.min(timeInterval.getDuration(), 10.0);
   }

   /**
    * Gets the contact sequence that composes the preview window
    * @return contacts in the preview window
    */
   public List<PreviewWindowSegment> getPlanningWindow()
   {
      return previewWindowContacts;
   }

   /**
    * Gets the full contact sequence. Note that this is likely different than the contact sequence passed in in {@link #computePlanningHorizon(List, double)},
    * as it the preview sequence likely includes only a partial contact. The contact set returned by this function includes breaking that contact sequence into
    * a portion that is included in the preview window, and a portion that is not.
    * @return all the contacts for the entire plan
    */
   public List<ContactPlaneProvider> getFullPlanningSequence()
   {
      return fullContactSet;
   }

   /**
    * Computes whether or not a segment in the {@link #getFullPlanningSequence()} was completed, and then excluded from {@link #getPlanningWindow()}.
    * @return if a segment was just completed
    */
   public boolean activeSegmentChanged()
   {
      return activeSegmentChanged.getBooleanValue();
   }

   public static boolean checkContactSequenceIsValid(List<PreviewWindowSegment> contactStateSequence)
   {
      if (!checkContactSequenceDoesNotEndInFlight(contactStateSequence))
         return false;

      return isTimeSequenceContinuous(contactStateSequence, 1e-2);
   }

   static boolean checkContactSequenceDoesNotEndInFlight(List<PreviewWindowSegment> contactStateSequence)
   {
      return contactStateSequence.get(contactStateSequence.size() - 1).getContactState().isLoadBearing();
   }

   public static boolean isTimeSequenceContinuous(List<PreviewWindowSegment> contactStateSequence, double epsilon)
   {
      for (int index = 0; index < contactStateSequence.size() - 1; index++)
      {
         if (!TimeIntervalTools.areTimeIntervalsConsecutive(contactStateSequence.get(index), contactStateSequence.get(index + 1), epsilon))
            return false;
      }

      return true;
   }

   private static void setPlaneProviderFromPreviewWindowSegment(ContactPlaneProvider planeProviderToPack, PreviewWindowSegment segment)
   {
      planeProviderToPack.reset();
      planeProviderToPack.getTimeInterval().set(segment);

      planeProviderToPack.setStartECMPPosition(segment.getContactPhase(0).getECMPStartPosition());
      planeProviderToPack.setStartECMPVelocity(segment.getContactPhase(0).getECMPStartVelocity());
      int lastId = segment.getNumberOfContactPhasesInSegment() - 1;
      planeProviderToPack.setEndECMPPosition(segment.getContactPhase(lastId).getECMPEndPosition());
      planeProviderToPack.setEndECMPVelocity(segment.getContactPhase(lastId).getECMPEndVelocity());

      for (int i = 0; i < segment.getNumberOfContactPlanes(); i++)
         planeProviderToPack.addContact(segment.getContactPose(i), segment.getContactsInBodyFrame(i));
   }
}
