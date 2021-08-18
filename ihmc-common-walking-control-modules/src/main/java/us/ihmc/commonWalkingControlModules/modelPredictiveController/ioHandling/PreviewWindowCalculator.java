package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
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

   private final YoDouble nominalInitialSegmentDuration = new YoDouble("nominalInitialSegmentDuration", registry);
   private final YoDouble nominalSegmentDuration = new YoDouble("nominalSegmentDuration", registry);
   private final YoDouble maximumPreviewWindowDuration = new YoDouble("maximumPreviewWindowDuration", registry);
   private final YoInteger maximumPreviewWindowSegments = new YoInteger("maximumPreviewWindowSegments", registry);
   private final YoInteger minimumPreviewWindowSegments = new YoInteger("minimumPreviewWindowSegments", registry);

   private final YoInteger segmentsInPreviewWindow = new YoInteger("segmentsInPreviewWindow", registry);
   private final YoDouble previewWindowDuration = new YoDouble("previewWindowDuration", registry);

   private final RecyclingArrayList<PreviewWindowSegment> previewWindowContacts = new RecyclingArrayList<>(PreviewWindowSegment::new);
   private final RecyclingArrayList<ContactPlaneProvider> fullContactSet = new RecyclingArrayList<>(ContactPlaneProvider::new);

   private final TDoubleArrayList contactGroupDurations = new TDoubleArrayList();
   private final TIntArrayList numberOfSegmentsInContactGroups = new TIntArrayList();
   private final RecyclingArrayList<TIntArrayList> contactPhasesInGroups = new RecyclingArrayList<>(TIntArrayList::new);

   private final ContactSegmentHelper contactSegmentHelper = new ContactSegmentHelper();
   private final List<ContactPlaneProvider> phasesInInterval = new ArrayList<>();

   public PreviewWindowCalculator(YoRegistry parentRegistry)
   {
      activeSegment.set(-1);
      this.nominalInitialSegmentDuration.set(0.125);
      this.nominalSegmentDuration.set(0.25);
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

   private final FramePoint3D modifiedStartPosition = new FramePoint3D();
   private final FramePoint3D modifiedEndPosition = new FramePoint3D();
   private final FrameVector3D modifiedStartVelocity = new FrameVector3D();
   private final FrameVector3D modifiedEndVelocity = new FrameVector3D();
   private final TDoubleArrayList segmentDurations = new TDoubleArrayList();

   private double computePlanningHorizon(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      int activeSegment = findTheActiveSegmentInTheContactSequence(fullContactSequence, timeAtStartOfWindow, true, false);
      activeSegmentChanged.set(false);

      if (this.activeSegment.getIntegerValue() != activeSegment)
      {
         activeSegmentChanged.set(true);
         this.activeSegment.set(activeSegment);
      }

      previewWindowContacts.clear();

      computeContactGroups(fullContactSequence, timeAtStartOfWindow, timeAtStartOfWindow + maximumPreviewWindowDuration.getDoubleValue(), activeSegment);

      double segmentStartTime = timeAtStartOfWindow;
      for (int contactGroupIdx = 0; contactGroupIdx < numberOfSegmentsInContactGroups.size(); contactGroupIdx++)
      {
         int segmentsInGroup = numberOfSegmentsInContactGroups.get(contactGroupIdx);

         segmentDurations.reset();
         if (segmentsInGroup > 2)
         {
            segmentDurations.add(nominalInitialSegmentDuration.getDoubleValue());
            segmentDurations.add(nominalInitialSegmentDuration.getDoubleValue());
            segmentDurations.add(contactGroupDurations.get(contactGroupIdx) - 2.0 * nominalInitialSegmentDuration.getDoubleValue());
         }
         else
         {
            double durationOfSegmentsInGroup = contactGroupDurations.get(contactGroupIdx) / segmentsInGroup;
            segmentDurations.add(durationOfSegmentsInGroup);
            segmentDurations.add(durationOfSegmentsInGroup);
         }

         TIntArrayList phasesInGroup = contactPhasesInGroups.get(contactGroupIdx);
         int startIdx = phasesInGroup.get(0);
         int endIndx = phasesInGroup.get(phasesInGroup.size() - 1);

         for (int segmentIdx = 0; segmentIdx < segmentsInGroup; segmentIdx++)
         {
            PreviewWindowSegment segment = previewWindowContacts.add();
            segment.reset();

            double segmentEndTime = segmentStartTime + segmentDurations.get(segmentIdx);
            startIdx = getPhasesSpanningInterval(segmentStartTime, segmentEndTime, startIdx, endIndx, fullContactSequence, phasesInInterval);
            for (int phaseIdx = 0; phaseIdx < phasesInInterval.size(); phaseIdx++)
            {
               ContactPlaneProvider contact = phasesInInterval.get(phaseIdx);
               segment.addContactPhaseInSegment(contact);
            }

            ContactStateBasics<?> startPhase = segment.getContactPhase(0);
            ContactStateBasics<?> endPhase = segment.getContactPhase(phasesInInterval.size() - 1);

            double alphaThroughStart = computeTheFractionThroughTheTimeInterval(segmentStartTime, phasesInInterval.get(0).getTimeInterval());
            double alphaThroughEnd = computeTheFractionThroughTheTimeInterval(segmentEndTime, phasesInInterval.get(phasesInInterval.size() - 1).getTimeInterval());
            double phaseStart = Math.max(segmentStartTime, startPhase.getTimeInterval().getStartTime());
            double phaseEnd = Math.min(segmentEndTime, endPhase.getTimeInterval().getEndTime());

            ContactSegmentHelper.cubicInterpolatePosition(modifiedStartPosition, startPhase, alphaThroughStart);
            ContactSegmentHelper.cubicInterpolateVelocity(modifiedStartVelocity, startPhase, alphaThroughStart);
            ContactSegmentHelper.cubicInterpolatePosition(modifiedEndPosition, endPhase, alphaThroughEnd);
            ContactSegmentHelper.cubicInterpolateVelocity(modifiedEndVelocity, endPhase, alphaThroughEnd);

            startPhase.setStartECMPPosition(modifiedStartPosition);
            startPhase.setStartECMPVelocity(modifiedStartVelocity);
            endPhase.setEndECMPPosition(modifiedEndPosition);
            endPhase.setEndECMPVelocity(modifiedEndVelocity);

            startPhase.getTimeInterval().setStartTime(phaseStart);
            endPhase.getTimeInterval().setEndTime(phaseEnd);

            for (int contactId = 0; contactId < phasesInInterval.get(0).getNumberOfContactPlanes(); contactId++)
               segment.addContact(phasesInInterval.get(0).getContactPose(contactId), phasesInInterval.get(0).getContactsInBodyFrame(contactId));

            segmentStartTime = segmentEndTime;
         }
      }

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

      PreviewWindowSegment lastSegment = previewWindowContacts.getLast();
      previewWindowLength += lastSegment.getDuration();

      if (!checkContactSequenceIsValid(previewWindowContacts, false))
         throw new IllegalArgumentException("The preview window is not valid.");

      populateTheFullContactSet(fullContactSequence);

      if (!ContactStateProviderTools.checkContactSequenceIsValid(fullContactSet))
         throw new IllegalArgumentException("The full contact sequence is not valid.");

      return previewWindowLength + flightDuration;
   }

   private void computeContactGroups(List<ContactPlaneProvider> fullContactSequence, double currentTime, double endTime, int activeSegment)
   {
      computePhasesInContactGroups(fullContactSequence, currentTime, endTime, activeSegment);

      numberOfSegmentsInContactGroups.reset();
      contactGroupDurations.reset();

      double relativeStartTime = currentTime - fullContactSequence.get(activeSegment).getTimeInterval().getStartTime();
      double timeRemaining = endTime - currentTime;
      int totalNumberOfSegments = 0;
      for (int groupIdx = 0; groupIdx < contactPhasesInGroups.size(); groupIdx++)
      {
         TIntArrayList phasesInGroup = contactPhasesInGroups.get(groupIdx);

         double groupDuration = 0.0;
         if (groupIdx == 0)
            groupDuration = -relativeStartTime;

         for (int phaseIdx = 0; phaseIdx < phasesInGroup.size(); phaseIdx++)
         {
            groupDuration += fullContactSequence.get(phasesInGroup.get(phaseIdx)).getTimeInterval().getDuration();
         }

         if (fullContactSequence.get(phasesInGroup.get(0)).getContactState().isLoadBearing())
         {
            groupDuration = Math.min(groupDuration, timeRemaining);
            timeRemaining -= groupDuration;
         }
         contactGroupDurations.add(groupDuration);
         double segmentDuration = totalNumberOfSegments > 1 ? nominalSegmentDuration.getDoubleValue() : nominalInitialSegmentDuration.getDoubleValue();
         int numberOfSegments = computeNumberOfSegmentsInGroup(fullContactSequence.get(groupIdx).getContactState(), groupDuration, segmentDuration);
         numberOfSegments = Math.min(numberOfSegments, 3);
         totalNumberOfSegments += numberOfSegments;
         numberOfSegmentsInContactGroups.add(numberOfSegments);
      }
   }

   private void computePhasesInContactGroups(List<ContactPlaneProvider> fullContactSequence, double currentTime, double endTime, int activeSegment)
   {
      contactPhasesInGroups.clear();

      TIntArrayList contactPhasesInGroup = contactPhasesInGroups.add();
      contactPhasesInGroup.reset();
      contactPhasesInGroup.add(activeSegment);

      ContactPlaneProvider startContact = fullContactSequence.get(activeSegment);

      double maxDuration = endTime - currentTime;
      double relativeStartTime = currentTime - startContact.getTimeInterval().getStartTime();
      double timeRemaining = maxDuration - startContact.getTimeInterval().getDuration() + relativeStartTime;

      int segmentIdx = activeSegment + 1;
      while (timeRemaining > 0.0 && segmentIdx < fullContactSequence.size())
      {
         ContactPlaneProvider previousContact = fullContactSequence.get(segmentIdx - 1);
         ContactPlaneProvider currentContact = fullContactSequence.get(segmentIdx);

         if (!doContactPhasesBelongToTheSameGroup(previousContact, currentContact))
         {
            contactPhasesInGroup = contactPhasesInGroups.add();
            contactPhasesInGroup.reset();
         }

         contactPhasesInGroup.add(segmentIdx);

         double currentDuration = Math.min(currentContact.getTimeInterval().getDuration(), 10.0);

         if (currentContact.getContactState().isLoadBearing())
         {
            if (timeRemaining - currentDuration < 0.0)
            {
               break;
            }
            else
            {
               timeRemaining -= currentDuration;
            }
         }

         segmentIdx++;
      }

   }

   private int computeNumberOfSegmentsInGroup(ContactState contactState, double groupDuration, double segmentDuration)
   {
      if (contactState == ContactState.FLIGHT)
         return 1;

      return Math.max((int) Math.round(groupDuration / nominalInitialSegmentDuration.getDoubleValue()), 1);
   }

   private static boolean doContactPhasesBelongToTheSameGroup(ContactPlaneProvider phaseA, ContactPlaneProvider phaseB)
   {
      if (phaseA.getContactState() != phaseB.getContactState())
         return false;

      if (phaseA.getNumberOfContactPlanes() != phaseB.getNumberOfContactPlanes())
         return false;

      for (int i = 0; i < phaseA.getNumberOfContactPlanes(); i++)
      {
         if (!phaseA.getContactPose(i).epsilonEquals(phaseB.getContactPose(i), 1e-3))
            return false;
      }

      return true;
   }

   private void populateTheFullContactSet(List<ContactPlaneProvider> fullContactSequence)
   {
      fullContactSet.clear();
      for (int i = 0; i < previewWindowContacts.size(); i++)
         setPlaneProviderFromPreviewWindowSegment(fullContactSet.add(), previewWindowContacts.get(i));

      double previewWindowEndTime = previewWindowContacts.getLast().getEndTime();
      if (previewWindowEndTime < fullContactSequence.get(fullContactSequence.size() - 1).getTimeInterval().getEndTime())
      {
         int activeSegmentIdx = findTheActiveSegmentInTheContactSequence(fullContactSequence, previewWindowEndTime, true, false);

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

   private static int getPhasesSpanningInterval(double startTime, double endTime, int startOfInterval, int endOfInterval,
                                                 List<ContactPlaneProvider> fullContactSequenece, List<ContactPlaneProvider> segmentsToPack)
   {
      segmentsToPack.clear();
      int startSegment = findTheActiveSegmentInTheContactSequence(fullContactSequenece, startTime, startOfInterval, endOfInterval, true, false);
      int endSegment = findTheActiveSegmentInTheContactSequence(fullContactSequenece, endTime, startSegment, endOfInterval, false, true);

      for (int i = startSegment; i <= endSegment; i++)
         segmentsToPack.add(fullContactSequenece.get(i));

      return endSegment;
   }

   private static int findTheActiveSegmentInTheContactSequence(List<ContactPlaneProvider> fullContactSequence,
                                                               double time,
                                                               boolean beginningInclusive,
                                                               boolean endInclusive)
   {
      return findTheActiveSegmentInTheContactSequence(fullContactSequence, time, 0, beginningInclusive, endInclusive);
   }

   private static final double timeEpsilon = 1e-6;

   private static int findTheActiveSegmentInTheContactSequence(List<ContactPlaneProvider> fullContactSequence,
                                                               double time,
                                                               int startSegment,
                                                               boolean beginningInclusive,
                                                               boolean endInclusive)
   {
      return findTheActiveSegmentInTheContactSequence(fullContactSequence, time, startSegment, fullContactSequence.size() - 1, beginningInclusive, endInclusive);
   }

   private static int findTheActiveSegmentInTheContactSequence(List<ContactPlaneProvider> fullContactSequence,
                                                               double time,
                                                               int startSegment,
                                                               int endSegment,
                                                               boolean beginningInclusive,
                                                               boolean endInclusive)
   {
      int activeSegment = -1;
      int max = endSegment + 1;
      for (int i = startSegment; i < max; i++)
      {
         TimeIntervalReadOnly timeInterval = fullContactSequence.get(i).getTimeInterval();
         boolean segmentIsValid = timeInterval.epsilonContains(time, timeEpsilon);
         boolean notAtEndOfSegment = i >= fullContactSequence.size() - 1 || (time + timeEpsilon < timeInterval.getEndTime() || endInclusive);
         if (segmentIsValid && notAtEndOfSegment && (beginningInclusive || time > timeInterval.getStartTime() - timeEpsilon))
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
      return checkContactSequenceIsValid(contactStateSequence, true);
   }

   public static boolean checkContactSequenceIsValid(List<PreviewWindowSegment> contactStateSequence, boolean checkForFlight)
   {
      if (checkForFlight && !checkContactSequenceDoesNotEndInFlight(contactStateSequence))
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
