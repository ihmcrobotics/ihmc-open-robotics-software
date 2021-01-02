package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProviderTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class PreviewWindowCalculator
{
   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger activeSegment = new YoInteger("activeSegmentInWindow", registry);
   private final YoBoolean activeSegmentChanged = new YoBoolean("activeSegmentChanged", registry);

   private final YoDouble maximumPreviewWindowDuration = new YoDouble("maximumPreviewWindowDuration", registry);
   private final YoInteger maximumPreviewWindowSegments = new YoInteger("maximumPreviewWindowSegments", registry);

   private final YoInteger segmentsInPreviewWindow = new YoInteger("segmentsInPreviewWindow", registry);
   private final YoDouble previewWindowDuration = new YoDouble("previewWindowDuration", registry);

   private final RecyclingArrayList<ContactPlaneProvider> previewWindowContacts = new RecyclingArrayList<>(ContactPlaneProvider::new);
   private final RecyclingArrayList<ContactPlaneProvider> fullContactSet = new RecyclingArrayList<>(ContactPlaneProvider::new);

   public PreviewWindowCalculator(YoRegistry parentRegistry)
   {
      activeSegment.set(-1);
      this.maximumPreviewWindowDuration.set(0.75);
      this.maximumPreviewWindowSegments.set(3);

      parentRegistry.addChild(registry);
   }

   public void compute(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      previewWindowContacts.clear();
      double previewWindowLength = computePlanningHorizon(fullContactSequence, timeAtStartOfWindow);

      this.previewWindowDuration.set(previewWindowLength);
      segmentsInPreviewWindow.set(previewWindowContacts.size());
   }

   public double getPreviewWindowDuration()
   {
      return previewWindowDuration.getDoubleValue();
   }

   private double computePlanningHorizon(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      int activeSegment = -1;
      activeSegmentChanged.set(false);
      for (int i = 0; i < fullContactSequence.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = fullContactSequence.get(i).getTimeInterval();
         if (timeInterval.intervalContains(timeAtStartOfWindow))
         {
            activeSegment = i;
            break;
         }
      }

      if (this.activeSegment.getIntegerValue() != activeSegment)
      {
         activeSegmentChanged.set(true);
         this.activeSegment.set(activeSegment);
      }

      previewWindowContacts.clear();

      double horizonDuration = -timeAtStartOfWindow;
      for (int i = activeSegment; i < fullContactSequence.size(); i++)
      {
         ContactPlaneProvider contact = fullContactSequence.get(i);

         previewWindowContacts.add().set(contact);
         horizonDuration += contact.getTimeInterval().getDuration();

         if (contact.getContactState().isLoadBearing() && (horizonDuration >= maximumPreviewWindowDuration.getDoubleValue()
                                                           || previewWindowContacts.size() > maximumPreviewWindowSegments.getValue() - 1))
            break;
      }

      cropInitialSegmentLength(previewWindowContacts.get(0), timeAtStartOfWindow);


      double previewWindowLength = 0.0;
      double flightDuration = 0.0;
      for (int i = 0; i < previewWindowContacts.size() - 1; i++)
      {
         double duration = previewWindowContacts.get(i).getTimeInterval().getDuration();
         if (previewWindowContacts.get(i).getContactState().isLoadBearing())
            previewWindowLength += duration;
         else
            flightDuration += duration;
      }

      ContactPlaneProvider trimmedFinalSegment = trimFinalSegmentLength(previewWindowContacts.getLast(), previewWindowLength);
      previewWindowLength += previewWindowContacts.getLast().getTimeInterval().getDuration();

      fullContactSet.clear();
      for (int i = 0; i < previewWindowContacts.size(); i++)
         fullContactSet.add().set(previewWindowContacts.get(i));

      if (trimmedFinalSegment != null)
      {
         fullContactSet.add().set(trimmedFinalSegment);
         for (int i = previewWindowContacts.size() + activeSegment; i < fullContactSequence.size(); i++)
            fullContactSet.add().set(fullContactSequence.get(i));
      }

      if (!ContactStateProviderTools.checkContactSequenceIsValid(previewWindowContacts))
         throw new IllegalArgumentException("The preview window is not valid.");
      if (!ContactStateProviderTools.checkContactSequenceIsValid(fullContactSet))
         throw new IllegalArgumentException("The full contact sequence is not valid.");

      return previewWindowLength + flightDuration;
   }

   private final FramePoint3D modifiedCoPLocation = new FramePoint3D();

   private void cropInitialSegmentLength(ContactPlaneProvider contact, double timeAtStartOfWindow)
   {
      TimeIntervalBasics timeInterval = contact.getTimeInterval();
      if (timeAtStartOfWindow > timeInterval.getEndTime())
         throw new IllegalArgumentException("Bad initial segment.");

      double segmentDuration = Math.min(timeInterval.getDuration(), 10.0);
      double alphaIntoSegment = (timeAtStartOfWindow - timeInterval.getStartTime()) / segmentDuration;
      modifiedCoPLocation.interpolate(contact.getCopStartPosition(), contact.getCopEndPosition(), alphaIntoSegment);

      timeInterval.setStartTime(timeAtStartOfWindow);
      contact.setStartCopPosition(modifiedCoPLocation);
   }

   private final ContactPlaneProvider splitSegmentRemaining = new ContactPlaneProvider();

   private ContactPlaneProvider trimFinalSegmentLength(ContactPlaneProvider contact, double timeAtStartOfSegment)
   {
      TimeIntervalBasics timeInterval = contact.getTimeInterval();
      double maxSegmentDuration = maximumPreviewWindowDuration.getDoubleValue() - timeAtStartOfSegment;
      if (maxSegmentDuration > timeInterval.getDuration())
         return null;

      double segmentDuration = Math.min(timeInterval.getDuration(), 10.0);
      double alphaIntoSegment = maxSegmentDuration / segmentDuration;
      modifiedCoPLocation.interpolate(contact.getCopStartPosition(), contact.getCopEndPosition(), alphaIntoSegment);

      splitSegmentRemaining.set(contact);

      double splitTime = maxSegmentDuration + timeInterval.getStartTime();

      contact.setEndTime(splitTime);
      contact.setEndCopPosition(modifiedCoPLocation);

      splitSegmentRemaining.setStartTime(splitTime);
      splitSegmentRemaining.setStartCopPosition(modifiedCoPLocation);

      return splitSegmentRemaining;
   }

   public List<ContactPlaneProvider> getPlanningWindow()
   {
      return previewWindowContacts;
   }

   public List<ContactPlaneProvider> getFullPlanningSequence()
   {
      return fullContactSet;
   }

   public boolean activeSegmentChanged()
   {
      return activeSegmentChanged.getBooleanValue();
   }
}
