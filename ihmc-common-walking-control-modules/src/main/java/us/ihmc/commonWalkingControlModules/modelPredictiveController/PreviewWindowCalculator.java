package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class PreviewWindowCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final CoMTrajectoryPlanner initializationCalculator;
   private final YoFramePoint3D dcmAtEndOfWindow = new YoFramePoint3D("dcmAtEndOfWindow", worldFrame, registry);

   private final YoInteger activeSegment = new YoInteger("activeSegmentInWindow", registry);
   private final YoBoolean activeSegmentChanged = new YoBoolean("activeSegmentChanged", registry);

   private final YoDouble maximumPreviewWindowDuration = new YoDouble("maximumPreviewWindowDuration", registry);
   private final YoInteger maximumPreviewWindowSegments = new YoInteger("maximumPreviewWindowSegments", registry);

   private final RecyclingArrayList<ContactPlaneProvider> previewWindowContacts = new RecyclingArrayList<>(ContactPlaneProvider::new);

   public PreviewWindowCalculator(double gravityZ, double nominalCoMHeight, YoRegistry parentRegistry)
   {
      initializationCalculator = new CoMTrajectoryPlanner(gravityZ, nominalCoMHeight, registry);

      activeSegment.set(-1);
      this.maximumPreviewWindowDuration.set(2.0);
      this.maximumPreviewWindowSegments.set(2);

      parentRegistry.addChild(registry);
   }

   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      initializationCalculator.setNominalCoMHeight(nominalCoMHeight);
   }

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      initializationCalculator.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void compute(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      initializationCalculator.solveForTrajectory(fullContactSequence);

      previewWindowContacts.clear();
      double previewWindowLength = computePlanningHorizon(fullContactSequence, timeAtStartOfWindow);

      initializationCalculator.compute(previewWindowLength + timeAtStartOfWindow);
      dcmAtEndOfWindow.set(initializationCalculator.getDesiredDCMPosition());
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
      cropFinalSegmentLength(previewWindowContacts.getLast());

      double previewWindowLength = 0.0;
      for (int i = 0; i < previewWindowContacts.size(); i++)
         previewWindowLength += previewWindowContacts.get(i).getTimeInterval().getDuration();

      return previewWindowLength;
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

   private void cropFinalSegmentLength(ContactPlaneProvider contact)
   {
      TimeIntervalBasics timeInterval = contact.getTimeInterval();
      if (maximumPreviewWindowDuration.getDoubleValue() > timeInterval.getDuration())
         return;

      double segmentDuration = Math.min(timeInterval.getDuration(), 10.0);
      double alphaIntoSegment = maximumPreviewWindowDuration.getDoubleValue() / segmentDuration;
      modifiedCoPLocation.interpolate(contact.getCopStartPosition(), contact.getCopEndPosition(), alphaIntoSegment);

      timeInterval.setEndTime(maximumPreviewWindowDuration.getDoubleValue() + timeInterval.getStartTime());
      contact.setEndCopPosition(modifiedCoPLocation);
   }

   public List<ContactPlaneProvider> getPlanningWindow()
   {
      return previewWindowContacts;
   }

   public FramePoint3DReadOnly getDCMAtEndOfWindow()
   {
      return dcmAtEndOfWindow;
   }

   public boolean activeSegmentChanged()
   {
      return activeSegmentChanged.getBooleanValue();
   }
}
