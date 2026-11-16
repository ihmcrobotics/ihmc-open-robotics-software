package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.robotics.SCS2YoGraphicHolder;

public interface PelvisPoseHistoryCorrectionInterface extends SCS2YoGraphicHolder
{
   /**
    * Converges the state estimator pelvis pose towards an external position provided by an external Pelvis Pose Subscriber
    */
   void doControl(long timestamp);

   void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber);

   /**
    * Whether this correction is switched on. Backed by a yo-variable, so it can be changed at runtime. A
    * {@link PelvisPoseCorrectionSeries} runs only the corrections that are active.
    */
   boolean isActive();

   void setActive(boolean active);
}
