package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;

public interface PelvisPoseHistoryCorrectionInterface
{

   /**
    * Converges the state estimator pelvis pose towards an external position provided by an external Pelvis Pose Subscriber
    * @param l 
    */
   public abstract void doControl(long timestamp);

   public abstract void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber);

}