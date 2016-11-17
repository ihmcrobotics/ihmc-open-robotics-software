package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;

public class LocateFiducialBehavior extends AbstractBehavior
{
   private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;
   
   public LocateFiducialBehavior(CommunicationBridgeInterface communicationBridge, FiducialDetectorBehaviorService fiducialDetectorBehaviorService)
   {
      this(communicationBridge, fiducialDetectorBehaviorService, 50);
   }

   public LocateFiducialBehavior(CommunicationBridgeInterface communicationBridge, FiducialDetectorBehaviorService fiducialDetectorBehaviorService, int targetFiducial)
   {
      super(communicationBridge);
      
      this.fiducialDetectorBehaviorService = fiducialDetectorBehaviorService;
      fiducialDetectorBehaviorService.setLocationEnabled(true);
      fiducialDetectorBehaviorService.setTargetIDToLocate(targetFiducial);
   }

   @Override
   public void doControl()
   {
      
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void initialize()
   {
      super.initialize();
      
      fiducialDetectorBehaviorService.initialize();

//      HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(3.0, new Quat4d(-2.0659514928934525E-6, -0.03904875558882198,
//                                                                                              1.6862782077278572E-6, 0.9992373064892308));
//      sendPacketToController(headTrajectoryMessage);
   }
   
   @Override
   public void pause()
   {
      super.pause();
      fiducialDetectorBehaviorService.pause();
   }

   @Override
   public void abort()
   {
      super.abort();
      fiducialDetectorBehaviorService.stop();
   }

   @Override
   public void resume()
   {
      super.resume();
      fiducialDetectorBehaviorService.resume();
   }

}
