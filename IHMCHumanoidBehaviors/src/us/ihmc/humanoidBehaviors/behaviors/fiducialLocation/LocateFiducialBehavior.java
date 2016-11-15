package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.robotics.math.frames.YoFramePose;

public class LocateFiducialBehavior extends AbstractBehavior
{
   private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;
   
   public LocateFiducialBehavior(CommunicationBridgeInterface communicationBridge, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(communicationBridge, yoGraphicsListRegistry, FollowFiducialBehavior.DEFAULT_FIDUCIAL_TO_FOLLOW);
   }

   public LocateFiducialBehavior(CommunicationBridgeInterface communicationBridge, YoGraphicsListRegistry yoGraphicsListRegistry, int targetFiducial)
   {
      super(communicationBridge);
      
      fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(this, yoGraphicsListRegistry);
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

   public void setTargetIDToLocate(int targetIDToLocate)
   {
      fiducialDetectorBehaviorService.setLocationEnabled(true);
      fiducialDetectorBehaviorService.setTargetIDToLocate(targetIDToLocate);
   }

   public YoFramePose getFiducialPoseWorldFrame()
   {
      return fiducialDetectorBehaviorService.getLocatedFiducialPoseWorldFrame();
   }
}
