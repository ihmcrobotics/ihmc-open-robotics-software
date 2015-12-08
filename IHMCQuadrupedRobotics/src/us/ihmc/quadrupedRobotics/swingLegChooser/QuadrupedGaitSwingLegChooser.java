package us.ihmc.quadrupedRobotics.swingLegChooser;

import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedGaitSwingLegChooser implements NextSwingLegChooser
{
   private final YoVariableRegistry registry = new YoVariableRegistry("QuadrupedGaitSwingLegChooser");
   private final YoFrameVector lastVelocity;
   
   public QuadrupedGaitSwingLegChooser(CommonQuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      lastVelocity = new YoFrameVector("lastVelocity", referenceFrames.getBodyFrame(), registry);
      parentRegistry.addChild(registry);
   }
   
   @Override
   public RobotQuadrant chooseNextSwingLeg(QuadrupedSupportPolygon supportPolygon, RobotQuadrant lastStepQuadrant, FrameVector desiredVelocity, double desiredYawRate)
   {
      RobotQuadrant nextSwingLeg = null;
      
      if(lastVelocity.getX() < 0 && desiredVelocity.getX() > 0 ||  lastVelocity.getX() > 0 && desiredVelocity.getX() < 0)
      {
         nextSwingLeg = lastStepQuadrant;
      }
      else if(desiredVelocity.getX() >= 0)
      {
         nextSwingLeg = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
      }
      else
      {
         nextSwingLeg = lastStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
      }
      lastVelocity.setAndMatchFrame(desiredVelocity);
      
      return nextSwingLeg;
   }

}
