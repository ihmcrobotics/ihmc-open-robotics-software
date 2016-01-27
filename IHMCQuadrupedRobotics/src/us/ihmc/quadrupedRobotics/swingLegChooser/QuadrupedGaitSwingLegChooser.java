package us.ihmc.quadrupedRobotics.swingLegChooser;

import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedGaitSwingLegChooser implements NextSwingLegChooser
{
   private final YoVariableRegistry registry = new YoVariableRegistry("QuadrupedGaitSwingLegChooser");
   private final YoFrameVector lastVelocity;
   private final QuadrantDependentList<TranslationReferenceFrame> feetFrames = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePoint> feet = new QuadrantDependentList<>();
   private final FramePoint feetCentroid = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePose feetCentroidPose = new FramePose(ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame centroidFrame = new PoseReferenceFrame("", feetCentroidPose);
   private final YoGraphicReferenceFrame centroidFrameViz;
   
   public QuadrupedGaitSwingLegChooser(CommonQuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      lastVelocity = new YoFrameVector("lastVelocity", referenceFrames.getBodyFrame(), registry);
      centroidFrameViz = new YoGraphicReferenceFrame(centroidFrame, registry, 0.5);
      yoGraphicsListRegistry.registerYoGraphic("centroidFrameViz", centroidFrameViz);
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
  
   public RobotQuadrant chooseNextSwingLegUsingDistanceFromCentroid(QuadrupedSupportPolygon supportPolygon, RobotQuadrant lastStepQuadrant, FrameVector desiredVelocity, double desiredYawRate)
   {
      RobotQuadrant nextSwingLeg = null;
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint foot = new FramePoint(ReferenceFrame.getWorldFrame());
         foot.setIncludingFrame(supportPolygon.getFootstep(quadrant));
         feet.set(quadrant,  foot);
         
         String prefix = quadrant.getCamelCaseNameForStartOfExpression();
         TranslationReferenceFrame footFrame = new TranslationReferenceFrame(prefix + "FootFrame", ReferenceFrame.getWorldFrame());
         footFrame.updateTranslation(foot);
         feetFrames.set(quadrant, footFrame);
      }
      
      double yaw = supportPolygon.getNominalYaw();
      if(desiredVelocity.getX() < 0)
      {
         yaw += Math.PI;
      }
      yaw -= desiredYawRate;
      feetCentroidPose.setOrientation(yaw, 0.0, 0.0);
      
      supportPolygon.getCentroid2d(feetCentroid);
      feetCentroidPose.setPosition(feetCentroid);
      centroidFrame.setPoseAndUpdate(feetCentroidPose);
      centroidFrameViz.update();
      
      RobotQuadrant forwardFoot = getForwardFoot();
      RobotQuadrant sameSideHindFoot = forwardFoot.getSameSideQuadrant();
      RobotQuadrant otherSideFrontFoot = forwardFoot.getAcrossBodyQuadrant();
      RobotQuadrant otherSideHindFoot = forwardFoot.getDiagonalOppositeQuadrant();
      
      TranslationReferenceFrame sameSideHindFootFrame = feetFrames.get(sameSideHindFoot);
      FramePoint otherSideHindFootFramePoint = feet.get(otherSideHindFoot);
      
      otherSideHindFootFramePoint.changeFrame(sameSideHindFootFrame);
      
      if(otherSideHindFootFramePoint.getX() < 0.0)
      {
         nextSwingLeg = otherSideHindFoot;
      }
      else
      {
         nextSwingLeg = otherSideFrontFoot;
      }
      
      
//      if(lastStepQuadrant.equals(nextSwingLeg) && (lastVelocity.getX() > 0 && desiredVelocity.getX() > 0 || lastVelocity.getX() < 0 && desiredVelocity.getX() < 0))
//      {
//         if(desiredVelocity.getX() > 0)
//         {
//            nextSwingLeg = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
//         }
//         
//         if(desiredVelocity.getX() < 0)
//         {
//            nextSwingLeg = lastStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
//         }
//      }
      lastVelocity.setAndMatchFrame(desiredVelocity);
      
      return nextSwingLeg;
   }
   

   private RobotQuadrant getForwardFoot()
   {
      double maxX = Double.MIN_VALUE;
      RobotQuadrant forwardFoot = null;
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint foot = feet.get(quadrant);
         foot.changeFrame(centroidFrame);
         if(foot.getX() > maxX)
         {
            maxX = foot.getX();
            forwardFoot = quadrant;
         }
      }
      return forwardFoot;
   }
   

}
