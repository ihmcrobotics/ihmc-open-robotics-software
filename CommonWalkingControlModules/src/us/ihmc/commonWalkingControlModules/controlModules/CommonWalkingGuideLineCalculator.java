package us.ihmc.commonWalkingControlModules.controlModules;


import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.GuideLineCalculator;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class CommonWalkingGuideLineCalculator implements GuideLineCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("GuideLineCalculator");
   private final SideDependentList<FrameLineSegment2d> guideLines = new SideDependentList<FrameLineSegment2d>();

   private final SideDependentList<ReferenceFrame> footZUpFrames;

   private final DoubleYoVariable captureForward = new DoubleYoVariable("captureForward", registry);
   private final DoubleYoVariable captureForwardOfSweet = new DoubleYoVariable("captureForwardOfSweet", registry);

   private final DoubleYoVariable velocityGainX = new DoubleYoVariable("velocityGainX", registry);
   private final DoubleYoVariable velocityGainY = new DoubleYoVariable("velocityGainY", registry);


//   private final double footLength, footBack;

   public CommonWalkingGuideLineCalculator(CommonHumanoidReferenceFrames referenceFrames, BooleanYoVariable onFinalStep, YoVariableRegistry parentRegistry)
   {
//      this.footLength = footLength;
//      this.footBack = footBack;
      this.footZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      parentRegistry.addChild(registry);
   }

   public FrameLineSegment2d getGuideLine(RobotSide supportLeg)
   {
      return guideLines.get(supportLeg);
   }

   public void reset()
   {
      throw new UnsupportedOperationException();
   }

   public void update(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePointInSupportFootZUp,
                      FramePoint finalDesiredSwingTarget, FrameVector2d desiredVelocity)
   {
      if (finalDesiredSwingTarget == null)
         throw new RuntimeException("finalDesiredSwingTarget == null");
      
      FramePoint2d supportFootSweetSpot = bipedSupportPolygons.getSweetSpotCopy(supportLeg);
      supportFootSweetSpot.changeFrame(footZUpFrames.get(supportLeg));
      supportFootSweetSpot.setX(supportFootSweetSpot.getX() + captureForwardOfSweet.getDoubleValue());

      FramePoint2d stepToLocation = finalDesiredSwingTarget.toFramePoint2d();
      stepToLocation.changeFrame(footZUpFrames.get(supportLeg));
      stepToLocation.setX(stepToLocation.getX() + captureForward.getDoubleValue());
      FrameLineSegment2d guideLine = new FrameLineSegment2d(supportFootSweetSpot, stepToLocation);

      guideLines.set(supportLeg, guideLine);
   }

   public void setParametersForR2()
   {
      velocityGainX.set(0.25);
      velocityGainY.set(0.05);
      captureForward.set(0.10); // 20);    // (0.08);
      captureForwardOfSweet.set(0.0); // 0.03);
   }
   
   public void setParametersForM2V2PushRecovery()
   {
      velocityGainX.set(0.0);
      velocityGainY.set(0.0);
      captureForward.set(0.0); // 0.08);    // 0.04; //0.08;
      captureForwardOfSweet.set(0.03);
   }

   public void setParametersForM2V2Walking()
   {
      velocityGainX.set(0.0); // 0.2);
      velocityGainY.set(0.05);
      captureForward.set(0.04); // 0.08);    // 0.04; //0.08;
      captureForwardOfSweet.set(0.0);
   }
}
