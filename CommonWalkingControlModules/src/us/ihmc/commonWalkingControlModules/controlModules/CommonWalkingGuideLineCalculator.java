package us.ihmc.commonWalkingControlModules.controlModules;


import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.GuideLineCalculator;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class CommonWalkingGuideLineCalculator implements GuideLineCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("GuideLineCalculator");
   private final SideDependentList<FrameLineSegment2d> guideLines = new SideDependentList<FrameLineSegment2d>();

   private final SideDependentList<ReferenceFrame> footZUpFrames;
   private final ReferenceFrame bodyZUpFrame;

   private final DoubleYoVariable captureForward = new DoubleYoVariable("captureForward", registry);
   private final DoubleYoVariable captureForwardOfSweet = new DoubleYoVariable("captureForwardOfSweet", registry);
   
   private final BooleanYoVariable onFinalStep;

   private final double footLength, footBack;
   
   public CommonWalkingGuideLineCalculator(CommonWalkingReferenceFrames referenceFrames, double footLength, double footBack, BooleanYoVariable onFinalStep,
           YoVariableRegistry parentRegistry)
   {
      this.onFinalStep = onFinalStep;
      this.footLength = footLength;
      this.footBack = footBack;
      
      this.bodyZUpFrame = referenceFrames.getABodyAttachedZUpFrame();

      this.footZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      parentRegistry.addChild(registry);
   }
   
   public void setCaptureForward(double captureForward)
   {
      this.captureForward.set(captureForward);
   }
   
   public void setCaptureForwardOfSweet(double captureForwardOfSweet)
   {
      this.captureForwardOfSweet.set(captureForwardOfSweet);
   }

   public FrameLineSegment2d getGuideLine(RobotSide side)
   {
      return guideLines.get(side);
   }

   public void reset()
   {
      throw new UnsupportedOperationException();
   }

   public void update(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePointInSupportFootZUp,
                      FramePoint finalDesiredSwingTarget)
   {
      FramePoint2d supportFootSweetSpot = bipedSupportPolygons.getSweetSpotCopy(supportLeg);

      if (onFinalStep == null || !onFinalStep.getBooleanValue())
         supportFootSweetSpot.setX(supportFootSweetSpot.getX() + captureForwardOfSweet.getDoubleValue());

      FramePoint2d stepToLocation;

      if (finalDesiredSwingTarget == null)    // If the finalDesired isn't specified, then track where the leg is currently.
      {
         stepToLocation = new FramePoint2d(footZUpFrames.get(supportLeg.getOppositeSide()));
         stepToLocation = stepToLocation.changeFrameCopy(bodyZUpFrame);
         stepToLocation = stepToLocation.changeFrameCopy(footZUpFrames.get(supportLeg));
      }
      else
      {
         stepToLocation = new FramePoint2d(finalDesiredSwingTarget.getReferenceFrame(), finalDesiredSwingTarget.getX(), finalDesiredSwingTarget.getY());
      }

      if (onFinalStep == null || !onFinalStep.getBooleanValue())
      {
         stepToLocation.setX(stepToLocation.getX() + captureForward.getDoubleValue());
      }

      else
      {
         // On final step, guide line goes from support foot sweet spot to other foot centroid.
         stepToLocation.setX(stepToLocation.getX() + footLength / 2.0 - footBack);
      }

//    if  (supportLeg == RobotSide.LEFT)
//    {
//       stepToLocation = new FramePoint2d(supportFootSweetSpot.getReferenceFrame(), stepForward.val, -stepInside.val);
//       pointOnLine2d = new FramePoint2d(supportFootSweetSpot.getReferenceFrame(), stepForward.val + captureForward.val, -stepInside.val);
//    }
//    else
//    {
//       stepToLocation = new FramePoint2d(supportFootSweetSpot.getReferenceFrame(), stepForward.val, stepInside.val);
//       pointOnLine2d = new FramePoint2d(supportFootSweetSpot.getReferenceFrame(), stepForward.val + captureForward.val, stepInside.val); // 0.08 is because stepForward is where the ankle will land...
//    }

//    FrameVector2d forward = new FrameVector2d(supportFootSweetSpot.getReferenceFrame(), 1.0, 0.0);
//    FrameLine2d guideLine = new FrameLine2d(supportFootSweetSpot, forward);

//    stepToLocations.set(supportLeg, stepToLocation);


      FrameLineSegment2d guideLine = new FrameLineSegment2d(supportFootSweetSpot, stepToLocation.changeFrameCopy(supportFootSweetSpot.getReferenceFrame()));

      guideLines.set(supportLeg, guideLine);
   }

}
