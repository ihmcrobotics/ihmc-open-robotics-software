package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SimpleDesiredCapturePointCalculator implements DesiredCapturePointCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredCapturePointCalculator");
   private final CouplingRegistry couplingRegistry;
   private final CommonWalkingReferenceFrames referenceFrames;

   private final DoubleYoVariable desiredCaptureForwardStayInDoubleSupport = new DoubleYoVariable("desiredCaptureForwardNotLoading", registry);
   private final DoubleYoVariable desiredCaptureKxx = new DoubleYoVariable("desiredCaptureKxx", registry);
   private final DoubleYoVariable desiredCaptureKxy = new DoubleYoVariable("desiredCaptureKxy", registry);

   public SimpleDesiredCapturePointCalculator(CouplingRegistry couplingRegistry, CommonWalkingReferenceFrames referenceFrames,
           YoVariableRegistry parentRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.referenceFrames = referenceFrames;
      parentRegistry.addChild(registry);
   }

   public FramePoint2d computeDesiredCapturePointSingleSupport(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity, SingleSupportCondition singleSupportCondition)
   { 
//      return couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(supportLeg);      
      double kxx, kxy;

//      if (singleSupportCondition == SingleSupportCondition.Loading)
//      {
//         kxx = 0.0;
//         kxy = 0.0;
//      }
//      else
//      {
//         kxx = desiredCaptureKxx.getDoubleValue();
//         kxy = supportLeg.negateIfLeftSide(desiredCaptureKxy.getDoubleValue());
//      }
      kxx = desiredCaptureKxx.getDoubleValue();
      kxy = supportLeg.negateIfLeftSide(desiredCaptureKxy.getDoubleValue());

      FramePoint2d desiredCapturePoint = couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(supportLeg);
      ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(supportLeg);
      desiredCapturePoint.changeFrame(ankleZUpFrame);
      desiredVelocity = desiredVelocity.changeFrameCopy(ankleZUpFrame);

      desiredCapturePoint.setX(desiredCapturePoint.getX() + kxx * desiredVelocity.getX());
      desiredCapturePoint.setY(desiredCapturePoint.getY() + kxy * Math.abs(desiredVelocity.getX()));
      return desiredCapturePoint;
   }

   public FramePoint2d computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, BipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity)
   {
      if (stayInDoubleSupport(loadingLeg))
      {
         FramePoint2d desiredCapturePoint = new FramePoint2d(referenceFrames.getMidFeetZUpFrame());

         FrameVector2d leftForward = new FrameVector2d(referenceFrames.getAnkleZUpFrame(RobotSide.LEFT), 1.0, 0.0);
         FrameVector2d rightForward = new FrameVector2d(referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT), 1.0, 0.0);

         leftForward.changeFrame(desiredCapturePoint.getReferenceFrame());
         rightForward.changeFrame(desiredCapturePoint.getReferenceFrame());

         FrameVector2d offset = leftForward;
         offset.add(rightForward);
         offset.normalize();
         offset.scale(desiredCaptureForwardStayInDoubleSupport.getDoubleValue());
         desiredCapturePoint.add(offset);
         return desiredCapturePoint;
      }
      else
      {
         return computeDesiredCapturePointSingleSupport(loadingLeg, bipedSupportPolygons, desiredVelocity, SingleSupportCondition.Loading);
      }
   }

   public void setUpParametersForR2()
   {
      desiredCaptureForwardStayInDoubleSupport.set(0.05);
      desiredCaptureKxx.set(0.15);
      desiredCaptureKxy.set(0.05);
   }

   public void setUpParametersForM2V2()
   {
      desiredCaptureForwardStayInDoubleSupport.set(0.02);
      desiredCaptureKxx.set(0.03);
      desiredCaptureKxy.set(0.01);
   }
   
   private static boolean stayInDoubleSupport(RobotSide loadingLeg)
   {
      return loadingLeg == null;
   }
}
