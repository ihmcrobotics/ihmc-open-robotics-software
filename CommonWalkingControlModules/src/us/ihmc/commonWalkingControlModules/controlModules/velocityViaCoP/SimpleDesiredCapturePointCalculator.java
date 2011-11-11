package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointCalculator;
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

   public FramePoint2d computeDesiredCapturePointSingleSupport(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint)
   {
      return couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(supportLeg);
   }

   public FramePoint2d computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, BipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity)
   {
      FramePoint2d desiredCapturePoint;

      if (stayInDoubleSupport(loadingLeg))
      {
         desiredCapturePoint = new FramePoint2d(referenceFrames.getMidFeetZUpFrame());

         FrameVector2d leftForward = new FrameVector2d(referenceFrames.getAnkleZUpFrame(RobotSide.LEFT), 1.0, 0.0);
         FrameVector2d rightForward = new FrameVector2d(referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT), 1.0, 0.0);

         leftForward.changeFrame(desiredCapturePoint.getReferenceFrame());
         rightForward.changeFrame(desiredCapturePoint.getReferenceFrame());

         FrameVector2d offset = leftForward;
         offset.add(rightForward);
         offset.normalize();
         offset.scale(desiredCaptureForwardStayInDoubleSupport.getDoubleValue());
         desiredCapturePoint.add(offset);
      }
      else
      {
         double kxx = desiredCaptureKxx.getDoubleValue();
         double kxy = loadingLeg.negateIfLeftSide(desiredCaptureKxy.getDoubleValue());

         desiredCapturePoint = couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(loadingLeg);
         ReferenceFrame ankleZUpFrame = referenceFrames.getAnkleZUpFrame(loadingLeg);
         desiredCapturePoint.changeFrame(ankleZUpFrame);
         desiredVelocity = desiredVelocity.changeFrameCopy(ankleZUpFrame);

         desiredCapturePoint.setX(desiredCapturePoint.getX() + kxx * desiredVelocity.getX());
         desiredCapturePoint.setY(desiredCapturePoint.getY() + kxy * Math.abs(desiredVelocity.getX()));
      }

      return desiredCapturePoint;
   }

   public void setUpParametersForR2()
   {
      desiredCaptureForwardStayInDoubleSupport.set(0.05);
      desiredCaptureKxx.set(0.18);
      desiredCaptureKxy.set(0.05);
   }

   public void setUpParametersForM2V2()
   {
      desiredCaptureForwardStayInDoubleSupport.set(0.02);
      desiredCaptureKxx.set(0.0); //0.1);    // TODO: tune
      desiredCaptureKxy.set(0.0); //0.05);    // TODO: tune
   }
   
   private static boolean stayInDoubleSupport(RobotSide loadingLeg)
   {
      return loadingLeg == null;
   }
}
