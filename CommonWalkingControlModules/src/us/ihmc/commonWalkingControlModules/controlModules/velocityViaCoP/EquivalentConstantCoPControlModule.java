package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointToDesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.GuideLineToDesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;


public class EquivalentConstantCoPControlModule implements DesiredCapturePointToDesiredCoPControlModule, GuideLineToDesiredCoPControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("EquivalentConstantCoPVelocityViaCoPControlModule");

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;

   private final DoubleYoVariable minimumEqConstTimespanDoubleSupport = new DoubleYoVariable("minimumEqConstTimespanDoubleSupport", registry);
   private final DoubleYoVariable minimumEqConstTimespanSingleSupport = new DoubleYoVariable("minimumEqConstTimespanSingleSupport", registry);
   private final DoubleYoVariable guideLineParameter = new DoubleYoVariable("guideLinePercentage", registry);
   private final DoubleYoVariable guideLineKp = new DoubleYoVariable("guideLineKp", registry);


   public EquivalentConstantCoPControlModule(CommonHumanoidReferenceFrames referenceFrames, ProcessedSensorsInterface processedSensors,
           CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.processedSensors = processedSensors;
      this.couplingRegistry = couplingRegistry;
      parentRegistry.addChild(registry);
   }
   
   // using desired capture point
   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
         FrameVector2d desiredVelocity, FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity)
   {
      FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getFootPolygonInAnkleZUp(supportLeg);
      double finalTime = minimumEqConstTimespanSingleSupport.getDoubleValue();
      double comHeight = computeCoMHeightUsingOneFoot(supportLeg);
      return computeDesiredCoP(supportPolygon, desiredCapturePoint, finalTime, comHeight);
   }

   // using desired capture point
   public FramePoint2d computeDesiredCoPDoubleSupport(OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint, FrameVector2d desiredVelocity,
         FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity)
   {
      FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getSupportPolygonInMidFeetZUp();

      double estimatedDoubleSupportTimeRemaining = couplingRegistry.getEstimatedDoubleSupportTimeRemaining();
      double finalTime;
      if (Double.isInfinite(estimatedDoubleSupportTimeRemaining))
         finalTime = minimumEqConstTimespanDoubleSupport.getDoubleValue();
      else
         finalTime = Math.max(estimatedDoubleSupportTimeRemaining, 50e-3);
      double comHeight = computeCoMHeightUsingBothFeet();
      return computeDesiredCoP(supportPolygon, desiredCapturePoint, finalTime, comHeight);
   }
   
   // using guide line
   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
         FrameVector2d desiredVelocity, FrameLineSegment2d guideLine)
   {
      FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getFootPolygonInAnkleZUp(supportLeg);
      FramePoint2d desiredFinalCapturePoint = guideLine.pointBetweenEndPointsGivenParameter(guideLineParameter.getDoubleValue());
      double finalTime = Math.max(couplingRegistry.getEstimatedSwingTimeRemaining(), 50e-3);    // FIXME: hack
      double comHeight = computeCoMHeightUsingOneFoot(supportLeg);
      FramePoint2d desiredCoP = computeDesiredCoP(supportPolygon, desiredFinalCapturePoint, finalTime, comHeight);
      doOrthogonalControl(desiredCoP, capturePoint, guideLine);
      return desiredCoP;
   }

   private FramePoint2d computeDesiredCoP(FrameConvexPolygon2d supportPolygon, FramePoint2d desiredFinalCapturePoint, double finalTime, double comHeight)
   {
      ReferenceFrame supportPolygonFrame = supportPolygon.getReferenceFrame();

      FramePoint currentCapturePoint = new FramePoint(couplingRegistry.getCapturePointInFrame(supportPolygonFrame));
      FramePoint2d currentCapturePoint2d = currentCapturePoint.toFramePoint2d();

      desiredFinalCapturePoint.changeFrame(supportPolygonFrame);

      double gravity = -processedSensors.getGravityInWorldFrame().getZ();
      FramePoint2d ret;
      if (finalTime > 0.0)
         ret = EquivalentConstantCoPCalculator.computeEquivalentConstantCoP(currentCapturePoint2d, desiredFinalCapturePoint, finalTime, comHeight, gravity);
      else
      {
         ret = desiredFinalCapturePoint;    // will stay like this only if line from current to desired ICP does not intersect support polygon
         FrameLine2d currentToDesired = new FrameLine2d(currentCapturePoint2d, desiredFinalCapturePoint);
         FramePoint2d[] intersections = supportPolygon.intersectionWith(currentToDesired);

         if (intersections != null)
         {
            double maxDistanceSquaredToDesired = Double.NEGATIVE_INFINITY;
            for (FramePoint2d intersection : intersections)
            {
               double distanceSquared = intersection.distanceSquared(desiredFinalCapturePoint);
               if (distanceSquared > maxDistanceSquaredToDesired)
               {
                  ret = intersection;
                  maxDistanceSquaredToDesired = distanceSquared;
               }
            }
         }
      }

      supportPolygon.orthogonalProjection(ret);
      return ret;
   }

   private double computeCoMHeightUsingOneFoot(RobotSide sideToGetCoMHeightFor)
   {
      ReferenceFrame footFrame = referenceFrames.getAnkleZUpReferenceFrames().get(sideToGetCoMHeightFor);
      FramePoint centerOfMass = processedSensors.getCenterOfMassPositionInFrame(footFrame);

      return centerOfMass.getZ();
   }

   private double computeCoMHeightUsingBothFeet()
   {
      double sum = 0.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         sum += computeCoMHeightUsingOneFoot(robotSide);
      }

      return sum / RobotSide.values.length;
   }
   
   private void doOrthogonalControl(FramePoint2d desiredCoPToChange, FramePoint2d capturePoint, FrameLineSegment2d guideLine)
   {
      FramePoint2d projectedCapturePoint = guideLine.orthogonalProjectionCopy(capturePoint);
      FrameVector2d control = new FrameVector2d(capturePoint);
      control.sub(projectedCapturePoint);
      control.scale(guideLineKp.getDoubleValue());
      guideLine.orthogonalProjection(desiredCoPToChange);
      desiredCoPToChange.add(control);
   }

   public void setParametersForM2V2()
   {
      minimumEqConstTimespanDoubleSupport.set(0.13);
      minimumEqConstTimespanSingleSupport.set(0.2);
      guideLineParameter.set(0.4);
      guideLineKp.set(3.0);
   }
}
