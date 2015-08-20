package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointToDesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCenterOfPressureFilter;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class LungingBasedDesiredCoPControlModule implements DesiredCoPControlModule
{
   private final DesiredCenterOfPressureFilter desiredCenterOfPressureFilter;
   private final LungingBasedDesiredCoPCMPVisualizer visualizer;

   private final CouplingRegistry couplingRegistry;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final DesiredCapturePointCalculator desiredCapturePointCalculator;
   private final DesiredCapturePointToDesiredCoPControlModule desiredCapturePointToDesiredCoPControlModule;

   public LungingBasedDesiredCoPControlModule(DesiredCapturePointCalculator desiredCapturePointCalculator,
         DesiredCapturePointToDesiredCoPControlModule desiredCapturePointToDesiredCoPControlModule,
         DesiredCenterOfPressureFilter desiredCenterOfPressureFilter, LungingBasedDesiredCoPCMPVisualizer visualizer, CouplingRegistry couplingRegistry,
         CommonHumanoidReferenceFrames referenceFrames)
   {
      this.desiredCenterOfPressureFilter = desiredCenterOfPressureFilter;
      this.visualizer = visualizer;
      this.couplingRegistry = couplingRegistry;
      this.referenceFrames = referenceFrames;
      this.desiredCapturePointCalculator = desiredCapturePointCalculator;
      this.desiredCapturePointToDesiredCoPControlModule = desiredCapturePointToDesiredCoPControlModule;
   }

   /**
    *  not used in lunging for now
    */
   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, FrameVector2d desiredVelocity, SingleSupportCondition singleSupportCondition,
         double timeInState)
   {
      return new FramePoint2d(referenceFrames.getAnkleZUpFrame(supportLeg));
   }

   public FramePoint2d computeDesiredCoPDoubleSupport(RobotSide loadingLeg, FrameVector2d desiredVelocity)
   {
      FramePoint2d desiredCoP;
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(referenceFrames.getMidFeetZUpFrame());
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = couplingRegistry.getOldBipedSupportPolygons().getSupportPolygonInMidFeetZUp();

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      if (lungeAxis == null)
      {
         // from CapturabilityBasedDesiredCoPControlModule
         OldBipedSupportPolygons bipedSupportPolygons = couplingRegistry.getOldBipedSupportPolygons();
         FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(midFeetZUpFrame).toFramePoint2d();

         FramePoint2d desiredCapturePoint = desiredCapturePointCalculator.computeDesiredCapturePointDoubleSupport(loadingLeg, bipedSupportPolygons,
               desiredVelocity);
         visualizer.setDesiredCapturePoint(desiredCapturePoint);
         desiredCoP = desiredCapturePointToDesiredCoPControlModule.computeDesiredCoPDoubleSupport(bipedSupportPolygons, capturePoint, desiredVelocity,
               desiredCapturePoint, new FrameVector2d(desiredCapturePoint.getReferenceFrame()));
      }
      else
      {
         FramePoint2d desiredCoPDirection = getDesiredCoPOrthogonalToLungeAxisCopy(midFeetZUpFrame, lungeAxis);
         desiredCoP = clipLineOnSupportPolygon(supportPolygonInMidFeetZUp, supportPolygonInMidFeetZUp.getCentroidCopy(), desiredCoPDirection);
         // putting it on the edge makes the feet roll/pitch
         desiredCoP.scale(0.75);
      }
      
      desiredCoP = desiredCenterOfPressureFilter.filter(desiredCoP, null);
      visualizer.setDesiredCoP(desiredCoP);

      visualizer.setDesiredCMP(couplingRegistry.getDesiredCMP()); //TODO this method should only do CoP

      return desiredCoP;
   }

   private FramePoint2d getDesiredCoPOrthogonalToLungeAxisCopy(ReferenceFrame expressedInFrame, FrameVector2d lungeAxis)   
   {
      FramePoint2d desiredCoP = new FramePoint2d(lungeAxis.getReferenceFrame(), lungeAxis.getY(), -lungeAxis.getX());
      desiredCoP.changeFrame(expressedInFrame);
      return desiredCoP;
   }
   
   private FramePoint2d clipLineOnSupportPolygon(FrameConvexPolygon2d supportPolygonInMidFeetZUp, FramePoint2d startPoint, FramePoint2d endPoint)
   {
      FrameLine2d directionOfPoint = new FrameLine2d(startPoint, endPoint);
      FramePoint2d[] intersectionPoints = directionOfPoint.intersectionWith(supportPolygonInMidFeetZUp);
      FramePoint2d ret;
      
      // no intersections
      if (intersectionPoints == null)
      {
         return new FramePoint2d(endPoint.getReferenceFrame());
      }
      
      // select point which is in the direction specified by the endPoint
      int nIntersections = intersectionPoints.length;
      if (nIntersections == 2)
      {
         FrameVector2d[] possiblePoints = new FrameVector2d[2];
         for (int i = 0 ; i < nIntersections ; i ++)
         {
            possiblePoints[i] = new FrameVector2d(intersectionPoints[i]);
            possiblePoints[i].sub(startPoint);
            possiblePoints[i].normalize();
            possiblePoints[i].sub(endPoint);
         }
         
         if (possiblePoints[0].length() < possiblePoints[1].length())
         {
            ret = intersectionPoints[0];
         }
         else
         {
            ret = intersectionPoints[1];
         }
      }
      else
      {
            throw new RuntimeException("Other than two points found");
            
      }
      return ret;
   }
}
