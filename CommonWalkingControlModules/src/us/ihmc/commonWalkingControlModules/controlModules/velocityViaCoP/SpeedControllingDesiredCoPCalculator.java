package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import javax.vecmath.Point2d;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLine2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointToDesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.GuideLineToDesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class SpeedControllingDesiredCoPCalculator implements DesiredCapturePointToDesiredCoPControlModule, GuideLineToDesiredCoPControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleDesiredCapturePointToDesiredCoPControlModule");

   private final CommonWalkingReferenceFrames referenceFrames;
   private final ProcessedSensorsInterface processedSensors;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable captureKp = new DoubleYoVariable("doubleSupportCaptureKp", registry);
   private final DoubleYoVariable perimeterDistance = new DoubleYoVariable("supportPolygonPerimeterDistance", registry);
   private final DoubleYoVariable minPerimeterDistance = new DoubleYoVariable("minSupportPolygonPerimeterDistance", registry);
   private final DoubleYoVariable kCaptureGuide = new DoubleYoVariable("kCaptureGuide", "ICP distance to guide line --> position of parallel line", registry);
   private final DoubleYoVariable speedControlXKp = new DoubleYoVariable("speedControlXKp", registry);

   private final YoFrameLine2d speedControlLineWorld = new YoFrameLine2d("capturePointLine", "", world, registry);

   public SpeedControllingDesiredCoPCalculator(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames,
           YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      parentRegistry.addChild(registry);
   }

   // compute desired CoP in single support using desired capture point
   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
           FrameVector2d desiredVelocity, FramePoint2d desiredCapturePoint)
   {
      FrameConvexPolygon2d footPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
      FramePoint2d desiredCenterOfPressure = doProportionalControl(capturePoint, desiredCapturePoint);
      FrameLine2d controlLine = new FrameLine2d(desiredCenterOfPressure, desiredCapturePoint);
      GeometryTools.movePointInsidePolygonAlongLine(desiredCenterOfPressure, footPolygon, controlLine);

      return desiredCenterOfPressure;
   }

   // compute desired CoP in single support using guide line
   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
           FrameVector2d desiredVelocity, FrameLineSegment2d guideLine)
   {
      FrameConvexPolygon2d footPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
      
      // Create parallel line
      FramePoint2d captureProjectedOntoGuideLine = guideLine.orthogonalProjectionCopy(capturePoint);

      FrameVector2d projectedToCurrent = new FrameVector2d(captureProjectedOntoGuideLine, capturePoint);
      projectedToCurrent.scale(kCaptureGuide.getDoubleValue());

      FramePoint2d shiftedPoint = new FramePoint2d(captureProjectedOntoGuideLine);
      shiftedPoint.add(projectedToCurrent);

      FrameVector2d frameVector2d = guideLine.getVectorCopy();
      FrameLine2d shiftedParallelLine = new FrameLine2d(shiftedPoint, frameVector2d);


      // Create speed control line
      FrameLine2d massLine = createSpeedControlLine(new FrameLine2d(guideLine), desiredVelocity);
      massLine.changeFrame(shiftedParallelLine.getReferenceFrame());
      FramePoint2d desiredCenterOfPressure = shiftedParallelLine.intersectionWith(massLine);

      desiredCenterOfPressure.changeFrame(footPolygon.getReferenceFrame());

      GeometryTools.movePointInsidePolygonAlongLine(desiredCenterOfPressure, footPolygon, shiftedParallelLine);
      
      return desiredCenterOfPressure;
   }

   // compute desired CoP in double support using desired capture point
   public FramePoint2d computeDesiredCoPDoubleSupport(BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint, FrameVector2d desiredVelocity,
           FramePoint2d desiredCapturePoint)
   {
      desiredCapturePoint.changeFrame(capturePoint.getReferenceFrame());
      
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      FrameLineSegment2d closestEdge = supportPolygon.getClosestEdge(capturePoint);
      perimeterDistance.set(closestEdge.distance(capturePoint));

      // Handle large disturbances where the iCP is outside the support polygon
      if (!supportPolygon.isPointInside(capturePoint))
      {
         FramePoint2d farthestToDesiredCP = determineFarthestPoint(desiredCapturePoint, closestEdge);
         FrameLine2d iCPLine = new FrameLine2d(capturePoint, farthestToDesiredCP);
         speedControlLineWorld.setFrameLine2d(iCPLine.changeFrameCopy(world));
         farthestToDesiredCP.changeFrame(world);

         return farthestToDesiredCP;

         // TODO: don't like this. The desired that's being passed in is wrong, we shouldn't change anything here.
      }

      // Handle large disturbances where the iCP is almost outside the support polygon
      if (perimeterDistance.getDoubleValue() < minPerimeterDistance.getDoubleValue())
      {
         FramePoint2d closestToDesiredCP = determineClosestVertex(desiredCapturePoint, closestEdge);
         double ratio = (minPerimeterDistance.getDoubleValue() - perimeterDistance.getDoubleValue()) / minPerimeterDistance.getDoubleValue();
         desiredCapturePoint.setX((1.0 - ratio) * desiredCapturePoint.getX() + ratio * closestToDesiredCP.getX());
         desiredCapturePoint.setY((1.0 - ratio) * desiredCapturePoint.getY() + ratio * closestToDesiredCP.getY());

         // TODO: don't like this. The desired that's being passed in is wrong, we shouldn't change anything here.
      }

      FramePoint2d centerOfPressureDesired = null;
      if (desiredVelocity.length() == 0.0)
      {
         centerOfPressureDesired = doProportionalControl(capturePoint, desiredCapturePoint);
         hideControlLine();
         centerOfPressureDesired.changeFrame(supportPolygon.getReferenceFrame());
         supportPolygon.orthogonalProjection(centerOfPressureDesired);
      }
      else
      {
         // Create Line from desired Capture Point to instantaneous Capture Point
         FrameLine2d controlLine = new FrameLine2d(capturePoint, desiredCapturePoint);
         speedControlLineWorld.setFrameLine2d(controlLine.changeFrameCopy(world));

         ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
         FrameVector2d comDirection = desiredVelocity.changeFrameCopy(midFeetZUpFrame);
         comDirection.normalize();
         FrameVector2d controlDirection = controlLine.getNormalizedFrameVector();

         // If the scalar projection of the desired CoM direction on the desired iCP direction is negative
         // control only the iCP position and don't do speed control.
         if (comDirection.dot(controlDirection) < 0.0)
         {
            centerOfPressureDesired = doProportionalControl(capturePoint, desiredCapturePoint);
         }
         else
         {
            FrameLine2d massLine = createSpeedControlLine(controlLine, desiredVelocity);
            centerOfPressureDesired = controlLine.intersectionWith(massLine);
         }

         centerOfPressureDesired.changeFrame(supportPolygon.getReferenceFrame());
         GeometryTools.movePointInsidePolygonAlongLine(centerOfPressureDesired, supportPolygon, new FrameLine2d(controlLine));
      }

      return centerOfPressureDesired;
   }

   private FramePoint2d determineFarthestPoint(FramePoint2d testPoint, FrameLineSegment2d lineSegment)
   {
      testPoint.checkReferenceFrameMatch(lineSegment);

      Point2d[] endPoints = lineSegment.getLineSegment2d().getEndpoints();
      double farthestDistance = Double.NEGATIVE_INFINITY;
      int farthestIndex = -1;
      for (int i = 0; i < endPoints.length; i++)
      {
         double distance = endPoints[i].distance(testPoint.getPoint());
         if (distance > farthestDistance)
         {
            farthestDistance = distance;
            farthestIndex = i;
         }
      }

      return new FramePoint2d(testPoint.getReferenceFrame(), endPoints[farthestIndex]);
   }

   private FramePoint2d determineClosestVertex(FramePoint2d testPoint, FrameLineSegment2d lineSegment)
   {
      testPoint.checkReferenceFrameMatch(lineSegment);

      Point2d[] endPoints = lineSegment.getLineSegment2d().getEndpoints();
      double closestDistance = Double.POSITIVE_INFINITY;
      int closestIndex = -1;
      for (int i = 0; i < endPoints.length; i++)
      {
         double distance = endPoints[i].distance(testPoint.getPoint());
         if (distance < closestDistance)
         {
            closestDistance = distance;
            closestIndex = i;
         }
      }

      return new FramePoint2d(testPoint.getReferenceFrame(), endPoints[closestIndex]);
   }

   private FrameLine2d createSpeedControlLine(FrameLine2d guideLine, FrameVector2d desiredVelocity)
   {
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FrameVector2d currentVelocity = processedSensors.getCenterOfMassVelocityInFrame(midFeetZUpFrame, null).toFrameVector2d();
      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(midFeetZUpFrame);
      
      double desiredVelocityMagnitude = desiredVelocity.length();

//    desiredVelocity = desiredVelocity.changeFrameCopy(desiredHeadingFrame);
//    ReferenceFrame desiredVelocityFrame = desiredVelocity.getReferenceFrame();
//    desiredVelocityFrame.checkReferenceFrameMatch(desiredHeadingFrame);

      FrameVector2d guideLineUnitVector = guideLine.getNormalizedFrameVector();

      FrameVector2d currentVelocityInFrame = currentVelocity.changeFrameCopy(guideLineUnitVector.getReferenceFrame());

      double currentVelocityProjectedIntoGuideLine = currentVelocityInFrame.dot(guideLineUnitVector);
      double velocityError = desiredVelocityMagnitude - currentVelocityProjectedIntoGuideLine;

      FrameVector2d controlOffset = new FrameVector2d(guideLineUnitVector);
      controlOffset.scale(-speedControlXKp.getDoubleValue() * velocityError);


      FramePoint2d centerOfMassPositionInFrame = centerOfMassPosition.changeFrameCopy(controlOffset.getReferenceFrame()).toFramePoint2d();

      // Project CoM on control line
      FrameVector2d velocityT = new FrameVector2d(guideLineUnitVector.getReferenceFrame());
      velocityT.setX(-guideLineUnitVector.getY());
      velocityT.setY(guideLineUnitVector.getX());


      FramePoint2d speedControlPosition = new FramePoint2d(centerOfMassPositionInFrame);
      speedControlPosition.add(controlOffset);

//    // Speed controller: Only increase speed for now
//    speedControlPosition.setX(speedControlPosition.getX()
//                              + speedControlXKp.getDoubleValue() * Math.min(0.0, currentVelocityInFrame.getX() - desiredVelocity.getX()));
//    speedControlPosition.setY(speedControlPosition.getY()
//                              + speedControlYKp.getDoubleValue() * Math.min(0.0, currentVelocityInFrame.getY() - desiredVelocity.getY()));


      if (velocityT.length() == 0.0)
         throw new RuntimeException("Not sure what to do when velocity is zero");

      FrameLine2d speedControlLine = new FrameLine2d(speedControlPosition, velocityT);
      speedControlLineWorld.setFrameLine2d(speedControlLine.changeFrameCopy(world));

      return speedControlLine;
   }

   private final FrameVector2d tempControl = new FrameVector2d(ReferenceFrame.getWorldFrame());

   private FramePoint2d doProportionalControl(FramePoint2d capturePoint, FramePoint2d desiredCapturePoint)
   {
      FramePoint2d desiredCenterOfPressure = new FramePoint2d(desiredCapturePoint);
      tempControl.setAndChangeFrame(capturePoint);
      tempControl.sub(desiredCapturePoint);
      tempControl.scale(captureKp.getDoubleValue());
      desiredCenterOfPressure.add(tempControl);

      return desiredCenterOfPressure;
   }

   private void hideControlLine()
   {
      speedControlLineWorld.setFrameLine2d(null);
   }

   public void setParametersForR2()
   {
      speedControlXKp.set(3.0);
      captureKp.set(4.0);    // 2.0); //6.0);
      kCaptureGuide.set(1.5);    // 2.0);
      minPerimeterDistance.set(0.04);    // 0.02);
   }

   public void setParametersForM2V2()
   {
      speedControlXKp.set(0.5);
      captureKp.set(3.5);    // 2.0); //6.0);
      kCaptureGuide.set(2.0);
      minPerimeterDistance.set(0.02);
   }
}
