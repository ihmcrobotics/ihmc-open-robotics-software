package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointToDesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.GuideLineToDesiredCoPControlModule;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
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
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.frames.YoFrameLine2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLine2d;


public class SpeedControllingDesiredCoPCalculator implements DesiredCapturePointToDesiredCoPControlModule, GuideLineToDesiredCoPControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleDesiredCapturePointToDesiredCoPControlModule");

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final ProcessedSensorsInterface processedSensors;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable doubleSupportCaptureKp = new DoubleYoVariable("doubleSupportCaptureKp", registry);
   private final DoubleYoVariable singleSupportCaptureKp = new DoubleYoVariable("singleSupportCaptureKp", registry);
//   private final DoubleYoVariable perimeterDistance = new DoubleYoVariable("supportPolygonPerimeterDistance", registry);
//   private final DoubleYoVariable minPerimeterDistance = new DoubleYoVariable("minSupportPolygonPerimeterDistance", registry);
   private final DoubleYoVariable kCaptureGuide = new DoubleYoVariable("kCaptureGuide", "ICP distance to guide line --> position of parallel line", registry);
   private final DoubleYoVariable speedControlXKp = new DoubleYoVariable("speedControlXKp", registry);
   private final DoubleYoVariable velocityError = new DoubleYoVariable("velocityError", registry);
   private final DoubleYoVariable desiredVelocityLength = new DoubleYoVariable("desiredVelocityLength", registry);

   private final YoFrameLine2d parallelLineWorld = new YoFrameLine2d("parallelLine", "", world, registry);
   private final YoFrameLine2d speedControlLineWorld = new YoFrameLine2d("capturePointLine", "", world, registry);

   private final YoFramePoint comPosition = new YoFramePoint("comPosition", "", world, registry);
   private final YoFrameVector desiredVelocityInWorld = new YoFrameVector("desiredVelocityInWorld", "", world, registry);
   private final YoFrameVector actualVelocityInWorld = new YoFrameVector("actualVelocityInWorld", "", world, registry);
   
   private final YoFramePoint2d desiredCoPBeforeProjection = new YoFramePoint2d("desiredCoPBeforeProjection", "", world, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);
   
   public SpeedControllingDesiredCoPCalculator(ProcessedSensorsInterface processedSensors, CommonHumanoidReferenceFrames referenceFrames,
           YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      parentRegistry.addChild(registry);
      
      if (yoGraphicsListRegistry != null)
      {
         YoGraphicsList yoGraphicList = new YoGraphicsList("CapturePointController");
         ArtifactList artifactList = new ArtifactList("Capture Point CoP Control Module");

         YoGraphicPosition desiredCoPBeforeProjectionViz = new YoGraphicPosition("desiredCoPBeforeProjection", desiredCoPBeforeProjection, 0.01, YoAppearance.DarkGreen(), GraphicType.CROSS);
         yoGraphicList.add(desiredCoPBeforeProjectionViz);
         artifactList.add(desiredCoPBeforeProjectionViz.createArtifact());
         

         YoArtifactLine2d speedControlLineArtifact = new YoArtifactLine2d("speedControlLineWorld", speedControlLineWorld, Color.BLUE);
         artifactList.add(speedControlLineArtifact);

         YoArtifactLine2d parallellLineArtifact = new YoArtifactLine2d("Parallel Line", parallelLineWorld, Color.GREEN);
         artifactList.add(parallellLineArtifact);
         
         YoGraphicVector actualVelocityDynamicGraphicVector = new YoGraphicVector("actualVelocity", comPosition, actualVelocityInWorld, 1.0, YoAppearance.Maroon());
         YoGraphicVector desiredVelocityDynamicGraphicVector = new YoGraphicVector("desiredVelocity", comPosition, desiredVelocityInWorld, 1.0, YoAppearance.Pink());
         yoGraphicList.add(actualVelocityDynamicGraphicVector);
         yoGraphicList.add(desiredVelocityDynamicGraphicVector);
         
//         artifactList.add(actualVelocityDynamicGraphicVector.createArtifact());
//         artifactList.add(desiredVelocityDynamicGraphicVector.createArtifact());
         
         
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
      omega0.set(Double.POSITIVE_INFINITY);
   }

   // compute desired CoP in single support using desired capture point
   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
           FrameVector2d desiredVelocity, FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity)
   {
      desiredVelocityLength.set(desiredVelocity.length());

      FrameConvexPolygon2d footPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
      FramePoint2d desiredCenterOfPressure = doProportionalControl(capturePoint, desiredCapturePoint, desiredCapturePointVelocity, singleSupportCaptureKp.getDoubleValue());
      FrameLine2d controlLine = new FrameLine2d(desiredCenterOfPressure, desiredCapturePoint);
      desiredCenterOfPressure.changeFrame(footPolygon.getReferenceFrame());
      controlLine.changeFrame(footPolygon.getReferenceFrame());
      GeometryTools.movePointInsidePolygonAlongLine(desiredCenterOfPressure, footPolygon, controlLine);

      return desiredCenterOfPressure;
   }

   // compute desired CoP in single support using guide line
   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
           FrameVector2d desiredVelocity, FrameLineSegment2d guideLine)
   {
      desiredVelocityLength.set(desiredVelocity.length());

      FrameConvexPolygon2d footPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
      
      // Create parallel line
      FramePoint2d captureProjectedOntoGuideLine = guideLine.orthogonalProjectionCopy(capturePoint);

      FrameVector2d projectedToCurrent = new FrameVector2d(captureProjectedOntoGuideLine, capturePoint);
      projectedToCurrent.scale(kCaptureGuide.getDoubleValue());

      FramePoint2d shiftedPoint = new FramePoint2d(captureProjectedOntoGuideLine);
      shiftedPoint.add(projectedToCurrent);

      FrameVector2d frameVector2d = new FrameVector2d();
      guideLine.getFrameVector(frameVector2d);
      FrameLine2d shiftedParallelLine = new FrameLine2d(shiftedPoint, frameVector2d);
      FrameLine2d shiftedParallelLineInWorld = new FrameLine2d(shiftedParallelLine);
      shiftedParallelLineInWorld.changeFrame(world);
      parallelLineWorld.setFrameLine2d(shiftedParallelLineInWorld);

      // Create speed control line
      FrameLine2d massLine = createSpeedControlLine(new FrameLine2d(guideLine), desiredVelocity);
      massLine.changeFrame(shiftedParallelLine.getReferenceFrame());
      FramePoint2d desiredCenterOfPressure = shiftedParallelLine.intersectionWith(massLine);

      desiredCenterOfPressure.changeFrame(footPolygon.getReferenceFrame());
      shiftedParallelLine.changeFrame(footPolygon.getReferenceFrame());

      GeometryTools.movePointInsidePolygonAlongLine(desiredCenterOfPressure, footPolygon, shiftedParallelLine);
      
      return desiredCenterOfPressure;
   }

   // compute desired CoP in double support using desired capture point
   public FramePoint2d computeDesiredCoPDoubleSupport(OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint, FrameVector2d desiredVelocity,
           FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity)
   {
      desiredVelocityLength.set(desiredVelocity.length());

      parallelLineWorld.setFrameLine2d(null);

      desiredCapturePoint.changeFrame(capturePoint.getReferenceFrame());
      desiredCapturePointVelocity.changeFrame(capturePoint.getReferenceFrame());
      
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
//      FrameLineSegment2d closestEdge = supportPolygon.getClosestEdge(capturePoint);
//      perimeterDistance.set(closestEdge.distance(capturePoint));
//
//      // Handle large disturbances where the iCP is outside the support polygon
//      if (!supportPolygon.isPointInside(capturePoint))
//      {
//         FramePoint2d farthestToDesiredCP = determineFarthestPoint(desiredCapturePoint, closestEdge);
//         FrameLine2d iCPLine = new FrameLine2d(capturePoint, farthestToDesiredCP);
//         speedControlLineWorld.setFrameLine2d(iCPLine.changeFrameCopy(world));
//         farthestToDesiredCP.changeFrame(world);
//
//         return farthestToDesiredCP;
//
//         // TODO: don't like this. The desired that's being passed in is wrong, we shouldn't change anything here.
//      }

//      // Handle large disturbances where the iCP is almost outside the support polygon
//      if (perimeterDistance.getDoubleValue() < minPerimeterDistance.getDoubleValue())
//      {
//         FramePoint2d closestToDesiredCP = determineClosestVertex(desiredCapturePoint, closestEdge);
//         double ratio = (minPerimeterDistance.getDoubleValue() - perimeterDistance.getDoubleValue()) / minPerimeterDistance.getDoubleValue();
//         desiredCapturePoint.setX((1.0 - ratio) * desiredCapturePoint.getX() + ratio * closestToDesiredCP.getX());
//         desiredCapturePoint.setY((1.0 - ratio) * desiredCapturePoint.getY() + ratio * closestToDesiredCP.getY());
//
//         // TODO: don't like this. The desired that's being passed in is wrong, we shouldn't change anything here.
//      }

      FramePoint2d centerOfPressureDesired;
      
      // Create Line from desired Capture Point to instantaneous Capture Point
      if (capturePoint.distance(desiredCapturePoint) > 1e-9)
      {
         FrameLine2d controlLine = new FrameLine2d(capturePoint, desiredCapturePoint);

         if (desiredVelocityLength.getDoubleValue() < 1e-7)
         {
            centerOfPressureDesired = doProportionalControl(capturePoint, desiredCapturePoint, desiredCapturePointVelocity, doubleSupportCaptureKp.getDoubleValue());
            hideControlLine();
            
            centerOfPressureDesired.changeFrame(world);
            desiredCoPBeforeProjection.set(centerOfPressureDesired);
            centerOfPressureDesired.changeFrame(supportPolygon.getReferenceFrame());
         }
         else
         {
            FrameLine2d controlLineInWorld = new FrameLine2d(controlLine);
            controlLineInWorld.changeFrame(world);
            speedControlLineWorld.setFrameLine2d(controlLineInWorld);

            ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
            FrameVector2d comDirection = new FrameVector2d(desiredVelocity);
            comDirection.changeFrame(midFeetZUpFrame);
            comDirection.normalize();
            FrameVector2d controlDirection = new FrameVector2d();
            controlLine.getNormalizedFrameVector(controlDirection);

            // If the scalar projection of the desired CoM direction on the desired iCP direction is negative
            // control only the iCP position and don't do speed control.
            if (comDirection.dot(controlDirection) < 0.0)
            {
               centerOfPressureDesired = doProportionalControl(capturePoint, desiredCapturePoint, desiredCapturePointVelocity, doubleSupportCaptureKp.getDoubleValue());
            }
            else
            {
               FrameLine2d massLine = createSpeedControlLine(controlLine, desiredVelocity);
               centerOfPressureDesired = controlLine.intersectionWith(massLine);
            }

            centerOfPressureDesired.changeFrame(supportPolygon.getReferenceFrame());
         }
         FrameLine2d controlLineInSupportFrame = new FrameLine2d(controlLine);
         controlLineInSupportFrame.changeFrame(supportPolygon.getReferenceFrame());
         GeometryTools.movePointInsidePolygonAlongLine(centerOfPressureDesired, supportPolygon, controlLineInSupportFrame);
      }
      else
      {
         centerOfPressureDesired = new FramePoint2d(desiredCapturePoint);
      }


      return centerOfPressureDesired;
   }

//   private FramePoint2d determineFarthestPoint(FramePoint2d testPoint, FrameLineSegment2d lineSegment)
//   {
//      testPoint.checkReferenceFrameMatch(lineSegment);
//
//      Point2d[] endPoints = lineSegment.getLineSegment2d().getEndpoints();
//      double farthestDistance = Double.NEGATIVE_INFINITY;
//      int farthestIndex = -1;
//      for (int i = 0; i < endPoints.length; i++)
//      {
//         double distance = endPoints[i].distance(testPoint.getPoint());
//         if (distance > farthestDistance)
//         {
//            farthestDistance = distance;
//            farthestIndex = i;
//         }
//      }
//
//      return new FramePoint2d(testPoint.getReferenceFrame(), endPoints[farthestIndex]);
//   }
//
//   private FramePoint2d determineClosestVertex(FramePoint2d testPoint, FrameLineSegment2d lineSegment)
//   {
//      testPoint.checkReferenceFrameMatch(lineSegment);
//
//      Point2d[] endPoints = lineSegment.getLineSegment2d().getEndpoints();
//      double closestDistance = Double.POSITIVE_INFINITY;
//      int closestIndex = -1;
//      for (int i = 0; i < endPoints.length; i++)
//      {
//         double distance = endPoints[i].distance(testPoint.getPoint());
//         if (distance < closestDistance)
//         {
//            closestDistance = distance;
//            closestIndex = i;
//         }
//      }
//
//      return new FramePoint2d(testPoint.getReferenceFrame(), endPoints[closestIndex]);
//   }
   
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   private FrameLine2d createSpeedControlLine(FrameLine2d guideLine, FrameVector2d desiredVelocity)
   {
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FrameVector2d currentVelocity = processedSensors.getCenterOfMassVelocityInFrame(midFeetZUpFrame).toFrameVector2d();
      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(midFeetZUpFrame);
      centerOfMassPosition.changeFrame(world);
      currentVelocity.changeFrame(world);
      desiredVelocity.changeFrame(world);
      
      comPosition.set(centerOfMassPosition);
      actualVelocityInWorld.setXY(currentVelocity);
      desiredVelocityInWorld.setXY(desiredVelocity);
      
      double desiredVelocityMagnitude = desiredVelocity.length();

//    desiredVelocity = desiredVelocity.changeFrameCopy(desiredHeadingFrame);
//    ReferenceFrame desiredVelocityFrame = desiredVelocity.getReferenceFrame();
//    desiredVelocityFrame.checkReferenceFrameMatch(desiredHeadingFrame);

      FrameVector2d guideLineUnitVector = new FrameVector2d();
      guideLine.getNormalizedFrameVector(guideLineUnitVector);

      FrameVector2d currentVelocityInFrame = new FrameVector2d(currentVelocity);
      currentVelocityInFrame.changeFrame(guideLineUnitVector.getReferenceFrame());

      double currentVelocityProjectedIntoGuideLine = currentVelocityInFrame.dot(guideLineUnitVector);
      velocityError.set(desiredVelocityMagnitude - currentVelocityProjectedIntoGuideLine);

      FrameVector2d controlOffset = new FrameVector2d(guideLineUnitVector);
      controlOffset.scale(-speedControlXKp.getDoubleValue() * velocityError.getDoubleValue());


      FramePoint centerOfMassPositionInFrame = new FramePoint(centerOfMassPosition);
      centerOfMassPositionInFrame.changeFrame(controlOffset.getReferenceFrame());
      FramePoint2d centerOfMassPosition2dInFrame = new FramePoint2d();
      centerOfMassPositionInFrame.getFramePoint2d(centerOfMassPosition2dInFrame);

      // Project CoM on control line
      FrameVector2d velocityT = new FrameVector2d(guideLineUnitVector.getReferenceFrame());
      velocityT.setX(-guideLineUnitVector.getY());
      velocityT.setY(guideLineUnitVector.getX());


      FramePoint2d speedControlPosition = new FramePoint2d(centerOfMassPosition2dInFrame);
      speedControlPosition.add(controlOffset);

//    // Speed controller: Only increase speed for now
//    speedControlPosition.setX(speedControlPosition.getX()
//                              + speedControlXKp.getDoubleValue() * Math.min(0.0, currentVelocityInFrame.getX() - desiredVelocity.getX()));
//    speedControlPosition.setY(speedControlPosition.getY()
//                              + speedControlYKp.getDoubleValue() * Math.min(0.0, currentVelocityInFrame.getY() - desiredVelocity.getY()));


      if (velocityT.length() == 0.0)
         throw new RuntimeException("Not sure what to do when velocity is zero");

      FrameLine2d speedControlLine = new FrameLine2d(speedControlPosition, velocityT);
      FrameLine2d speedControlLineInWorld = new FrameLine2d(speedControlLine);
      speedControlLineInWorld.changeFrame(world);
      speedControlLineWorld.setFrameLine2d(speedControlLineInWorld);

      return speedControlLine;
   }

   private final FrameVector2d tempControl = new FrameVector2d(ReferenceFrame.getWorldFrame());

   private FramePoint2d doProportionalControl(FramePoint2d capturePoint, FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity, double captureKp)
   {
      desiredCapturePointVelocity.changeFrame(desiredCapturePoint.getReferenceFrame());
      FramePoint2d desiredCenterOfPressure = new FramePoint2d(capturePoint);
      
      // feed forward part
      tempControl.setIncludingFrame(desiredCapturePointVelocity);
      tempControl.scale(1.0 / omega0.getDoubleValue());
      desiredCenterOfPressure.sub(tempControl);
      
      // feedback part
      tempControl.setIncludingFrame(capturePoint);
      tempControl.sub(desiredCapturePoint);
      tempControl.scale(captureKp);
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
      doubleSupportCaptureKp.set(3.0);    // 1.0); //5.0);
      singleSupportCaptureKp.set(3.0);
      kCaptureGuide.set(1.5);    // 2.0);
//      minPerimeterDistance.set(0.04);    // 0.02);
   }

   public void setParametersForR2InverseDynamics()
   {
      speedControlXKp.set(0.0);
      doubleSupportCaptureKp.set(1.5); // 0.5); // 0.5
      singleSupportCaptureKp.set(1.5); // 0.5); // 1.0 // 0.5
      kCaptureGuide.set(1.5);
   }
   
   public void setParametersForM2V2()
   {
      speedControlXKp.set(0.5);
      doubleSupportCaptureKp.set(2.5);    // 1.0); //5.0);
      singleSupportCaptureKp.set(1.5);
      kCaptureGuide.set(2.0);
//      minPerimeterDistance.set(0.02);
   }
}
