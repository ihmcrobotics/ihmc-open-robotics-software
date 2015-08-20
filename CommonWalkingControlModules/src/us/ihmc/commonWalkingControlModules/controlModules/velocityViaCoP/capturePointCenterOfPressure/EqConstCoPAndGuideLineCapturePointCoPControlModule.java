package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.capturePointCenterOfPressure;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.CapturePointCenterOfPressureControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLine2d;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFrameLine2d;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;


public class EqConstCoPAndGuideLineCapturePointCoPControlModule implements CapturePointCenterOfPressureControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("EqConstCoPAndGuideLineCapturePointCoPControlModule");

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable minFinalTime = new DoubleYoVariable("minFinalTime", registry);
   private final DoubleYoVariable additionalSingleSupportSwingTime = new DoubleYoVariable("additionalSingleSupportSwingTime", registry);
   
   private final YoFrameLineSegment2d guideLineWorld = new YoFrameLineSegment2d("guideLine", "", world, registry);
   private final YoFrameLine2d parallelLineWorld = new YoFrameLine2d("parallelLine", "", world, registry);

   private final YoFramePoint2d desiredCenterOfPressure = new YoFramePoint2d("desiredCoP", "", world, registry);
   private final DoubleYoVariable kCaptureGuide = new DoubleYoVariable("kCaptureGuide", "ICP distance to guide line --> position of parallel line", registry);


   public EqConstCoPAndGuideLineCapturePointCoPControlModule(CommonHumanoidReferenceFrames referenceFrames, ProcessedSensorsInterface processedSensors,
           CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.processedSensors = processedSensors;
      this.couplingRegistry = couplingRegistry;

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (yoGraphicsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList("Capture Point CoP Control Module");

         YoGraphicPosition centerOfPressureDesiredGraphic = new YoGraphicPosition("Desired Center of Pressure", desiredCenterOfPressure, 0.012,
                                                                    YoAppearance.Gray(), YoGraphicPosition.GraphicType.CROSS);
         yoGraphicsListRegistry.registerYoGraphic("EquivalentConstantCoPVelocityViaCoPControlModule", centerOfPressureDesiredGraphic);
         artifactList.add(centerOfPressureDesiredGraphic.createArtifact());

         YoArtifactLineSegment2d guideLineArtifact = new YoArtifactLineSegment2d("Guide Line", guideLineWorld, Color.RED);
         artifactList.add(guideLineArtifact);

         YoArtifactLine2d parallellLineArtifact = new YoArtifactLine2d("Parallel Line", parallelLineWorld, Color.GREEN);
         artifactList.add(parallellLineArtifact);

         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void controlDoubleSupport(OldBipedSupportPolygons bipedSupportPolygons, FramePoint currentCapturePoint, FramePoint desiredCapturePoint,
                                    FramePoint centerOfMassPositionInWorldFrame, FrameVector2d desiredVelocity, FrameVector2d currentVelocity)
   {
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      double finalTime = minFinalTime.getDoubleValue();
      double comHeight = computeCoMHeightUsingBothFeet();
      computeDesiredCoP(supportPolygon, currentCapturePoint.toFramePoint2d(), desiredCapturePoint.toFramePoint2d(), finalTime, comHeight, null);
   }

   public void controlSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons supportPolygons, FramePoint currentCapturePoint, FrameVector2d desiredVelocity,
                                    FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, FramePoint centerOfMassPositionInZUpFrame,
                                    FrameVector2d currentVelocity)
   {
      FrameConvexPolygon2d supportPolygon = supportPolygons.getFootPolygonInAnkleZUp(supportLeg);
      double finalTime = computeFinalTimeSingleSupport();
      double comHeight = computeCoMHeightUsingOneFoot(supportLeg);
      FramePoint2d desiredFinalCapturePoint;
      if (desiredCapturePoint != null)
         desiredFinalCapturePoint = desiredCapturePoint.toFramePoint2d();
      else
         desiredFinalCapturePoint = guideLine.getSecondEndPointCopy();

      computeDesiredCoP(supportPolygon, currentCapturePoint.toFramePoint2d(), desiredFinalCapturePoint, finalTime, comHeight, guideLine);
   }

   public void packDesiredCenterOfPressure(FramePoint desiredCenterOfPressureToPack)
   {
      double x = desiredCenterOfPressure.getX();
      double y = desiredCenterOfPressure.getY();
      double z = 0.0;

      desiredCenterOfPressureToPack.setIncludingFrame(desiredCenterOfPressure.getReferenceFrame(), x, y, z);
   }

   private void computeDesiredCoP(FrameConvexPolygon2d supportPolygon, FramePoint2d currentCapturePoint2d, FramePoint2d desiredFinalCapturePoint, double finalTime,
                                  double comHeight, FrameLineSegment2d guideLine)
   {
      ReferenceFrame supportPolygonFrame = supportPolygon.getReferenceFrame();
      desiredFinalCapturePoint.changeFrame(supportPolygonFrame);

      double gravity = -processedSensors.getGravityInWorldFrame().getZ();
      FramePoint2d desiredCenterOfPressure = EquivalentConstantCoPCalculator.computeEquivalentConstantCoP(currentCapturePoint2d, desiredFinalCapturePoint,
                                                finalTime, comHeight, gravity);

      if (guideLine != null)
      {
         FrameLineSegment2d guideLineInWorld = new FrameLineSegment2d(guideLine);
         guideLineInWorld.changeFrame(world);
         guideLineWorld.setFrameLineSegment2d(guideLineInWorld);
         guideLine.changeFrame(supportPolygonFrame);
         FrameLine2d shiftedParallelLine = createShiftedParallelLine(guideLine, currentCapturePoint2d);
         shiftedParallelLine.orthogonalProjection(desiredCenterOfPressure);
         movePointInsidePolygon(desiredCenterOfPressure, supportPolygon, shiftedParallelLine);
      }
      else
      {
         FrameLine2d currentToDesired = new FrameLine2d(currentCapturePoint2d, desiredFinalCapturePoint);
         movePointInsidePolygon(desiredCenterOfPressure, supportPolygon, currentToDesired);
         hideLines();
      }

      desiredCenterOfPressure.changeFrame(this.desiredCenterOfPressure.getReferenceFrame());
      this.desiredCenterOfPressure.set(desiredCenterOfPressure);
   }

   private FrameLine2d createShiftedParallelLine(FrameLineSegment2d guideLine, FramePoint2d currentCapturePoint2d)
   {
      FramePoint2d captureProjectedOntoGuideLine = guideLine.orthogonalProjectionCopy(currentCapturePoint2d);

      FrameVector2d projectedToCurrent = new FrameVector2d(captureProjectedOntoGuideLine, currentCapturePoint2d);
      projectedToCurrent.scale(kCaptureGuide.getDoubleValue());

      FramePoint2d shiftedPoint = new FramePoint2d(captureProjectedOntoGuideLine);
      shiftedPoint.add(projectedToCurrent);

      FrameVector2d directionVector = new FrameVector2d();
      guideLine.getFrameVector(directionVector);
      FrameLine2d shiftedParallelLine = new FrameLine2d(shiftedPoint, directionVector);

      FrameLine2d shiftedParallelLineInWorld = new FrameLine2d(shiftedParallelLine);
      shiftedParallelLineInWorld.changeFrame(world);
      parallelLineWorld.setFrameLine2d(shiftedParallelLineInWorld);

      return shiftedParallelLine;
   }

   private void hideLines()
   {
      guideLineWorld.setFrameLineSegment2d(null);
      parallelLineWorld.setFrameLine2d(null);
   }

   private static void movePointInsidePolygon(FramePoint2d point, FrameConvexPolygon2d polygon, FrameLine2d guideLine)
   {
      // If feasible CoP is not inside the convex hull of the feet, project it into it.
      if (!polygon.isPointInside(point))
      {
         // supportPolygon.orthogonalProjection(centerOfPressureDesired2d);

         FramePoint2d[] intersections = polygon.intersectionWith(guideLine);
         if (intersections != null)
         {
            FramePoint2d intersectionToUse;

            if (intersections.length == 2)
            {
               double distanceSquaredToIntersection0 = point.distanceSquared(intersections[0]);
               double distanceSquaredToIntersection1 = point.distanceSquared(intersections[1]);

               if (distanceSquaredToIntersection0 <= distanceSquaredToIntersection1)
                  intersectionToUse = intersections[0];
               else
                  intersectionToUse = intersections[1];


               point.setX(intersectionToUse.getX());
               point.setY(intersectionToUse.getY());

               // Move in a little along the line:
               FrameLineSegment2d guideLineSegment = new FrameLineSegment2d(intersections);
               FrameVector2d frameVector2d = new FrameVector2d();
               guideLineSegment.getFrameVector(frameVector2d);
               frameVector2d.normalize();
               frameVector2d.scale(-0.002);    // Move toward desired capture by 2 mm to prevent some jerky behavior with VTPs..

               point.setX(point.getX() + frameVector2d.getX());
               point.setY(point.getY() + frameVector2d.getY());
            }
            else
            {
               throw new RuntimeException("This is interesting, shouldn't get here.");
            }
         }
         else
         {
            point.set(polygon.getClosestVertexCopy(guideLine));
         }
      }
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

   private double computeFinalTimeSingleSupport()
   {
      double estimatedSwingTimeRemaining = couplingRegistry.getEstimatedSwingTimeRemaining();
      double ret = Math.max(estimatedSwingTimeRemaining, minFinalTime.getDoubleValue());
      ret += additionalSingleSupportSwingTime.getDoubleValue();
      return ret;
   }
   
   public void setParametersForR2()
   {
      minFinalTime.set(0.1);
      kCaptureGuide.set(2.0);
      additionalSingleSupportSwingTime.set(0.1);
   }

   public void setParametersForM2V2()
   {
      minFinalTime.set(0.1);
      kCaptureGuide.set(3.0);
      additionalSingleSupportSwingTime.set(0.3);
   }
}
