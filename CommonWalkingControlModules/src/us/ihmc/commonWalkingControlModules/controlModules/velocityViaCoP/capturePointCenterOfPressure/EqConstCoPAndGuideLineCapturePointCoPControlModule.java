package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.capturePointCenterOfPressure;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.CapturePointCenterOfPressureControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFrameLine2d;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

import com.yobotics.simulationconstructionset.plotting.YoFrameLine2dArtifact;
import com.yobotics.simulationconstructionset.plotting.YoFrameLineSegment2dArtifact;

public class EqConstCoPAndGuideLineCapturePointCoPControlModule implements CapturePointCenterOfPressureControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("EqConstCoPAndGuideLineCapturePointCoPControlModule");

   private final CommonWalkingReferenceFrames referenceFrames;
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable minFinalTime = new DoubleYoVariable("minFinalTime", registry);
   private final DoubleYoVariable additionalSingleSupportSwingTime = new DoubleYoVariable("additionalSingleSupportSwingTime", registry);
   
   private final YoFrameLineSegment2d guideLineWorld = new YoFrameLineSegment2d("guideLine", "", world, registry);
   private final YoFrameLine2d parallelLineWorld = new YoFrameLine2d("parallelLine", "", world, registry);

   private final YoFramePoint2d desiredCenterOfPressure = new YoFramePoint2d("desiredCoP", "", world, registry);
   private final DoubleYoVariable kCaptureGuide = new DoubleYoVariable("kCaptureGuide", "ICP distance to guide line --> position of parallel line", registry);


   public EqConstCoPAndGuideLineCapturePointCoPControlModule(CommonWalkingReferenceFrames referenceFrames, ProcessedSensorsInterface processedSensors,
           CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry, YoGraphicsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.processedSensors = processedSensors;
      this.couplingRegistry = couplingRegistry;

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (dynamicGraphicObjectsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList("Capture Point CoP Control Module");

         YoGraphicPosition centerOfPressureDesiredGraphic = new YoGraphicPosition("Desired Center of Pressure", desiredCenterOfPressure, 0.012,
                                                                    YoAppearance.Gray(), YoGraphicPosition.GraphicType.CROSS);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("EquivalentConstantCoPVelocityViaCoPControlModule", centerOfPressureDesiredGraphic);
         artifactList.add(centerOfPressureDesiredGraphic.createArtifact());

         YoFrameLineSegment2dArtifact guideLineArtifact = new YoFrameLineSegment2dArtifact("Guide Line", guideLineWorld, Color.RED);
         artifactList.add(guideLineArtifact);

         YoFrameLine2dArtifact parallellLineArtifact = new YoFrameLine2dArtifact("Parallel Line", parallelLineWorld, Color.GREEN);
         artifactList.add(parallellLineArtifact);

         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void controlDoubleSupport(BipedSupportPolygons bipedSupportPolygons, FramePoint currentCapturePoint, FramePoint desiredCapturePoint,
                                    FramePoint centerOfMassPositionInWorldFrame, FrameVector2d desiredVelocity, FrameVector2d currentVelocity)
   {
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      double finalTime = minFinalTime.getDoubleValue();
      double comHeight = computeCoMHeightUsingBothFeet();
      computeDesiredCoP(supportPolygon, currentCapturePoint.toFramePoint2d(), desiredCapturePoint.toFramePoint2d(), finalTime, comHeight, null);
   }

   public void controlSingleSupport(RobotSide supportLeg, BipedSupportPolygons supportPolygons, FramePoint currentCapturePoint, FrameVector2d desiredVelocity,
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

      FrameVector2d directionVector = guideLine.getVectorCopy();
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
               FrameVector2d frameVector2d = guideLineSegment.getVectorCopy();
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
