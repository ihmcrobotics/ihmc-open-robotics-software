package us.ihmc.commonWalkingControlModules.controlModules.capturePointCenterOfPressure;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.CapturePointCenterOfPressureControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.YoFrameLine2dArtifact;
import com.yobotics.simulationconstructionset.plotting.YoFrameLineSegment2dArtifact;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLine2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;

public class SpeedControllingCapturePointCenterOfPressureControlModule implements CapturePointCenterOfPressureControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SpeedControllingCapturePointCenterOfPressureController");

   // Reference frames
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame midFeetZUp, desiredHeadingFrame;

   private DoubleYoVariable speedControlXKp = new DoubleYoVariable("speedControlXKp", registry);
   private DoubleYoVariable speedControlYKp = new DoubleYoVariable("speedControlYKp", registry);

   private DoubleYoVariable perimeterDistance = new DoubleYoVariable("supportPolygonPerimeterDistance", registry);
   private DoubleYoVariable minPerimeterDistance = new DoubleYoVariable("minSupportPolygonPerimeterDistance", registry);

   private DoubleYoVariable doubleSupportCaptureKp = new DoubleYoVariable("doubleSupportCaptureKp", registry);

   private final DoubleYoVariable alphaDesiredCoP = new DoubleYoVariable("alphaDesiredCoP", registry);
   private final AlphaFilteredYoFramePoint2d filteredDesiredCenterOfPressure = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d("desiredCenterOfPressure", "",
         registry, alphaDesiredCoP, ReferenceFrame.getWorldFrame());

   private final BooleanYoVariable lastTickSingleSupport = new BooleanYoVariable("lastTickSingleSupport", registry);

   private final DynamicGraphicPosition centerOfPressureDesiredWorldGraphicPosition;

   private final YoFrameLine2d capturePointLine = new YoFrameLine2d("capturePointLine", "", world, registry);
   private final YoFrameLine2d comDirectionLine = new YoFrameLine2d("comDirectionLine", "", world, registry);
   private final YoFrameLineSegment2d guideLineWorld = new YoFrameLineSegment2d("guideLine", "", world, registry);
   private final YoFrameLine2d parallelLineWorld = new YoFrameLine2d("parallelLine", "", world, registry);

   private final DoubleYoVariable kCaptureGuide = new DoubleYoVariable("kCaptureGuide", "ICP distance to guide line --> position of parallel line", registry);



   public SpeedControllingCapturePointCenterOfPressureControlModule(double controlDT, CommonWalkingReferenceFrames referenceFrames,
           YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, ReferenceFrame desiredHeadingFrame)
   {
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      this.desiredHeadingFrame = desiredHeadingFrame;

      alphaDesiredCoP.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(8.84, controlDT));
      lastTickSingleSupport.set(true);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObjectsList dynamicGraphicObjectList = new DynamicGraphicObjectsList("CapturePointController");

         centerOfPressureDesiredWorldGraphicPosition = new DynamicGraphicPosition("Desired Center of Pressure", filteredDesiredCenterOfPressure, 0.012,
                 YoAppearance.Gray(), DynamicGraphicPosition.GraphicType.CROSS);

         dynamicGraphicObjectList.add(centerOfPressureDesiredWorldGraphicPosition);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectList);

         ArtifactList artifactList = new ArtifactList("Capture Point CoP Control Module");

         artifactList.add(centerOfPressureDesiredWorldGraphicPosition.createArtifact());

         YoFrameLine2dArtifact cpLineArtifact = new YoFrameLine2dArtifact("CP Line", capturePointLine, Color.RED);
         artifactList.add(cpLineArtifact);


         YoFrameLine2dArtifact comDirectionArtifact = new YoFrameLine2dArtifact("comDirectionLine", comDirectionLine, Color.BLUE);
         artifactList.add(comDirectionArtifact);

         YoFrameLineSegment2dArtifact guideLineArtifact = new YoFrameLineSegment2dArtifact("Guide Line", guideLineWorld, Color.RED);
         artifactList.add(guideLineArtifact);

         YoFrameLine2dArtifact parallellLineArtifact = new YoFrameLine2dArtifact("Parallel Line", parallelLineWorld, Color.GREEN);
         artifactList.add(parallellLineArtifact);

         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }
      else
      {
         centerOfPressureDesiredWorldGraphicPosition = null;
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      resetCoPFilter();
   }

   private void resetCoPFilter()
   {
      filteredDesiredCenterOfPressure.reset();
   }

   private FrameLine2d createSpeedControlLine(FrameVector2d currentVelocity, FrameVector2d desiredVelocity, FramePoint centerOfMassPosition,
           ReferenceFrame currentFrame)
   {
      ReferenceFrame desiredVelocityFrame = desiredVelocity.getReferenceFrame();
      desiredVelocityFrame.checkReferenceFrameMatch(desiredHeadingFrame);
      FrameVector2d currentVelocityInFrame = currentVelocity.changeFrameCopy(desiredVelocityFrame);
      FramePoint2d centerOfMassPositionInFrame = centerOfMassPosition.changeFrameCopy(desiredVelocityFrame).toFramePoint2d();

      // Project CoM on control line
      FrameVector2d velocityT = new FrameVector2d(desiredVelocity);
      velocityT.setX(desiredVelocity.getY());
      velocityT.setY(desiredVelocity.getX());


      FramePoint2d speedControlPosition = new FramePoint2d(centerOfMassPositionInFrame);

      // Speed controller: Only increase speed for now
      speedControlPosition.setX(speedControlPosition.getX()
                                + speedControlXKp.getDoubleValue() * Math.min(0.0, currentVelocityInFrame.getX() - desiredVelocity.getX()));
      speedControlPosition.setY(speedControlPosition.getY()
                                + speedControlYKp.getDoubleValue() * Math.min(0.0, currentVelocityInFrame.getY() - desiredVelocity.getY()));


      if (velocityT.length() == 0.0)
         throw new RuntimeException("Not sure what to do when velocity is zero");
      
      FrameLine2d massLine = new FrameLine2d(speedControlPosition, velocityT);
      comDirectionLine.setFrameLine2d(massLine.changeFrameCopy(world));
  
      return massLine.changeFrameCopy(currentFrame);
   }

   public void controlDoubleSupport(BipedSupportPolygons bipedSupportPolygons, FramePoint currentCapturePoint, FramePoint desiredCapturePoint,
           FramePoint centerOfMassPositionInZUpFrame, FrameVector2d desiredVelocity, FrameVector2d currentVelocity)
   {
      guideLineWorld.setFrameLineSegment2d(null);
      parallelLineWorld.setFrameLine2d(null);


      // Check if everything is in the correct coordinate frame
      boolean everythingInZUpFrame = desiredCapturePoint.getReferenceFrame().isZupFrame() && currentCapturePoint.getReferenceFrame().isZupFrame()
            && centerOfMassPositionInZUpFrame.getReferenceFrame().isZupFrame() && desiredVelocity.getReferenceFrame().isZupFrame()
            && currentVelocity.getReferenceFrame().isZupFrame();
      if (!everythingInZUpFrame)
      {
         throw new RuntimeException("Everything has to be in a Z Up frame.");
      }

      // Check if we have a support Polygon
      if (bipedSupportPolygons == null)
      {
         throw new RuntimeException("The support polygon cannot be null.");
      }

//      if (lastTickSingleSupport.getBooleanValue())
//      {
//         resetCoPFilter();
//         lastTickSingleSupport.set(false);
//      }


      FramePoint2d currentCapturePoint2d = currentCapturePoint.toFramePoint2d();
      FramePoint2d desiredCapturePoint2d = desiredCapturePoint.toFramePoint2d();


      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();

      FrameLineSegment2d closestEdge = supportPolygon.getClosestEdge(currentCapturePoint2d);

      perimeterDistance.set(closestEdge.distance(currentCapturePoint2d));


      // Handle large disturbances where the iCP is (almost) outside the support polygon
      if (!supportPolygon.isPointInside(currentCapturePoint2d) || (perimeterDistance.getDoubleValue() < minPerimeterDistance.getDoubleValue()))
      {
         FramePoint2d p1 = closestEdge.getFirstEndPointCopy();
         FramePoint2d p2 = closestEdge.getSecondEndPointCopy();

         FramePoint2d closestToDesiredCP = null;
         FramePoint2d farthestToDesiredCP = null;

         if (p1.distance(desiredCapturePoint2d) < p2.distance(desiredCapturePoint2d))
         {
            closestToDesiredCP = p1;
            farthestToDesiredCP = p2;
         }
         else
         {
            closestToDesiredCP = p2;
            farthestToDesiredCP = p1;
         }

         if (!supportPolygon.isPointInside(currentCapturePoint2d))
         {
            FrameLine2d iCPLine = new FrameLine2d(currentCapturePoint2d, farthestToDesiredCP);
            capturePointLine.setFrameLine2d(iCPLine.changeFrameCopy(world));
            farthestToDesiredCP.changeFrame(world);
            this.filteredDesiredCenterOfPressure.update(farthestToDesiredCP);

            return;
         }
         else if (perimeterDistance.getDoubleValue() < minPerimeterDistance.getDoubleValue())
         {
            double ratio = (minPerimeterDistance.getDoubleValue() - perimeterDistance.getDoubleValue()) / minPerimeterDistance.getDoubleValue();

            desiredCapturePoint2d.setX((1.0 - ratio) * desiredCapturePoint2d.getX() + ratio * closestToDesiredCP.getX());
            desiredCapturePoint2d.setY((1.0 - ratio) * desiredCapturePoint2d.getY() + ratio * closestToDesiredCP.getY());

         }
      }
      
      FramePoint2d centerOfPressureDesired = null;
      if (desiredVelocity.length() == 0.0)
      {
         centerOfPressureDesired = doProportionalControl(currentCapturePoint2d, desiredCapturePoint2d);
         capturePointLine.setFrameLine2d(null);
         centerOfPressureDesired.changeFrame(supportPolygon.getReferenceFrame());
         supportPolygon.orthogonalProjection(centerOfPressureDesired);
      }
      else
      {
         // Create Line from dCP to iCP
         FrameLine2d controlLine = new FrameLine2d(currentCapturePoint2d, desiredCapturePoint2d);
         capturePointLine.setFrameLine2d(controlLine.changeFrameCopy(world));


         FrameVector2d comDirection = desiredVelocity.changeFrameCopy(midFeetZUp);
         comDirection.normalize();
         FrameVector2d controlDirection = controlLine.getNormalizedFrameVector();
         
         // If the scalar projection of the desired CoM direction on the desired iCP direction is negative
         // control only the iCP position and don't do speed control.
         if (comDirection.dot(controlDirection) < 0.0)
         {
            centerOfPressureDesired = doProportionalControl(currentCapturePoint2d, desiredCapturePoint2d);
         }
         else
         {
            FrameLine2d massLine = createSpeedControlLine(currentVelocity, desiredVelocity, centerOfMassPositionInZUpFrame, midFeetZUp);
            centerOfPressureDesired = controlLine.intersectionWith(massLine);
         }
         centerOfPressureDesired.changeFrame(supportPolygon.getReferenceFrame());
         movePointInsidePolygonAlongLine(centerOfPressureDesired, supportPolygon, controlLine);
      }

      centerOfPressureDesired.changeFrame(world);
      this.filteredDesiredCenterOfPressure.update(centerOfPressureDesired);
   }

   private FramePoint2d doProportionalControl(FramePoint2d currentCapturePoint2d, FramePoint2d desiredCapturePoint2d)
   {
      FramePoint2d centerOfPressureDesired;
      centerOfPressureDesired = new FramePoint2d(desiredCapturePoint2d);
      FrameVector2d control = new FrameVector2d(currentCapturePoint2d);
      control.sub(desiredCapturePoint2d);
      control.scale(doubleSupportCaptureKp.getDoubleValue());
      centerOfPressureDesired.add(control);
      return centerOfPressureDesired;
   }

   public void controlSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, RobotSide supportLeg,
           ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons, FramePoint centerOfMassPositionInZUpFrame, FrameVector2d desiredVelocity,
           FrameVector2d currentVelocity)
   {
//      if (!lastTickSingleSupport.getBooleanValue())
//      {
//         resetCoPFilter();
//         lastTickSingleSupport.set(true);
//      }

      // Disable double support stuff
      comDirectionLine.setFrameLine2d(null);
      capturePointLine.setFrameLine2d(null);

      // Validate input
      // Check if everything is in the correct coordinate frame
      if (!(currentCapturePoint.getReferenceFrame().isZupFrame() && centerOfMassPositionInZUpFrame.getReferenceFrame().isZupFrame()
            && desiredVelocity.getReferenceFrame().isZupFrame() && currentVelocity.getReferenceFrame().isZupFrame()))
      {
         throw new RuntimeException("Everything has to be in the Z Up frame.");
      }

      // Check if we have a support Polygon
      if (supportPolygons == null)
      {
         throw new RuntimeException("The support polygon cannot be null.");
      }


      FrameConvexPolygon2d footPolygon = supportPolygons.getFootPolygonInAnkleZUp(supportLeg);
      FramePoint2d currentCapturePoint2d = currentCapturePoint.toFramePoint2d();
      
      double epsilon = 1e-9;
      boolean stayInDoubleSupport = (desiredVelocity.lengthSquared() < epsilon);
      FramePoint2d desiredCenterOfPressure;
      if (stayInDoubleSupport)
      {
         FramePoint2d desiredCapturePoint2d = desiredCapturePoint.toFramePoint2d();
         desiredCenterOfPressure = new FramePoint2d(desiredCapturePoint2d);
         FrameVector2d control = new FrameVector2d(currentCapturePoint2d);
         control.sub(desiredCapturePoint2d);
         control.scale(doubleSupportCaptureKp.getDoubleValue());
         desiredCenterOfPressure.add(control);
         footPolygon.orthogonalProjection(desiredCenterOfPressure);
      }
      else
      {
         // Create parallel line
         FramePoint2d captureProjectedOntoGuideLine = guideLine.orthogonalProjectionCopy(currentCapturePoint2d);
         
         FrameVector2d projectedToCurrent = new FrameVector2d(captureProjectedOntoGuideLine, currentCapturePoint2d);
         projectedToCurrent.scale(kCaptureGuide.getDoubleValue());

         FramePoint2d shiftedPoint = new FramePoint2d(captureProjectedOntoGuideLine);
         shiftedPoint.add(projectedToCurrent);

         FrameVector2d frameVector2d = guideLine.getVectorCopy();
         FrameLine2d shiftedParallelLine = new FrameLine2d(shiftedPoint, frameVector2d);


         // Create speed control line
         FrameLine2d massLine = createSpeedControlLine(currentVelocity, desiredVelocity, centerOfMassPositionInZUpFrame, referenceFrame);
         desiredCenterOfPressure = shiftedParallelLine.intersectionWith(massLine);

         desiredCenterOfPressure.changeFrame(footPolygon.getReferenceFrame());
         
         // Plot stuff
         guideLineWorld.setFrameLineSegment2d(guideLine.changeFrameCopy(world));      
         parallelLineWorld.setFrameLine2d(shiftedParallelLine.changeFrameCopy(world));
         
         // FrameLineSegment2d desiredCaptureToDesiredCop = new FrameLineSegment2d(desiredCapturePoint2d, centerOfPressureDesired);

         movePointInsidePolygonAlongLine(desiredCenterOfPressure, footPolygon, shiftedParallelLine);
      }
      desiredCenterOfPressure.changeFrame(world);
      this.filteredDesiredCenterOfPressure.update(desiredCenterOfPressure);
   }

   public void packDesiredCenterOfPressure(FramePoint desiredCenterOfPressureToPack)
   {
      double x = filteredDesiredCenterOfPressure.getX();
      double y = filteredDesiredCenterOfPressure.getY();
      double z = 0.0;

      desiredCenterOfPressureToPack.set(filteredDesiredCenterOfPressure.getReferenceFrame(), x, y, z);
   }

   public void setParametersForR2()
   {
      speedControlXKp.set(3.0);
      speedControlYKp.set(0.0);
      doubleSupportCaptureKp.set(4.0); // 2.0); //6.0);
      kCaptureGuide.set(1.5); // 2.0);
      minPerimeterDistance.set(0.04); // 0.02);
   }
   
   public void setParametersForM2V2()
   {
      speedControlXKp.set(0.5);
      speedControlYKp.set(0.0);
      doubleSupportCaptureKp.set(3.5); // 2.0); //6.0);
      kCaptureGuide.set(2.0);
      minPerimeterDistance.set(0.02);
   }
   
   private static void movePointInsidePolygonAlongLine(FramePoint2d point, FrameConvexPolygon2d polygon, FrameLine2d guideLine)
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
}
