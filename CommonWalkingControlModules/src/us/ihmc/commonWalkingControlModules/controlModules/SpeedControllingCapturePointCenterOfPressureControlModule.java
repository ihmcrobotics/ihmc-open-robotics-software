package us.ihmc.commonWalkingControlModules.controlModules;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
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
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLine2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class SpeedControllingCapturePointCenterOfPressureControlModule implements CapturePointCenterOfPressureControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SpeedControllingCapturePointCenterOfPressureController");

   // Reference frames
   private ReferenceFrame midFeetZUp, world;

   // Points
   private final YoFramePoint centerOfPressureDesiredWorld, centerOfPressureDesiredMidFeet, centerOfPressureDesiredLeftAnkleZUp,
                              centerOfPressureDesiredRightAnkleZUp;

   private final SideDependentList<YoFramePoint> centerOfPressureDesiredAnkleZUp;

   private final YoFrameLine2d cpLine;
   private final YoFrameLine2d comDirectionLine;

   private DoubleYoVariable speedControlXKp = new DoubleYoVariable("speedControlXKp", registry);
   private DoubleYoVariable speedControlYKp = new DoubleYoVariable("speedControlYKp", registry);

   private DoubleYoVariable perimeterDistance = new DoubleYoVariable("supportPolygonPerimeterDistance", registry);
   private DoubleYoVariable minPerimeterDistance = new DoubleYoVariable("minSupportPolygonPerimeterDistance", registry);

   private DoubleYoVariable captureKp = new DoubleYoVariable("captureKp", registry);


   private final DoubleYoVariable alphaDesiredCoP = new DoubleYoVariable("alphaDesiredCoP", registry);
   private final AlphaFilteredYoVariable xDesiredCoP = new AlphaFilteredYoVariable("xDesiredCoP", registry, alphaDesiredCoP);
   private final AlphaFilteredYoVariable yDesiredCoP = new AlphaFilteredYoVariable("yDesiredCoP", registry, alphaDesiredCoP);
   private final BooleanYoVariable lastTickSingleSupport = new BooleanYoVariable("lastTickSingleSupport", registry);

   private final DynamicGraphicPosition centerOfPressureDesiredWorldGraphicPosition;

   private final YoFrameLineSegment2d guideLineWorld;
   private final YoFrameLine2d parallelLineWorld;

   private final DoubleYoVariable kCaptureGuide = new DoubleYoVariable("K_capture_guide", registry);



   public SpeedControllingCapturePointCenterOfPressureControlModule(double controlDT, CommonWalkingReferenceFrames referenceFrames,
           YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      world = ReferenceFrame.getWorldFrame();

      cpLine = new YoFrameLine2d("cpLine", "", world, registry);
      comDirectionLine = new YoFrameLine2d("comDirectionLine", "", world, registry);

      // Points
      centerOfPressureDesiredWorld = new YoFramePoint("spdCopDesWorld", "", world, registry);
      centerOfPressureDesiredMidFeet = new YoFramePoint("spdCopDesMidfeet", "", midFeetZUp, registry);
      centerOfPressureDesiredLeftAnkleZUp = new YoFramePoint("spdCopDesLaZUp", "", referenceFrames.getAnkleZUpReferenceFrames().get(RobotSide.LEFT), registry);
      centerOfPressureDesiredRightAnkleZUp = new YoFramePoint("spdCopDesRaZUp", "", referenceFrames.getAnkleZUpReferenceFrames().get(RobotSide.RIGHT),
              registry);


      centerOfPressureDesiredAnkleZUp = new SideDependentList<YoFramePoint>(centerOfPressureDesiredLeftAnkleZUp, centerOfPressureDesiredRightAnkleZUp);


      alphaDesiredCoP.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(8.84, controlDT));
      lastTickSingleSupport.set(true);

      guideLineWorld = new YoFrameLineSegment2d("guideLine", "", world, registry);
      parallelLineWorld = new YoFrameLine2d("parallelLine", "", world, registry);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObjectsList dynamicGraphicObjectList = new DynamicGraphicObjectsList("CapturePointController");

         centerOfPressureDesiredWorldGraphicPosition = new DynamicGraphicPosition("Desired Center of Pressure", centerOfPressureDesiredWorld, 0.012,
                 YoAppearance.Gray(), DynamicGraphicPosition.GraphicType.CROSS);

         dynamicGraphicObjectList.add(centerOfPressureDesiredWorldGraphicPosition);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectList);

         ArtifactList artifactList = new ArtifactList("Capture Point CoP Control Module");

         artifactList.add(centerOfPressureDesiredWorldGraphicPosition.createArtifact());

         YoFrameLine2dArtifact cpLineArtifact = new YoFrameLine2dArtifact("cp Line", cpLine, Color.RED);
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

      if (parentRegistry != null)    // && (VarListsToRegister.REGISTER_CAPTURE_POINT_CENTER_OF_PRESSURE_CONTROLLER))
      {
         parentRegistry.addChild(registry);
      }

      resetCoPFilter();
   }

   private void setCenterOfPressureDesired(FramePoint2d centerOfPressureDesired)
   {
      centerOfPressureDesiredMidFeet.set(new FramePoint(midFeetZUp, centerOfPressureDesired.getX(), centerOfPressureDesired.getY(), 0.0));
      centerOfPressureDesiredWorld.set(centerOfPressureDesiredMidFeet.getFramePointCopy().changeFrameCopy(world));
   }

   private void resetCoPFilter()
   {
      xDesiredCoP.reset();
      yDesiredCoP.reset();
   }

   private void filterCenterOfPressureDesired(FramePoint2d centerOfPressureDesired)
   {
      xDesiredCoP.update(centerOfPressureDesired.getX());
      yDesiredCoP.update(centerOfPressureDesired.getY());

      centerOfPressureDesired.setX(xDesiredCoP.getDoubleValue());
      centerOfPressureDesired.setY(yDesiredCoP.getDoubleValue());
   }

   private FrameLine2d createSpeedControlLine(FrameVector2d currentVelocity, FrameVector2d desiredVelocity, FramePoint centerOfMassPosition,
           ReferenceFrame currentFrame)
   {
      // Get the referenceFrame of the desired heading
      ReferenceFrame desiredVelocityFrame = desiredVelocity.getReferenceFrame();

      if (desiredVelocityFrame.getName() != "DesiredHeadingFrame")
      {
         // TODO: Get this frame somewhere else, so we don't have check for a name here!
         throw new RuntimeException("desiredVelocity is in the " + desiredVelocityFrame.toString() + " but should be in DesiredHeadingFrame");
      }


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

   private FramePoint2d movePointInsidePolygon(FramePoint2d point, FrameConvexPolygon2d polygon, FrameLine2d guideLine, ReferenceFrame currentFrame)
   {
      FramePoint2d returnPoint = new FramePoint2d(point);

      // If feasible CoP is not inside the convex hull of the feet, project it into it.
      if (!polygon.isPointInside(point))
      {
         // supportPolygon.orthogonalProjection(centerOfPressureDesired2d);

         if (returnPoint.getReferenceFrame() != currentFrame)
         {
            returnPoint = returnPoint.changeFrameCopy(currentFrame);
         }



         FramePoint2d[] intersections = polygon.intersectionWith(guideLine);
         if (intersections != null)
         {
            FramePoint2d intersectionToUse;

            if (intersections.length == 2)
            {
               double distanceSquaredToIntersection0 = returnPoint.distanceSquared(intersections[0]);
               double distanceSquaredToIntersection1 = returnPoint.distanceSquared(intersections[1]);

               if (distanceSquaredToIntersection0 <= distanceSquaredToIntersection1)
                  intersectionToUse = intersections[0];
               else
                  intersectionToUse = intersections[1];


               returnPoint.setX(intersectionToUse.getX());
               returnPoint.setY(intersectionToUse.getY());

               // Move in a little along the line:
               FrameLineSegment2d guideLineSegment = new FrameLineSegment2d(intersections);
               FrameVector2d frameVector2d = guideLineSegment.getVectorCopy();
               frameVector2d.normalize();
               frameVector2d.scale(-0.002);    // Move toward desired capture by 2 mm to prevent some jerky behavior with VTPs..

               returnPoint.setX(returnPoint.getX() + frameVector2d.getX());
               returnPoint.setY(returnPoint.getY() + frameVector2d.getY());
            }
            else
            {
               throw new RuntimeException("This is interesting, shouldn't get here.");
            }
         }
         else
         {
            returnPoint = polygon.getClosestVertexCopy(guideLine);

         }
      }

      return returnPoint;
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
         throw new RuntimeException("Everything has to be in the Z Up frame.");
      }

      // Check if we have a support Polygon
      if (bipedSupportPolygons == null)
      {
         throw new RuntimeException("The support polygon cannot be null.");
      }

      if (lastTickSingleSupport.getBooleanValue())
      {
         resetCoPFilter();
         lastTickSingleSupport.set(false);
      }


      FramePoint2d currentCapturePoint2d = currentCapturePoint.toFramePoint2d();
      FramePoint2d desiredCapturePoint2d = desiredCapturePoint.toFramePoint2d();


      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();

      FrameLineSegment2d closestEdge = supportPolygon.getClosestEdge(currentCapturePoint2d);

      perimeterDistance.set(closestEdge.distance(currentCapturePoint2d));


      // Handle large disturbances where the iCP is (almost) out of the support polygon
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
            cpLine.setFrameLine2d(iCPLine.changeFrameCopy(world));
            filterCenterOfPressureDesired(farthestToDesiredCP);
            setCenterOfPressureDesired(farthestToDesiredCP);

            return;
         }
         else if (perimeterDistance.getDoubleValue() < minPerimeterDistance.getDoubleValue())
         {
            double ratio = (minPerimeterDistance.getDoubleValue() - perimeterDistance.getDoubleValue()) / minPerimeterDistance.getDoubleValue();

            desiredCapturePoint2d.setX((1.0 - ratio) * desiredCapturePoint2d.getX() + ratio * closestToDesiredCP.getX());
            desiredCapturePoint2d.setY((1.0 - ratio) * desiredCapturePoint2d.getY() + ratio * closestToDesiredCP.getY());

         }
      }

      // Create Line from dCP to iCP
      FrameLine2d controlLine = new FrameLine2d(currentCapturePoint2d, desiredCapturePoint2d);
      cpLine.setFrameLine2d(controlLine.changeFrameCopy(world));




      FrameVector2d comDirection = desiredVelocity.changeFrameCopy(midFeetZUp);
      comDirection.normalize();
      FrameVector2d controlDirection = controlLine.getNormalizedFrameVector();


      FramePoint2d centerOfPressureDesired = null;

      // If the scalar projection of the desired CoM direction on the desired iCP direction is negative
      // control only the iCP position and don't do speed control.
      if (comDirection.dot(controlDirection) < 0.0 || (desiredVelocity.length() == 0.0))
      {
         double distance = desiredCapturePoint2d.distance(currentCapturePoint2d);
         centerOfPressureDesired = new FramePoint2d(desiredCapturePoint2d);
         controlDirection.scale(captureKp.getDoubleValue() * distance);
         centerOfPressureDesired.sub(controlDirection);
      }
      else
      {
         FrameLine2d massLine = createSpeedControlLine(currentVelocity, desiredVelocity, centerOfMassPositionInZUpFrame, midFeetZUp);
         centerOfPressureDesired = controlLine.intersectionWith(massLine);
      }

      // Filter center of pressure for more robustness
      filterCenterOfPressureDesired(centerOfPressureDesired);

      // FrameLineSegment2d desiredCaptureToDesiredCop = new FrameLineSegment2d(desiredCapturePoint2d, centerOfPressureDesired);
      centerOfPressureDesired = movePointInsidePolygon(centerOfPressureDesired, supportPolygon, controlLine, midFeetZUp);


      setCenterOfPressureDesired(centerOfPressureDesired);




   }

   public void controlSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, RobotSide supportLeg,
           ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons, FramePoint centerOfMassPositionInZUpFrame, FrameVector2d desiredVelocity,
           FrameVector2d currentVelocity)
   {
      if (!lastTickSingleSupport.getBooleanValue())
      {
         resetCoPFilter();
         lastTickSingleSupport.set(true);
      }


      // Disable double support stuff
      comDirectionLine.setFrameLine2d(null);
      cpLine.setFrameLine2d(null);

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

      if (guideLine == null)
      {
         throw new RuntimeException("Needs a guideline");
      }

      // Plot stuff
      guideLineWorld.setFrameLineSegment2d(guideLine.changeFrameCopy(world));


      FramePoint2d currentCapturePoint2d = currentCapturePoint.toFramePoint2d();

      // Foot stuff:
      FrameConvexPolygon2d footPolygon = supportPolygons.getFootPolygonInAnkleZUp(supportLeg);


      // Create parallell line
      FramePoint2d captureProjectedOntoGuideLine = guideLine.orthogonalProjectionCopy(currentCapturePoint2d);

      FrameVector2d projectedToCurrent = new FrameVector2d(captureProjectedOntoGuideLine, currentCapturePoint2d);
      projectedToCurrent.scale(kCaptureGuide.getDoubleValue());

      FramePoint2d shiftedPoint = new FramePoint2d(captureProjectedOntoGuideLine);
      shiftedPoint.add(projectedToCurrent);

      FrameVector2d frameVector2d = guideLine.getVectorCopy();
      FrameLine2d shiftedParallelLine = new FrameLine2d(shiftedPoint, frameVector2d);

      parallelLineWorld.setFrameLine2d(shiftedParallelLine.changeFrameCopy(world));

      // Create speed control line
      FrameLine2d massLine = createSpeedControlLine(currentVelocity, desiredVelocity, centerOfMassPositionInZUpFrame, referenceFrame);
      FramePoint2d centerOfPressureDesired = shiftedParallelLine.intersectionWith(massLine);

      centerOfPressureDesired = movePointInsidePolygon(centerOfPressureDesired, footPolygon, shiftedParallelLine, referenceFrame);

      filterCenterOfPressureDesired(centerOfPressureDesired);

      centerOfPressureDesiredAnkleZUp.get(supportLeg).set(centerOfPressureDesired.getX(), centerOfPressureDesired.getY(), 0.0);
      centerOfPressureDesiredWorld.set(centerOfPressureDesiredAnkleZUp.get(supportLeg).getFramePointCopy().changeFrameCopy(world));


   }

   public void controlSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, RobotSide supportLeg,
           ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons)
   {
      throw new RuntimeException("Needs center of mass data");

   }

   public YoFramePoint getCenterOfPressureDesiredMidFeet()
   {
      return centerOfPressureDesiredMidFeet;
   }

   public YoFramePoint getCenterOfPressureDesiredLeftAnkleZUp()
   {
      return centerOfPressureDesiredLeftAnkleZUp;
   }

   public YoFramePoint getCenterOfPressureDesiredRightAnkleZUp()
   {
      return centerOfPressureDesiredRightAnkleZUp;
   }

   public YoFramePoint getCenterOfPressureDesiredAnkleZUp(RobotSide robotSide)
   {
      return centerOfPressureDesiredAnkleZUp.get(robotSide);
   }

   public void setParametersForR2()
   {
      speedControlXKp.set(3.0);
      speedControlYKp.set(0.0);
      captureKp.set(2.0); //6.0);
      kCaptureGuide.set(2.0);
      minPerimeterDistance.set(0.02);
   }
}
