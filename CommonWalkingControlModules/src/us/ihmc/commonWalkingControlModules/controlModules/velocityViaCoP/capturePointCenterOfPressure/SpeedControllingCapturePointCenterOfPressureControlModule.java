package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.capturePointCenterOfPressure;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.CapturePointCenterOfPressureControlModule;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLine2d;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFrameLine2d;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;


public class SpeedControllingCapturePointCenterOfPressureControlModule implements CapturePointCenterOfPressureControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SpeedControllingCapturePointCenterOfPressureController");

   // Reference frames
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame midFeetZUp, desiredHeadingFrame;

   private final DoubleYoVariable speedControlXKp = new DoubleYoVariable("speedControlXKp", registry);
   private final DoubleYoVariable speedControlYKp = new DoubleYoVariable("speedControlYKp", registry);

   private final DoubleYoVariable perimeterDistance = new DoubleYoVariable("supportPolygonPerimeterDistance", registry);
   private final DoubleYoVariable minPerimeterDistance = new DoubleYoVariable("minSupportPolygonPerimeterDistance", registry);

   private final DoubleYoVariable doubleSupportCaptureKp = new DoubleYoVariable("doubleSupportCaptureKp", registry);
   private final DoubleYoVariable singleSupportCaptureKp = new DoubleYoVariable("singleSupportCaptureKp", registry);

   private final YoFrameLine2d capturePointLine = new YoFrameLine2d("capturePointLine", "", world, registry);
   private final YoFrameLine2d comSpeedControllingLine = new YoFrameLine2d("comSpeedControllingLine", "", world, registry);
   private final YoFrameLineSegment2d guideLineWorld = new YoFrameLineSegment2d("guideLine", "", world, registry);
   private final YoFrameLine2d parallelLineWorld = new YoFrameLine2d("parallelLine", "", world, registry);
   private final YoFramePoint2d desiredCoP = new YoFramePoint2d("desiredCoP", "", world, registry);

   private final DoubleYoVariable kCaptureGuide = new DoubleYoVariable("kCaptureGuide", "ICP distance to guide line --> position of parallel line", registry);

   private DoubleYoVariable resizeFootPolygonBy = new DoubleYoVariable("resizeFootPolygonBy", registry);

   private final ConvexPolygonShrinker shrinker = new ConvexPolygonShrinker(); 

   //TODO: 110523: Clean this up and make it better. ComVelocity control line is still hackish.

   public SpeedControllingCapturePointCenterOfPressureControlModule(double controlDT, CommonHumanoidReferenceFrames referenceFrames,
           YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, ReferenceFrame desiredHeadingFrame)
   {
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      this.desiredHeadingFrame = desiredHeadingFrame;

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicsList yoGraphicList = new YoGraphicsList("CapturePointController");
         ArtifactList artifactList = new ArtifactList("Capture Point CoP Control Module");

         YoArtifactLine2d cpLineArtifact = new YoArtifactLine2d("CP Line", capturePointLine, Color.MAGENTA);
         artifactList.add(cpLineArtifact);

         YoArtifactLine2d comSpeedControllingArtifact = new YoArtifactLine2d("comSpeedControllingLine", comSpeedControllingLine, Color.BLUE);
         artifactList.add(comSpeedControllingArtifact);

         YoArtifactLineSegment2d guideLineArtifact = new YoArtifactLineSegment2d("Guide Line", guideLineWorld, Color.RED);
         artifactList.add(guideLineArtifact);

         YoArtifactLine2d parallellLineArtifact = new YoArtifactLine2d("Parallel Line", parallelLineWorld, Color.GREEN);
         artifactList.add(parallellLineArtifact);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   private FrameLine2d createSpeedControlLine(FrameLine2d guideLine, FrameVector2d currentVelocity, FrameVector2d desiredVelocity, FramePoint centerOfMassPosition)
   {
      double desiredVelocityMagnitude = desiredVelocity.length();
      
//      desiredVelocity = desiredVelocity.changeFrameCopy(desiredHeadingFrame);
//      ReferenceFrame desiredVelocityFrame = desiredVelocity.getReferenceFrame();
//      desiredVelocityFrame.checkReferenceFrameMatch(desiredHeadingFrame);
      
      FrameVector2d guideLineUnitVector = new FrameVector2d();
      guideLine.getNormalizedFrameVector(guideLineUnitVector);
      
      FrameVector2d currentVelocityInFrame = new FrameVector2d(currentVelocity);
      currentVelocityInFrame.changeFrame(guideLineUnitVector.getReferenceFrame());
      
      double currentVelocityProjectedIntoGuideLine = currentVelocityInFrame.dot(guideLineUnitVector);
      double velocityError = desiredVelocityMagnitude - currentVelocityProjectedIntoGuideLine;
      
      FrameVector2d controlOffset = new FrameVector2d(guideLineUnitVector);
      controlOffset.scale(-speedControlXKp.getDoubleValue() * velocityError);
      
      
      FramePoint centerOfMassPositionInFrame = new FramePoint(centerOfMassPosition);
      centerOfMassPositionInFrame.changeFrame(controlOffset.getReferenceFrame());
      FramePoint2d centerOfMassPosition2dInFrame = centerOfMassPositionInFrame.toFramePoint2d();

      // Project CoM on control line
      FrameVector2d velocityT = new FrameVector2d(guideLineUnitVector.getReferenceFrame());
      velocityT.setX(-guideLineUnitVector.getY());
      velocityT.setY(guideLineUnitVector.getX());


      FramePoint2d speedControlPosition = new FramePoint2d(centerOfMassPosition2dInFrame);
      speedControlPosition.add(controlOffset);
      
//      // Speed controller: Only increase speed for now
//      speedControlPosition.setX(speedControlPosition.getX()
//                                + speedControlXKp.getDoubleValue() * Math.min(0.0, currentVelocityInFrame.getX() - desiredVelocity.getX()));
//      speedControlPosition.setY(speedControlPosition.getY()
//                                + speedControlYKp.getDoubleValue() * Math.min(0.0, currentVelocityInFrame.getY() - desiredVelocity.getY()));


      if (velocityT.length() == 0.0)
         throw new RuntimeException("Not sure what to do when velocity is zero");
      
      FrameLine2d massLine = new FrameLine2d(speedControlPosition, velocityT);
      FrameLine2d massLineInWorld = new FrameLine2d(massLine);
      massLineInWorld.changeFrame(world);
      comSpeedControllingLine.setFrameLine2d(massLineInWorld);
  
      return massLine;
   }
   
   
   public void controlDoubleSupport(OldBipedSupportPolygons bipedSupportPolygons, FramePoint currentCapturePoint, FramePoint desiredCapturePoint,
           FramePoint centerOfMassPositionInZUpFrame, FrameVector2d desiredVelocity, FrameVector2d currentVelocity)
   {
      guideLineWorld.setFrameLineSegment2d(null);
      parallelLineWorld.setFrameLine2d(null);


      // Check if everything is in the correct coordinate frame
      boolean everythingInZUpFrame = desiredCapturePoint.getReferenceFrame().isZupFrame() && currentCapturePoint.getReferenceFrame().isZupFrame();
      
      if (centerOfMassPositionInZUpFrame != null && !centerOfMassPositionInZUpFrame.getReferenceFrame().isZupFrame()) everythingInZUpFrame = false;
      if (desiredVelocity != null && !desiredVelocity.getReferenceFrame().isZupFrame()) everythingInZUpFrame = false;
      if (currentVelocity != null && !currentVelocity.getReferenceFrame().isZupFrame()) everythingInZUpFrame = false;
      
      if (!everythingInZUpFrame)
      {
         throw new RuntimeException("Everything has to be in a Z Up frame.");
      }

      // Check if we have a support Polygon
      if (bipedSupportPolygons == null)
      {
         throw new RuntimeException("The support polygon cannot be null.");
      }


      FramePoint2d currentCapturePoint2d = currentCapturePoint.toFramePoint2d();
      FramePoint2d desiredCapturePoint2d = desiredCapturePoint.toFramePoint2d();


      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();

      FrameLineSegment2d closestEdge = supportPolygon.getClosestEdgeCopy(currentCapturePoint2d);

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
            // take the farthest vertex and move it inside a little:
            FrameVector2d delta = new FrameVector2d(farthestToDesiredCP);
            delta.sub(supportPolygon.getCentroidCopy());
            delta.scale(1e-4);
            farthestToDesiredCP.sub(delta);

            FrameLine2d iCPLine = new FrameLine2d(currentCapturePoint2d, farthestToDesiredCP);
            FrameLine2d icpLineInWorld = new FrameLine2d(iCPLine);
            icpLineInWorld.changeFrame(world);
            capturePointLine.setFrameLine2d(icpLineInWorld);
            farthestToDesiredCP.changeFrame(world);
            
            FramePoint2d farthestToDesiredCPInMidFeet = new FramePoint2d(farthestToDesiredCP);
            farthestToDesiredCPInMidFeet.changeFrame(midFeetZUp);
            
            if (!supportPolygon.isPointInside(farthestToDesiredCPInMidFeet)) // better than doing this before, because the changeFrameCopy might move it slightly outside the support polygon
               throw new RuntimeException("!supportPolygon.isPointInside(farthestToDesiredCP)");

            this.desiredCoP.set(farthestToDesiredCP);

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
      
      
      FrameConvexPolygon2d somewhatSmallerPolygon = new FrameConvexPolygon2d();
      shrinker.shrinkConstantDistanceInto(supportPolygon, 0.004, somewhatSmallerPolygon);
      
      if ((desiredVelocity == null) || (desiredVelocity.length() == 0.0))
      {
         centerOfPressureDesired = doProportionalControl(currentCapturePoint2d, desiredCapturePoint2d);
         capturePointLine.setFrameLine2d(null);
         centerOfPressureDesired.changeFrame(supportPolygon.getReferenceFrame());
         
         somewhatSmallerPolygon.orthogonalProjection(centerOfPressureDesired);
         
         
         
      }
      else
      {
         // Create Line from desired Capture Point to instantaneous Capture Point
         FrameLine2d controlLine = new FrameLine2d(currentCapturePoint2d, desiredCapturePoint2d);
         FrameLine2d controlLineInWorld = new FrameLine2d(controlLine);
         controlLineInWorld.changeFrame(world);
         capturePointLine.setFrameLine2d(controlLineInWorld);


         FrameVector2d comDirection = new FrameVector2d(desiredVelocity);
         comDirection.changeFrame(midFeetZUp);
         comDirection.normalize();
         FrameVector2d controlDirection = new FrameVector2d();
         controlLine.getNormalizedFrameVector(controlDirection);
         
         // If the scalar projection of the desired CoM direction on the desired iCP direction is negative
         // control only the iCP position and don't do speed control.
         if (comDirection.dot(controlDirection) < 0.0)
         {
            centerOfPressureDesired = doProportionalControl(currentCapturePoint2d, desiredCapturePoint2d);
         }
         else
         {
            FrameLine2d massLine = createSpeedControlLine(controlLine, currentVelocity, desiredVelocity, centerOfMassPositionInZUpFrame);
            
            if (massLine == null) 
            {
               throw new RuntimeException("massLine == null");
            }
                         
            centerOfPressureDesired = controlLine.intersectionWith(massLine);
         }
         centerOfPressureDesired.changeFrame(supportPolygon.getReferenceFrame());
         GeometryTools.movePointInsidePolygonAlongLine(centerOfPressureDesired, supportPolygon, new FrameLine2d(controlLine));
      }
      
      if (!supportPolygon.isPointInside(centerOfPressureDesired))
         throw new RuntimeException("!supportPolygon.isPointInside(centerOfPressureDesired)");

      centerOfPressureDesired.changeFrame(world);
      this.desiredCoP.set(centerOfPressureDesired);
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

   public final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();
   
   public void controlSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons supportPolygons, FramePoint currentCapturePoint, FrameVector2d desiredVelocity,
           FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, FramePoint centerOfMassPositionInZUpFrame, FrameVector2d currentVelocity)
   {
      // Disable double support stuff
      comSpeedControllingLine.setFrameLine2d(null);
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


      shrinker.shrinkConstantDistanceInto(supportPolygons.getFootPolygonInAnkleZUp(supportLeg), resizeFootPolygonBy.getDoubleValue(), footPolygon);
      
      
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
         control.scale(singleSupportCaptureKp.getDoubleValue());
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

         FrameVector2d frameVector2d = new FrameVector2d();
         guideLine.getFrameVector(frameVector2d);
         FrameLine2d shiftedParallelLine = new FrameLine2d(shiftedPoint, frameVector2d);


         // Create speed control line
         FrameLine2d massLine = createSpeedControlLine(new FrameLine2d(guideLine), currentVelocity, desiredVelocity, centerOfMassPositionInZUpFrame);
         massLine.changeFrame(shiftedParallelLine.getReferenceFrame());
         desiredCenterOfPressure = shiftedParallelLine.intersectionWith(massLine);

         desiredCenterOfPressure.changeFrame(footPolygon.getReferenceFrame());
         
         // Plot stuff
         FrameLineSegment2d guideLineInWorld = new FrameLineSegment2d(guideLine);
         guideLineInWorld.changeFrame(world);
         guideLineWorld.setFrameLineSegment2d(guideLineInWorld);      
         FrameLine2d shiftedParallelLineInWorld = new FrameLine2d(shiftedParallelLine);
         shiftedParallelLineInWorld.changeFrame(world);
         parallelLineWorld.setFrameLine2d(shiftedParallelLineInWorld);
         
         // FrameLineSegment2d desiredCaptureToDesiredCop = new FrameLineSegment2d(desiredCapturePoint2d, centerOfPressureDesired);

         GeometryTools.movePointInsidePolygonAlongLine(desiredCenterOfPressure, footPolygon, shiftedParallelLine);
      }
      desiredCenterOfPressure.changeFrame(world);
      this.desiredCoP.set(desiredCenterOfPressure);
   }

   public void packDesiredCenterOfPressure(FramePoint desiredCenterOfPressureToPack)
   {      
      double x = desiredCoP.getX();
      double y = desiredCoP.getY();
      double z = 0.0;

      desiredCenterOfPressureToPack.setIncludingFrame(desiredCoP.getReferenceFrame(), x, y, z);
   }

   public void setParametersForR2()
   {
      speedControlXKp.set(3.0);
      speedControlYKp.set(0.0);
      doubleSupportCaptureKp.set(4.0); // 2.0); //6.0);
      singleSupportCaptureKp.set(2.5); 
      kCaptureGuide.set(1.5); // 2.0);
      minPerimeterDistance.set(0.04); // 0.02);
   }
   
   public void setParametersForM2V2()
   {
      speedControlXKp.set(0.5);
      speedControlYKp.set(0.0);
      doubleSupportCaptureKp.set(3.5); 
      singleSupportCaptureKp.set(2.5); 
      kCaptureGuide.set(2.0);
      minPerimeterDistance.set(0.02);
      resizeFootPolygonBy.set(0.01);
   }
}
