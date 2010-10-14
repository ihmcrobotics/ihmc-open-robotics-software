package us.ihmc.commonWalkingControlModules.controlModules;

import java.awt.Color;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturePointCalculatorInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.YoFrameLine2dArtifact;
import com.yobotics.simulationconstructionset.plotting.YoFrameLineSegment2dArtifact;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLine2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLineSegment2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class SpeedControllingCapturePointCenterOfPressureControlModule implements CapturePointCenterOfPressureControlModule
{
   // Standard controller
   private CapturePointCenterOfPressureControlModule standardCapturePointCenterOfPressureControlModule;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("SpeedControllingCapturePointCenterOfPressureController");
   
   // Reference frames
   private ReferenceFrame bodyZUp, midFeetZUp, world;
   
   // Points
   private final YoFramePoint centerOfPressureDesiredWorld, centerOfPressureDesiredMidFeet, centerOfPressureDesiredLeftAnkleZUp,
   centerOfPressureDesiredRightAnkleZUp;

   private final SideDependentList<YoFramePoint> centerOfPressureDesiredAnkleZUp;
   
   private final YoFrameLine2d cpLine;
   private final YoFrameLine2d comDirectionLine;
   
   private DoubleYoVariable speedControlXKp = new DoubleYoVariable("speedControlXKp", registry);
   private DoubleYoVariable speedControlYKp = new DoubleYoVariable("speedControlYKp", registry);
   		
   
   private final DynamicGraphicPosition centerOfPressureDesiredWorldGraphicPosition;

   public SpeedControllingCapturePointCenterOfPressureControlModule(double controlDT, CommonWalkingReferenceFrames referenceFrames, YoVariableRegistry yoVariableRegistry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      // Load standard controller
      standardCapturePointCenterOfPressureControlModule = 
         new StandardCapturePointCenterOfPressureControlModule(controlDT, referenceFrames, yoVariableRegistry, null);
      
      // Setup reference frames
      bodyZUp = referenceFrames.getABodyAttachedZUpFrame();
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      world = ReferenceFrame.getWorldFrame();

      cpLine = new YoFrameLine2d("cpLine", "", world, registry);
      comDirectionLine = new YoFrameLine2d("comDirectionLine", "", world, registry);
      
      // Points
      centerOfPressureDesiredWorld = new YoFramePoint("spdCopDesWorld", "", world, registry);
      centerOfPressureDesiredMidFeet = new YoFramePoint("spdCopDesMidfeet", "", midFeetZUp, registry);
      centerOfPressureDesiredLeftAnkleZUp = new YoFramePoint("spdCopDesLaZUp", "", referenceFrames.getAnkleZUpReferenceFrames().get(RobotSide.LEFT),
              registry);
      centerOfPressureDesiredRightAnkleZUp = new YoFramePoint("spdCopDesRaZUp", "",
              referenceFrames.getAnkleZUpReferenceFrames().get(RobotSide.RIGHT), registry);

      
      centerOfPressureDesiredAnkleZUp = new SideDependentList<YoFramePoint>(centerOfPressureDesiredLeftAnkleZUp, centerOfPressureDesiredRightAnkleZUp);

      
      speedControlXKp.set(6.0);
      
      
      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObjectsList dynamicGraphicObjectList = new DynamicGraphicObjectsList("CapturePointController");

         centerOfPressureDesiredWorldGraphicPosition = new DynamicGraphicPosition("Desired Center of Pressure", centerOfPressureDesiredWorld, 0.012,
                 YoAppearance.Gray(), DynamicGraphicPosition.GraphicType.CROSS);

         dynamicGraphicObjectList.add(centerOfPressureDesiredWorldGraphicPosition);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectList);

         ArtifactList artifactList = new ArtifactList("Capture Point CoP Control Module");

         artifactList.add(centerOfPressureDesiredWorldGraphicPosition.createArtifact());

         YoFrameLine2dArtifact dynamicGraphicYoFrameLine2dArtifact = new YoFrameLine2dArtifact("cp Line", cpLine, Color.RED);
         artifactList.add(dynamicGraphicYoFrameLine2dArtifact);
         
         
         YoFrameLine2dArtifact comDirectionArtifact = new YoFrameLine2dArtifact("comDirectionLine", comDirectionLine, Color.RED);
         artifactList.add(comDirectionArtifact);
//
//         YoFrameLine2dArtifact dynamicGraphicYoFrameLine2dArtifact = new YoFrameLine2dArtifact("Parallel Line", parallelLineWorld, Color.GREEN);
//         artifactList.add(dynamicGraphicYoFrameLine2dArtifact);

         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }
      else
      {
         centerOfPressureDesiredWorldGraphicPosition = null;
      }
      
      if (yoVariableRegistry != null)    // && (VarListsToRegister.REGISTER_CAPTURE_POINT_CENTER_OF_PRESSURE_CONTROLLER))
      {
         yoVariableRegistry.addChild(registry);
      }

   }
   
   private void copyCenterOfPressureDesired()
   {
      centerOfPressureDesiredWorld.set(standardCapturePointCenterOfPressureControlModule.getCenterOfPressureDesiredWorld().getFramePointCopy());
      centerOfPressureDesiredLeftAnkleZUp.set(standardCapturePointCenterOfPressureControlModule.getCenterOfPressureDesiredLeftAnkleZUp().getFramePointCopy());
      centerOfPressureDesiredMidFeet.set(standardCapturePointCenterOfPressureControlModule.getCenterOfPressureDesiredMidFeet());
      centerOfPressureDesiredRightAnkleZUp.set(standardCapturePointCenterOfPressureControlModule.getCenterOfPressureDesiredRightAnkleZUp());
   }
   
   @Override
   public void XYCoPControllerDoubleSupport(BipedSupportPolygons bipedSupportPolygons, CapturePointCalculatorInterface yoboticsBipedCapturePointCalculator,
         FramePoint desiredCapturePoint)
   {
      throw new RuntimeException("Needs Center of Mass data");
   }
   
   
   @Override
   public void XYCoPControllerDoubleSupport(BipedSupportPolygons bipedSupportPolygons, CapturePointCalculatorInterface yoboticsBipedCapturePointCalculator,
         FramePoint desiredCapturePoint, FramePoint centerOfMassPosition, FrameVector2d desiredVelocity, FrameVector currentCOMVelocity)
   {
      FramePoint currentCapturePoint = yoboticsBipedCapturePointCalculator.getCapturePointInFrame(desiredCapturePoint.getReferenceFrame());

      XYCoPControllerDoubleSupport(bipedSupportPolygons, currentCapturePoint, desiredCapturePoint, centerOfMassPosition, desiredVelocity, currentCOMVelocity);
   }
   
  

   @Override
   public void XYCoPControllerDoubleSupport(BipedSupportPolygons bipedSupportPolygons, 
         FramePoint currentCapturePoint, FramePoint desiredCapturePoint, FramePoint centerOfMassPositionInZUpFrame, FrameVector2d desiredVelocity, FrameVector currentVelocity)
   {
     // Create Line from dCP to iCP
      FrameLine2d controlLine = new FrameLine2d(desiredCapturePoint.toFramePoint2d(), currentCapturePoint.toFramePoint2d());
      cpLine.setFrameLine2d(controlLine.changeFrameCopy(world));
      
      
      // Project CoM on control line
      FrameVector2d velocityT = new FrameVector2d(desiredVelocity);
      velocityT.setX(desiredVelocity.getY());
      velocityT.setY(desiredVelocity.getX());
      
      
      FramePoint2d speedControlPosition = new FramePoint2d(centerOfMassPositionInZUpFrame.toFramePoint2d());
      
      currentVelocity.changeFrame(world);
      
      // Speed controller: Only increase speed for now
      speedControlPosition.setX(speedControlPosition.getX() + speedControlXKp.getDoubleValue() * Math.min(0.0,currentVelocity.getX() - desiredVelocity.getX()));
      speedControlPosition.setY(speedControlPosition.getY() + speedControlYKp.getDoubleValue() * Math.min(0.0,currentVelocity.getY() - desiredVelocity.getY()));
      
      
      
      
      
      
      velocityT = velocityT.changeFrameCopy(midFeetZUp);
      
      FrameLine2d massLine = new FrameLine2d(speedControlPosition,velocityT);
      comDirectionLine.setFrameLine2d(massLine.changeFrameCopy(world));
      
     FramePoint2d centerOfPressureDesired = controlLine.intersectionWith(massLine);
 
      //centerOfPressureDesired.sub(controlLine);
      
     
     if (bipedSupportPolygons != null)
     {

        FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();

        // If feasible CoP is not inside the convex hull of the feet, project it into it.
        if (!supportPolygon.isPointInside(centerOfPressureDesired))
        {
           // supportPolygon.orthogonalProjection(centerOfPressureDesired2d);

           if (desiredCapturePoint.getReferenceFrame() != midFeetZUp)
           {
              desiredCapturePoint = desiredCapturePoint.changeFrameCopy(midFeetZUp);
           }

           FramePoint2d desiredCapturePoint2d = new FramePoint2d(desiredCapturePoint.getReferenceFrame(), desiredCapturePoint.getX(),
                                                   desiredCapturePoint.getY());
           FrameLineSegment2d desiredCaptureToDesiredCop = new FrameLineSegment2d(desiredCapturePoint2d, centerOfPressureDesired);


           FramePoint2d[] intersections = supportPolygon.intersectionWith(desiredCaptureToDesiredCop);
           if (intersections != null)
           {
              FramePoint2d intersectionToUse;

              if (intersections.length == 1)
              {
                 intersectionToUse = intersections[0];
              }
              else
              {
                 double distanceSquaredToIntersection0 = centerOfPressureDesired.distanceSquared(intersections[0]);
                 double distanceSquaredToIntersection1 = centerOfPressureDesired.distanceSquared(intersections[1]);

                 if (distanceSquaredToIntersection0 <= distanceSquaredToIntersection1)
                    intersectionToUse = intersections[0];
                 else
                    intersectionToUse = intersections[1];
              }

              centerOfPressureDesired.setX(intersectionToUse.getX());
              centerOfPressureDesired.setY(intersectionToUse.getY());

              // Move in a little along the line:
              FrameVector2d frameVector2d = desiredCaptureToDesiredCop.getVectorCopy();
              frameVector2d.normalize();
              frameVector2d.scale(-0.002);    // Move toward desired capture by 2 mm to prevent some jerky behavior with VTPs..

              centerOfPressureDesired.setX(centerOfPressureDesired.getX() + frameVector2d.getX());
              centerOfPressureDesired.setY(centerOfPressureDesired.getY() + frameVector2d.getY());
           }
           else
           {
              throw new RuntimeException("Shouldn't get here");
           }
        }
     }
     
     
     
     
      centerOfPressureDesiredMidFeet.set(new FramePoint(midFeetZUp,centerOfPressureDesired.getX(), centerOfPressureDesired.getY(), 0.0));
      
      centerOfPressureDesiredWorld.set(centerOfPressureDesiredMidFeet.getFramePointCopy().changeFrameCopy(world));

      
      
      
   }

   @Override
   public void XYCoPControllerSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, RobotSide supportLeg, ReferenceFrame referenceFrame,
         BipedSupportPolygons supportPolygons)
   {
      FramePoint2d sweetSpot2d = supportPolygons.getSweetSpotCopy(supportLeg);
      FramePoint sweetSpot = new FramePoint(sweetSpot2d.getReferenceFrame(), sweetSpot2d.getX(), sweetSpot2d.getY(), 0.0);

      XYCoPControllerSingleSupport(currentCapturePoint, guideLine, sweetSpot, supportLeg, referenceFrame, supportPolygons);
   }

   @Override
   public void XYCoPControllerSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, RobotSide supportLeg,
         ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons)
   {
      comDirectionLine.setFrameLine2d(null);
      cpLine.setFrameLine2d(null);
      standardCapturePointCenterOfPressureControlModule.XYCoPControllerSingleSupport(currentCapturePoint, guideLine, desiredCapturePoint, supportLeg, referenceFrame, supportPolygons);
      copyCenterOfPressureDesired();
   }

   @Override
   public void setMaxCaptureToCoP(double maxCaptureToCoP)
   {
      standardCapturePointCenterOfPressureControlModule.setMaxCaptureToCoP(maxCaptureToCoP);
   }

   @Override
   public void setKCaptureX(double kx)
   {
      standardCapturePointCenterOfPressureControlModule.setKCaptureX(kx);
   }

   @Override
   public double getKCaptureX()
   {
      return standardCapturePointCenterOfPressureControlModule.getKCaptureY();
   }

   @Override
   public void setKCaptureY(double ky)
   {
      standardCapturePointCenterOfPressureControlModule.setKCaptureY(ky);
   }

   @Override
   public double getKCaptureY()
   {
      return standardCapturePointCenterOfPressureControlModule.getKCaptureY();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.controlModules.CapturePointCenterOfPressureControlModule#getCenterOfPressureDesiredWorld()
    */
   @Override
   public YoFramePoint getCenterOfPressureDesiredWorld()
   {
      return centerOfPressureDesiredWorld;
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.controlModules.CapturePointCenterOfPressureControlModule#getCenterOfPressureDesiredMidFeet()
    */
   @Override
   public YoFramePoint getCenterOfPressureDesiredMidFeet()
   {
      return centerOfPressureDesiredMidFeet;
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.controlModules.CapturePointCenterOfPressureControlModule#getCenterOfPressureDesiredLeftAnkleZUp()
    */
   @Override
   public YoFramePoint getCenterOfPressureDesiredLeftAnkleZUp()
   {
      return centerOfPressureDesiredLeftAnkleZUp;
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.controlModules.CapturePointCenterOfPressureControlModule#getCenterOfPressureDesiredRightAnkleZUp()
    */
   @Override
   public YoFramePoint getCenterOfPressureDesiredRightAnkleZUp()
   {
      return centerOfPressureDesiredRightAnkleZUp;
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.controlModules.CapturePointCenterOfPressureControlModule#getCenterOfPressureDesiredAnkleZUp(us.ihmc.commonWalkingControlModules.RobotSide)
    */
   @Override
   public YoFramePoint getCenterOfPressureDesiredAnkleZUp(RobotSide robotSide)
   {
      return centerOfPressureDesiredAnkleZUp.get(robotSide);
   }

   

}
