package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;


public interface CapturePointCenterOfPressureControlModule
{
   /**
    * Finds instantaneous center of pressure to make robot move to desired x and y
    * @param desiredVelocity
    * @param currentVelocity TODO
    * @param xDesiredMidFrame double
    * @param yDesiredMidframe double
    * @param processedSensors ProcessedSensors
    * @todo add stepping if outside footbase
    */
   public abstract void controlDoubleSupport(OldBipedSupportPolygons bipedSupportPolygons, FramePoint capturePoint, FramePoint desiredCapturePoint,
           FramePoint centerOfMassPositionInWorldFrame, FrameVector2d desiredVelocity, FrameVector2d currentVelocity);

   /**
    * sets centerOfPressureDesiredWorld field
    * based on params
    * @param supportLeg RobotSide
    * @param supportPolygons BipedSupportPolygons
    * @param desiredCapturePoint FramePoint
    * @param captureTime double
    * @param desiredCenterOfPressure FramePoint
    */
   public abstract void controlSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons supportPolygons, FramePoint capturePoint,
           FrameVector2d desiredVelocity, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, FramePoint centerOfMassPositionInZUpFrame,
           FrameVector2d currentVelocity); // TODO: get rid of last 3 arguments

   public abstract void packDesiredCenterOfPressure(FramePoint desiredCenterOfPressureToPack);
}
