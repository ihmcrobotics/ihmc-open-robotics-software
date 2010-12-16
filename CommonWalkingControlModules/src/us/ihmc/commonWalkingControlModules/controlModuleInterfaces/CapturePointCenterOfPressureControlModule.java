package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

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
   public abstract void controlDoubleSupport(BipedSupportPolygons bipedSupportPolygons, FramePoint currentCapturePoint, FramePoint desiredCapturePoint,
           FramePoint centerOfMassPositionInWorldFrame, FrameVector2d desiredVelocity, FrameVector2d currentVelocity);

   /**
    * sets centerOfPressureDesiredWorld field
    * based on params
    *
    * @param supportLeg RobotSide
    * @param desiredCapturePoint FramePoint
    * @param captureTime double
    * @param supportPolygons BipedSupportPolygons
    * @param desiredCenterOfPressure FramePoint
    */
   public abstract void controlSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint,
           RobotSide supportLeg, ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons);

   public abstract void controlSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint,
           RobotSide supportLeg, ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons, FramePoint centerOfMassPositionInZUpFrame,
           FrameVector2d desiredVelocity, FrameVector2d currentVelocity);

   public abstract YoFramePoint getCenterOfPressureDesiredMidFeet();

   public abstract YoFramePoint getCenterOfPressureDesiredAnkleZUp(RobotSide robotSide);
}
