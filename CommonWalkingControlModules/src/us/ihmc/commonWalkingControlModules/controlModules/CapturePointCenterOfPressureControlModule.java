package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturePointCalculatorInterface;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public interface CapturePointCenterOfPressureControlModule
{

   public abstract void XYCoPControllerDoubleSupport(BipedSupportPolygons bipedSupportPolygons,
         CapturePointCalculatorInterface yoboticsBipedCapturePointCalculator, FramePoint desiredCapturePoint);

   /**
    * Finds instantaneous center of pressure to make robot move to desired x and y
    *
    * @param xDesiredMidFrame double
    * @param yDesiredMidframe double
    * @param processedSensors ProcessedSensors
    * @todo add stepping if outside footbase
    */
   public abstract void XYCoPControllerDoubleSupport(BipedSupportPolygons bipedSupportPolygons, FramePoint currentCapturePoint, FramePoint desiredCapturePoint);

   public abstract void XYCoPControllerSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, RobotSide supportLeg,
         ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons);

   /**
    * XYCoPControllerSingleSupport sets centerOfPressureDesiredWorld field
    * based on params
    *
    * @param supportLeg RobotSide
    * @param desiredCapturePoint FramePoint
    * @param captureTime double
    * @param supportPolygons BipedSupportPolygons
    * @param desiredCenterOfPressure FramePoint
    */
   public abstract void XYCoPControllerSingleSupport(FramePoint currentCapturePoint, FrameLineSegment2d guideLine, FramePoint desiredCapturePoint,
         RobotSide supportLeg, ReferenceFrame referenceFrame, BipedSupportPolygons supportPolygons);

   public abstract void setMaxCaptureToCoP(double maxCaptureToCoP);

   /**
    *    sets k for x capture point
    *    @param kx double
    */
   public abstract void setKCaptureX(double kx);

   /**
    * gets k for x capture point
    * @return double
    */
   public abstract double getKCaptureX();

   /**
    * sets k for y capture point
    * @param ky double
    */
   public abstract void setKCaptureY(double ky);

   /**
    * gets k for y capture point
    * @return double
    */
   public abstract double getKCaptureY();

   public abstract YoFramePoint getCenterOfPressureDesiredWorld();

   public abstract YoFramePoint getCenterOfPressureDesiredMidFeet();

   public abstract YoFramePoint getCenterOfPressureDesiredLeftAnkleZUp();

   public abstract YoFramePoint getCenterOfPressureDesiredRightAnkleZUp();

   public abstract YoFramePoint getCenterOfPressureDesiredAnkleZUp(RobotSide robotSide);

}