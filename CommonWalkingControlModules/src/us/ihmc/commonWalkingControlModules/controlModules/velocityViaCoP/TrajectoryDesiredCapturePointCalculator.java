package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import java.awt.Color;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.plotting.Artifact;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLine2d;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameLine2d;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;


public class TrajectoryDesiredCapturePointCalculator implements DesiredCapturePointCalculator
{

   private enum TransferState
   {
      DOUBLE_SUPPORT, SINGLE_SUPPORT_LEFT, SINGLE_SUPPORT_RIGHT, TRANSFER_TO_SINGLE_SUPPORT
   }

   private enum SweetSpot
   {
      LEFT_FOOT, RIGHT_FOOT, MIDPOINT
   }

   private final YoVariableRegistry registry = new YoVariableRegistry("DesiredCapturePointCalculator");
   private final ProcessedSensorsInterface processedSensors;

   private final EnumYoVariable<TransferState> stateInLastTick = new EnumYoVariable<TransferState>("stateInLastTick", registry, TransferState.class);
   private final EnumYoVariable<SweetSpot> sweetSpotToUseForStartPoint = new EnumYoVariable<SweetSpot>("sweetSpotToUseForStartPoint", registry, SweetSpot.class);

   private final DoubleYoVariable capturePointMoveDuration = new DoubleYoVariable("capturePointMoveDuration", registry);
   private final YoMinimumJerkTrajectory capturePointTrajectory = new YoMinimumJerkTrajectory("capturePointTrajectory", registry);
   private final DoubleYoVariable sweetSpotAdjustmentX = new DoubleYoVariable("sweetSpotAdjustmentX", registry);
   private final DoubleYoVariable distancePastSwitchLine = new DoubleYoVariable("distancePastSwitchLine", "Amount to push the desired capture point past the switch line", registry);

   private final YoFrameLine2d captureLine = new YoFrameLine2d("captureLine", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameLine2d switchLine = new YoFrameLine2d("switchLine", "", ReferenceFrame.getWorldFrame(), registry);
   
   public TrajectoryDesiredCapturePointCalculator(ProcessedSensorsInterface processedSensors, YoGraphicsListRegistry yoGraphicListRegistry, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      parentRegistry.addChild(registry);
      stateInLastTick.set(TransferState.DOUBLE_SUPPORT);
      
      if (yoGraphicListRegistry != null)
      {
         YoArtifactLine2d captureLineArtifact = new YoArtifactLine2d("Capture Line", captureLine, Color.orange);
         YoArtifactLine2d switchLineArtifact = new YoArtifactLine2d("Switch Line", switchLine, Color.pink);

         yoGraphicListRegistry.registerArtifacts("Trajectory Desired Capture Point", new Artifact[]{captureLineArtifact, switchLineArtifact});
      }
   }
  
   
   public FramePoint2d computeDesiredCapturePointSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity, SingleSupportCondition singleSupportCondition)
   {
      hideCaptureLineAndSwitchLine();
      
      if (supportLeg == RobotSide.LEFT)
         stateInLastTick.set(TransferState.SINGLE_SUPPORT_LEFT);
      else
         stateInLastTick.set(TransferState.SINGLE_SUPPORT_RIGHT);

     
      return getSweetSpot(supportLeg, bipedSupportPolygons);
   }
   
   public FramePoint2d computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, OldBipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity)
   {
      if (loadingLeg == null)
      {
         hideCaptureLineAndSwitchLine();
         return computeTransferToDoubleSupport(bipedSupportPolygons);
      }
      else
      {
         return computeTransferToSingleSupport(loadingLeg, bipedSupportPolygons);
      }
   }

   private FramePoint2d getSweetSpot(RobotSide side, OldBipedSupportPolygons bipedSupportPolygons)
   {
      FramePoint2d sweetSpot;
      if(side == null)
      {
         sweetSpot = bipedSupportPolygons.getSupportPolygonInMidFeetZUp().getCentroidCopy();
      }
      else
      {
         sweetSpot = bipedSupportPolygons.getSweetSpotCopy(side);
      }

      sweetSpot.setX(sweetSpot.getX() + sweetSpotAdjustmentX.getDoubleValue());
      
      return sweetSpot;
   }
   
   private FramePoint2d getStartPoint(OldBipedSupportPolygons bipedSupportPolygons)
   {
      FramePoint2d startPoint = null;
      switch (sweetSpotToUseForStartPoint.getEnumValue())
      {
      case LEFT_FOOT:
         startPoint = getSweetSpot(RobotSide.LEFT, bipedSupportPolygons);
         break;
      case RIGHT_FOOT:
         startPoint = getSweetSpot(RobotSide.RIGHT, bipedSupportPolygons);
         break;
      case MIDPOINT:
         startPoint = getSweetSpot(null, bipedSupportPolygons);
         break;
      }
      
      return startPoint;
   }
   
   
   private FramePoint2d computeMorphToEndPoint(FramePoint2d startPoint, FramePoint2d endPoint, OldBipedSupportPolygons bipedSupportPolygons)
   {
      endPoint.changeFrame(startPoint.getReferenceFrame());

      capturePointTrajectory.computeTrajectory(processedSensors.getTime());
//    return FramePoint2d.morph(startPoint, endPoint, capturePointTrajectory.getPosition());
      FramePoint2d frame = new FramePoint2d();
      frame.interpolate(startPoint, endPoint, capturePointTrajectory.getPosition());
      return frame;
   }

   private FramePoint2d computeTransferToDoubleSupport(OldBipedSupportPolygons bipedSupportPolygons)
   {
      if (stateInLastTick.getEnumValue() != TransferState.DOUBLE_SUPPORT)
      {
         initializeTrajectory();
      }

      FramePoint2d startPoint = getStartPoint(bipedSupportPolygons);
      FramePoint2d endPoint = getSweetSpot(null, bipedSupportPolygons);
      FramePoint2d desiredCP = computeMorphToEndPoint(startPoint, endPoint, bipedSupportPolygons);
      stateInLastTick.set(TransferState.DOUBLE_SUPPORT);
      return desiredCP;
   }
   
   private void initializeTrajectory()
   {

      switch (stateInLastTick.getEnumValue())
      {
      case SINGLE_SUPPORT_LEFT:
         sweetSpotToUseForStartPoint.set(SweetSpot.LEFT_FOOT);
         break;
      case SINGLE_SUPPORT_RIGHT:
         sweetSpotToUseForStartPoint.set(SweetSpot.RIGHT_FOOT);
         break;
      case DOUBLE_SUPPORT:
         sweetSpotToUseForStartPoint.set(SweetSpot.MIDPOINT);
         break;
      case TRANSFER_TO_SINGLE_SUPPORT:
         return;
      }

      double startTime = processedSensors.getTime();
      double endTime = startTime + capturePointMoveDuration.getDoubleValue();
      
      capturePointTrajectory.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, startTime, endTime);
      
   }

   private FramePoint2d computeTransferToSingleSupport(RobotSide loadingLeg, OldBipedSupportPolygons bipedSupportPolygons)
   {
      if (stateInLastTick.getEnumValue() != TransferState.TRANSFER_TO_SINGLE_SUPPORT)
      {
         initializeTrajectory();
      }

      FramePoint2d startPoint = getStartPoint(bipedSupportPolygons);
      FramePoint2d endPoint = getSweetSpot(loadingLeg, bipedSupportPolygons);
      endPoint.changeFrame(startPoint.getReferenceFrame());
      
      if (startPoint.distanceSquared(endPoint) > 1e-7)
      {
         computeCaptureLineAndSwitchLine(startPoint, endPoint);
         
         FrameVector2d endPointAdjustment = new FrameVector2d(startPoint, endPoint);
         endPointAdjustment.normalize();
         endPointAdjustment.scale(this.distancePastSwitchLine.getDoubleValue());
         
         endPoint = new FramePoint2d(endPoint);
         endPoint.add(endPointAdjustment);
      }

      FramePoint2d desiredCP = computeMorphToEndPoint(startPoint, endPoint, bipedSupportPolygons);

      stateInLastTick.set(TransferState.TRANSFER_TO_SINGLE_SUPPORT);

      return desiredCP;
   }

   
   private void computeCaptureLineAndSwitchLine(FramePoint2d startPoint, FramePoint2d endPoint)
   {
      FrameLine2d startToEnd = new FrameLine2d(startPoint, endPoint);

      startToEnd.changeFrame(ReferenceFrame.getWorldFrame());
      this.captureLine.setFrameLine2d(startToEnd);

      FrameVector2d perpendicularVector = new FrameVector2d();
      startToEnd.getNormalizedFrameVector(perpendicularVector);
      perpendicularVector.rotate90();

      FramePoint2d endPointInWorld = new FramePoint2d(endPoint);
      endPointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      FrameLine2d frameLine2d = new FrameLine2d(endPointInWorld, perpendicularVector);
      this.switchLine.setFrameLine2d(frameLine2d);
   }
   
   private void hideCaptureLineAndSwitchLine()
   {
      Point2d naNPoint2d = new Point2d(Double.NaN, Double.NaN);
      FrameLine2d naNLine2d = new FrameLine2d(ReferenceFrame.getWorldFrame(), naNPoint2d, naNPoint2d);
      
      this.captureLine.setFrameLine2d(naNLine2d);
      this.switchLine.setFrameLine2d(naNLine2d);
     
   }

   public boolean isPointPastSwitchLine(FramePoint2d framePoint2d, RobotSide loadingLeg, OldBipedSupportPolygons bipedSupportPolygons)
   {
      FramePoint2d startPoint = getStartPoint(bipedSupportPolygons);
      FramePoint2d endPoint = getSweetSpot(loadingLeg, bipedSupportPolygons);
      endPoint.changeFrame(startPoint.getReferenceFrame());
      FramePoint2d pointToCheck = new FramePoint2d(framePoint2d);
      pointToCheck.changeFrame(ReferenceFrame.getWorldFrame());
      
      computeCaptureLineAndSwitchLine(startPoint, endPoint);

     
      return switchLine.getFrameLine2d().isPointOnRightSideOfLine(pointToCheck);
   }
   
   public void setUpParametersForR2()
   {
      capturePointMoveDuration.set(1.0);
      distancePastSwitchLine.set(0.04);
      initializeTrajectory();

   }

   public void setUpParametersForM2V2()
   {
      capturePointMoveDuration.set(1.0);
      sweetSpotAdjustmentX.set(-0.02);
      distancePastSwitchLine.set(0.04);
      initializeTrajectory();

   }

}
