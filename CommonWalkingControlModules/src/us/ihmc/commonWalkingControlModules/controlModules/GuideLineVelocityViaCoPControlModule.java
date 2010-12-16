package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.CapturePointCenterOfPressureControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.GuideLineCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VelocityViaCoPControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredStepLocation.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class GuideLineVelocityViaCoPControlModule implements VelocityViaCoPControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("GuideLineVelocityViaCoPControlModule");

   private final CommonWalkingReferenceFrames referenceFrames;
   private final CapturePointCenterOfPressureControlModule capturePointCenterOfPressureControlModule;
   private final GuideLineCalculator guideLineCalculator;
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;

   private final YoFramePoint desiredCapturePointInWorld = new YoFramePoint("desiredCapturePoint", "", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint desiredCenterOfPressure = new FramePoint(ReferenceFrame.getWorldFrame());
   private final DoubleYoVariable desiredCaptureForwardDoubleSupport = new DoubleYoVariable("desiredCaptureForwardDoubleSupport", registry);
   private final DoubleYoVariable desiredCaptureInwardDoubleSupport = new DoubleYoVariable("desiredCaptureInwardDoubleSupport", registry);


   public GuideLineVelocityViaCoPControlModule(double controlDT, CouplingRegistry couplingRegistry, ProcessedSensorsInterface processedSensors,
           CommonWalkingReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GuideLineCalculator guideLineCalculator,
           CapturePointCenterOfPressureControlModule capturePointCenterOfPressureControlModule)
   {
      this.referenceFrames = referenceFrames;
      this.couplingRegistry = couplingRegistry;
      this.processedSensors = processedSensors;
      this.capturePointCenterOfPressureControlModule = capturePointCenterOfPressureControlModule;
      this.guideLineCalculator = guideLineCalculator;

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObject desiredCapturePointGraphic = desiredCapturePointInWorld.createDynamicGraphicPosition("Desired Capture Point", 0.01,
                                                              YoAppearance.Yellow(), GraphicType.ROTATED_CROSS);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("GuideLineVelocityViaCoPControlModule", desiredCapturePointGraphic);
         dynamicGraphicObjectsListRegistry.registerArtifact("GuideLineVelocityViaCoPControlModule", desiredCapturePointGraphic.createArtifact());
      }
   }

   public FramePoint2d computeDesiredCoPDoubleSupport(RobotSide loadingLeg, FrameVector2d desiredVelocity)
   {
      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();

      FramePoint desiredCapturePoint;

      if (loadingLeg == null)
      {
         FramePoint2d leftSweetSpot = bipedSupportPolygons.getSweetSpotCopy(RobotSide.LEFT);
         FramePoint2d rightSweetSpot = bipedSupportPolygons.getSweetSpotCopy(RobotSide.RIGHT);
         rightSweetSpot.changeFrame(leftSweetSpot.getReferenceFrame());
         FrameLineSegment2d sweetSpotToSweetSpot = new FrameLineSegment2d(leftSweetSpot, rightSweetSpot);
         desiredCapturePoint = new FramePoint(sweetSpotToSweetSpot.midpoint().toFramePoint());
         
//         desiredCapturePoint = new FramePoint(referenceFrames.getMidFeetZUpFrame());
      }
      else
      {
         double desiredCaptureY = loadingLeg.negateIfLeftSide(desiredCaptureInwardDoubleSupport.getDoubleValue());
         desiredCapturePoint = new FramePoint(referenceFrames.getAnkleZUpReferenceFrames().get(loadingLeg), desiredCaptureForwardDoubleSupport.getDoubleValue(), desiredCaptureY, 0.0);
      }

      this.desiredCapturePointInWorld.set(desiredCapturePoint.changeFrameCopy(ReferenceFrame.getWorldFrame()));

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

//    desiredCapturePoint = desiredCapturePoint.changeFrameCopy(currentCapturePoint.getReferenceFrame());
      desiredCapturePoint.changeFrame(midFeetZUpFrame);
      FramePoint currentCapturePoint = couplingRegistry.getCapturePoint().changeFrameCopy(midFeetZUpFrame);
      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(midFeetZUpFrame);
      FrameVector2d currentCOMVelocity = processedSensors.getCenterOfMassVelocityInFrame(midFeetZUpFrame).toFrameVector2d();

      desiredVelocity.setX(0.0);
      desiredVelocity.setY(0.0);
      capturePointCenterOfPressureControlModule.controlDoubleSupport(bipedSupportPolygons, currentCapturePoint, desiredCapturePoint,
              centerOfMassPosition, desiredVelocity, currentCOMVelocity);
      capturePointCenterOfPressureControlModule.packDesiredCenterOfPressure(desiredCenterOfPressure);

      return new FramePoint2d(desiredCenterOfPressure.getReferenceFrame(), desiredCenterOfPressure.getX(), desiredCenterOfPressure.getY());
   }

   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, FrameVector2d desiredVelocity)
   {
      desiredCapturePointInWorld.set(Double.NaN, Double.NaN, Double.NaN);

      ReferenceFrame supportFootAnkleZUpFrame = referenceFrames.getAnkleZUpReferenceFrames().get(supportLeg);

      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();

      Footstep footstep = couplingRegistry.getDesiredFootstep();
      FramePoint finalDesiredSwingTarget = footstep.footstepPosition;
//      FramePoint localFinalDesiredSwingTarget = finalDesiredSwingTarget.changeFrameCopy(supportFootAnkleZUpFrame);

      FramePoint capturePoint = couplingRegistry.getCapturePoint();
      FramePoint capturePointInAnkleZUp = capturePoint.changeFrameCopy(supportFootAnkleZUpFrame);
      FramePoint2d capturePoint2d = capturePointInAnkleZUp.toFramePoint2d();

      FrameVector2d desiredVelocityInSupportFootFrame = desiredVelocity.changeFrameCopy(supportFootAnkleZUpFrame);
      FrameVector2d actualCenterOfMassVelocityInSupportFootFrame = processedSensors.getCenterOfMassVelocityInFrame(supportFootAnkleZUpFrame).toFrameVector2d();


      guideLineCalculator.update(supportLeg, bipedSupportPolygons, capturePoint2d, finalDesiredSwingTarget, desiredVelocityInSupportFootFrame,
                                 actualCenterOfMassVelocityInSupportFootFrame);

      FrameLineSegment2d guideLine = guideLineCalculator.getGuideLine(supportLeg);
      guideLine = guideLine.changeFrameCopy(supportFootAnkleZUpFrame);

      FramePoint desiredCapturePoint = null;

      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(supportFootAnkleZUpFrame);

      capturePointCenterOfPressureControlModule.controlSingleSupport(capturePointInAnkleZUp, guideLine, desiredCapturePoint, supportLeg,
              supportFootAnkleZUpFrame, bipedSupportPolygons, centerOfMassPosition, desiredVelocity, actualCenterOfMassVelocityInSupportFootFrame);    // , percentToFarEdgeOfFoot); // calculates capture points

      capturePointCenterOfPressureControlModule.packDesiredCenterOfPressure(desiredCenterOfPressure);

      return desiredCenterOfPressure.toFramePoint2d();
   }

   public void setUpParametersForR2()
   {
      desiredCaptureForwardDoubleSupport.set(0.2);    // 0.15;
      desiredCaptureInwardDoubleSupport.set(0.02);
   }
   
   public void setUpParametersForM2V2()
   {
      desiredCaptureForwardDoubleSupport.set(0.04); // equal to where the guide line ends // 0.08);
      desiredCaptureInwardDoubleSupport.set(0.0);
   }
}
