package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.CapturePointCenterOfPressureControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.GuideLineCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VelocityViaCoPControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class GuideLineVelocityViaCoPControlModule implements VelocityViaCoPControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("GuideLineVelocityViaCoPControlModule");

   private final double controlDT;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final CapturePointCenterOfPressureControlModule capturePointCenterOfPressureControlModule;
   private final GuideLineCalculator guideLineCalculator;
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;

   private final YoFramePoint desiredCapturePointInWorld = new YoFramePoint("desiredCapturePoint", "", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint desiredCenterOfPressure = new FramePoint(ReferenceFrame.getWorldFrame());
   private final DoubleYoVariable desiredCaptureForwardDoubleSupport = new DoubleYoVariable("desiredCaptureForwardDoubleSupport", registry);
   private final DoubleYoVariable desiredCaptureInwardDoubleSupport = new DoubleYoVariable("desiredCaptureInwardDoubleSupport", registry);
   private final DoubleYoVariable desiredCaptureForwardNotLoading = new DoubleYoVariable("desiredCaptureForwardNotLoading", registry);

   private final DoubleYoVariable alphaDesiredCoP = new DoubleYoVariable("alphaDesiredCoP", registry);
   private final AlphaFilteredYoFramePoint2d filteredDesiredCoP;
   private Boolean lastTickDoubleSupport = null;


   public GuideLineVelocityViaCoPControlModule(double controlDT, CouplingRegistry couplingRegistry, ProcessedSensorsInterface processedSensors,
           CommonWalkingReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GuideLineCalculator guideLineCalculator,
           CapturePointCenterOfPressureControlModule capturePointCenterOfPressureControlModule)
   {
      this.controlDT = controlDT;
      this.referenceFrames = referenceFrames;
      this.couplingRegistry = couplingRegistry;
      this.processedSensors = processedSensors;
      this.capturePointCenterOfPressureControlModule = capturePointCenterOfPressureControlModule;
      this.guideLineCalculator = guideLineCalculator;
      this.filteredDesiredCoP = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d("filteredDesiredCoP", "", registry, alphaDesiredCoP,
              referenceFrames.getMidFeetZUpFrame());

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
      if ((lastTickDoubleSupport == null) ||!lastTickDoubleSupport)
      {
         filteredDesiredCoP.reset();
      }

      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();

      FramePoint desiredCapturePoint = computeDesiredCapturePointDoubleSupport(loadingLeg, desiredVelocity, bipedSupportPolygons);

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

//    desiredCapturePoint = desiredCapturePoint.changeFrameCopy(currentCapturePoint.getReferenceFrame());
      desiredCapturePoint.changeFrame(midFeetZUpFrame);
      FramePoint currentCapturePoint = couplingRegistry.getCapturePointInFrame(midFeetZUpFrame);
      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(midFeetZUpFrame);
      FrameVector2d currentBodyVelocity = processedSensors.getCenterOfMassVelocityInFrame(midFeetZUpFrame, null).toFrameVector2d();

//    desiredVelocity.setX(0.0);
//    desiredVelocity.setY(0.0);
      capturePointCenterOfPressureControlModule.controlDoubleSupport(bipedSupportPolygons, currentCapturePoint, desiredCapturePoint, centerOfMassPosition,
              desiredVelocity, currentBodyVelocity);
      capturePointCenterOfPressureControlModule.packDesiredCenterOfPressure(desiredCenterOfPressure);

      FramePoint2d desiredCoP2d = new FramePoint2d(desiredCenterOfPressure.getReferenceFrame(), desiredCenterOfPressure.getX(), desiredCenterOfPressure.getY());
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      desiredCoP2d.changeFrame(supportPolygon.getReferenceFrame());
      supportPolygon.orthogonalProjection(desiredCoP2d);
      desiredCoP2d.changeFrame(filteredDesiredCoP.getReferenceFrame());
      filteredDesiredCoP.update(desiredCoP2d);

      return filteredDesiredCoP.getFramePoint2dCopy();
   }

   private FramePoint computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, FrameVector2d desiredVelocity, BipedSupportPolygons bipedSupportPolygons)
   {
      FramePoint desiredCapturePoint;

      boolean stayInDoubleSupport = loadingLeg == null;
      if (stayInDoubleSupport)
      {
         desiredCapturePoint = new FramePoint(referenceFrames.getMidFeetZUpFrame());

         FrameVector leftForward = new FrameVector(referenceFrames.getAnkleZUpFrame(RobotSide.LEFT), 1.0, 0.0, 0.0);
         FrameVector rightForward = new FrameVector(referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT), 1.0, 0.0, 0.0);

         leftForward.changeFrame(desiredCapturePoint.getReferenceFrame());
         rightForward.changeFrame(desiredCapturePoint.getReferenceFrame());

         FrameVector offset = leftForward;
         offset.add(rightForward);
         offset.normalize();
         offset.scale(desiredCaptureForwardNotLoading.getDoubleValue());
         desiredCapturePoint.add(offset);
      }
      else if (desiredVelocity.lengthSquared() > 0.0)
      {
         double desiredCaptureY = loadingLeg.negateIfLeftSide(desiredCaptureInwardDoubleSupport.getDoubleValue());
         desiredCapturePoint = new FramePoint(referenceFrames.getAnkleZUpReferenceFrames().get(loadingLeg),
                 desiredCaptureForwardDoubleSupport.getDoubleValue(), desiredCaptureY, 0.0);
      }
      else
      {
         desiredCapturePoint = bipedSupportPolygons.getSweetSpotCopy(loadingLeg).toFramePoint();
      }

      this.desiredCapturePointInWorld.set(desiredCapturePoint.changeFrameCopy(ReferenceFrame.getWorldFrame()));

      return desiredCapturePoint;
   }

   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, FrameVector2d desiredVelocity)
   {
      if ((lastTickDoubleSupport == null) || lastTickDoubleSupport)
      {
         filteredDesiredCoP.reset();
      }

      desiredCapturePointInWorld.set(Double.NaN, Double.NaN, Double.NaN);

      ReferenceFrame supportFootAnkleZUpFrame = referenceFrames.getAnkleZUpReferenceFrames().get(supportLeg);

      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();

      FramePoint capturePointInAnkleZUp = couplingRegistry.getCapturePointInFrame(supportFootAnkleZUpFrame);
      FramePoint2d capturePoint2d = capturePointInAnkleZUp.toFramePoint2d();

      FrameVector2d actualCenterOfMassVelocityInSupportFootFrame = processedSensors.getCenterOfMassVelocityInFrame(supportFootAnkleZUpFrame,
                                                                      supportLeg).toFrameVector2d();

      FramePoint desiredCapturePoint = null;
      FrameLineSegment2d guideLine = null;
      if (desiredVelocity.lengthSquared() > 0.0)
      {
         Footstep footstep = couplingRegistry.getDesiredFootstep();
         FramePoint finalDesiredSwingTarget = footstep.getFootstepPose().getPosition();
         FrameVector2d desiredVelocityInSupportFootFrame = desiredVelocity.changeFrameCopy(supportFootAnkleZUpFrame);
         guideLineCalculator.update(supportLeg, bipedSupportPolygons, capturePoint2d, finalDesiredSwingTarget, desiredVelocityInSupportFootFrame,
                                    actualCenterOfMassVelocityInSupportFootFrame);
         guideLine = guideLineCalculator.getGuideLine(supportLeg);
         guideLine.changeFrame(supportFootAnkleZUpFrame);
      }
      else
      {
         desiredCapturePoint = couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(supportLeg).toFramePoint();
      }


      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(supportFootAnkleZUpFrame);

      capturePointCenterOfPressureControlModule.controlSingleSupport(capturePointInAnkleZUp, guideLine, desiredCapturePoint, supportLeg,
              supportFootAnkleZUpFrame, bipedSupportPolygons, centerOfMassPosition, desiredVelocity, actualCenterOfMassVelocityInSupportFootFrame);    // , percentToFarEdgeOfFoot); // calculates capture points

      capturePointCenterOfPressureControlModule.packDesiredCenterOfPressure(desiredCenterOfPressure);
      FramePoint2d desiredCoP2d = desiredCenterOfPressure.toFramePoint2d();
      desiredCoP2d.changeFrame(filteredDesiredCoP.getReferenceFrame());
      filteredDesiredCoP.update(desiredCoP2d);

      return filteredDesiredCoP.getFramePoint2dCopy();
   }

   public void setUpParametersForR2()
   {
      desiredCaptureForwardDoubleSupport.set(0.18);    // 0.2);    // 0.15;
      desiredCaptureInwardDoubleSupport.set(0.01);    // 0.02);
      desiredCaptureForwardNotLoading.set(0.05);
      alphaDesiredCoP.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(5.0, controlDT));
   }

   public void setUpParametersForM2V2()
   {
      desiredCaptureForwardDoubleSupport.set(0.03);    // 0.02); // 0.04 (equal to where the guide line ends) // 0.08);
      desiredCaptureInwardDoubleSupport.set(0.0);
      desiredCaptureForwardNotLoading.set(0.02);
      alphaDesiredCoP.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(5.0, controlDT));
   }
}
