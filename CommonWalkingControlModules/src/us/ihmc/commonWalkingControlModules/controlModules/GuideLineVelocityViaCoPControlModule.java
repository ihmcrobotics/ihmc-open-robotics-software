package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
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

import com.yobotics.simulationconstructionset.BooleanYoVariable;
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

   private final BooleanYoVariable putWeightOnLeftToes = new BooleanYoVariable("putWeightOnLeftToes", registry);
   private final BooleanYoVariable putWeightOnRightToes = new BooleanYoVariable("putWeightOnRightToes", registry);
   private final SideDependentList<BooleanYoVariable> putWeightOnToes = new SideDependentList<BooleanYoVariable>(putWeightOnLeftToes, putWeightOnRightToes);


   private final YoFramePoint desiredCapturePointInWorld = new YoFramePoint("desiredCapturePoint", "", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable desiredCaptureForward = new DoubleYoVariable("desiredCaptureX", registry);
   private final DoubleYoVariable desiredCaptureIn = new DoubleYoVariable("desiredCaptureY", registry);


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


      putWeightOnToes.get(RobotSide.LEFT).set(false);
      putWeightOnToes.get(RobotSide.RIGHT).set(false);

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

   public FramePoint2d computeCapturePoint()
   {
      FramePoint capturePoint = couplingRegistry.getCapturePoint();
      FramePoint2d ret = new FramePoint2d(capturePoint.getReferenceFrame(), capturePoint.getX(), capturePoint.getY());

      return ret;
   }


   public FramePoint2d computeDesiredCoPDoubleSupport(RobotSide loadingLeg, FrameVector2d desiredVelocity)
   {
      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();

//    FramePoint2d desiredCapturePoint = new FramePoint2d(referenceFrames.getAnkleZUpReferenceFrames().get(RobotSide.LEFT));
      FramePoint currentCapturePoint = couplingRegistry.getCapturePoint();

      FramePoint desiredCapturePoint;

      if (loadingLeg == null)
      {
         desiredCapturePoint = new FramePoint(referenceFrames.getMidFeetZUpFrame());
      }
      else
      {
         // TODO: Should probably use sweet spot, but currently using a user defined parameter
//         FramePoint2d sweetSpot = bipedSupportPolygons.getSweetSpotCopy(loadingLeg);
//         desiredCapturePoint = new FramePoint(referenceFrames.getAnkleZUpReferenceFrames().get(loadingLeg), sweetSpot.getX(), sweetSpot.getY(), 0.0);

         double desiredCaptureY = loadingLeg.negateIfLeftSide(desiredCaptureIn.getDoubleValue());
         desiredCapturePoint = new FramePoint(referenceFrames.getAnkleZUpReferenceFrames().get(loadingLeg), desiredCaptureForward.getDoubleValue(), desiredCaptureY, 0.0);
      }


      this.desiredCapturePointInWorld.set(desiredCapturePoint.changeFrameCopy(ReferenceFrame.getWorldFrame()));

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

//    desiredCapturePoint = desiredCapturePoint.changeFrameCopy(currentCapturePoint.getReferenceFrame());
      desiredCapturePoint = desiredCapturePoint.changeFrameCopy(midFeetZUpFrame);
      currentCapturePoint = currentCapturePoint.changeFrameCopy(midFeetZUpFrame);
      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(midFeetZUpFrame);
      FrameVector2d currentCOMVelocity = processedSensors.getCenterOfMassVelocityInFrame(midFeetZUpFrame).toFrameVector2d();

      capturePointCenterOfPressureControlModule.XYCoPControllerDoubleSupport(bipedSupportPolygons, currentCapturePoint, desiredCapturePoint,
              centerOfMassPosition, desiredVelocity, currentCOMVelocity);
      YoFramePoint desiredCenterOfPressure = capturePointCenterOfPressureControlModule.getCenterOfPressureDesiredMidFeet();

      return new FramePoint2d(desiredCenterOfPressure.getReferenceFrame(), desiredCenterOfPressure.getX(), desiredCenterOfPressure.getY());
   }

   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg)
   {
      return computeDesiredCoPSingleSupport(supportLeg, null);
   }

   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, FrameVector2d desiredVelocity)
   {
      desiredCapturePointInWorld.set(Double.NaN, Double.NaN, Double.NaN);

      ReferenceFrame supportFootAnkleZUpFrame = referenceFrames.getAnkleZUpReferenceFrames().get(supportLeg);

      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();

      Footstep footstep = couplingRegistry.getDesiredStepLocation();
      FramePoint finalDesiredSwingTarget = footstep.footstepPosition;
      finalDesiredSwingTarget = finalDesiredSwingTarget.changeFrameCopy(supportFootAnkleZUpFrame);

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

      capturePointCenterOfPressureControlModule.XYCoPControllerSingleSupport(capturePointInAnkleZUp, guideLine, desiredCapturePoint, supportLeg,
              supportFootAnkleZUpFrame, bipedSupportPolygons, centerOfMassPosition, desiredVelocity, actualCenterOfMassVelocityInSupportFootFrame);    // , percentToFarEdgeOfFoot); // calculates capture points

      YoFramePoint desiredCenterOfPressure = capturePointCenterOfPressureControlModule.getCenterOfPressureDesiredAnkleZUp(supportLeg);

      return desiredCenterOfPressure.getFramePointCopy().toFramePoint2d();
   }

   public FramePoint2d getDesiredCoPOffset()
   {
      throw new RuntimeException("Not Implemented!");
   }

   public void setDesiredCoPOffset(FramePoint2d framePoint)
   {
      // @todo  what is this supposed to do?
      // throw new RuntimeException("Not Implemented!");
   }

   public void setPutWeightOnToes(RobotSide robotSide)
   {
      this.putWeightOnToes.get(robotSide).set(true);
   }


   public void unSetPutWeightOnToes(RobotSide robotSide)
   {
      this.putWeightOnToes.get(robotSide).set(false);
   }
   
   public void setUpParametersForR2()
   {
      desiredCaptureForward.set(0.2);    // 0.15;
      desiredCaptureIn.set(0.02);
   }
}
