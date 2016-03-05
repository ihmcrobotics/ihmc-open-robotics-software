package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerAPI.output.ControllerStatusOutputManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPBasedLinearMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class ICPAndMomentumBasedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame centerOfMassFrame;
   private final MomentumBasedController momentumBasedController;

   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoFramePoint2d yoDesiredCapturePoint = new YoFramePoint2d("desiredICP", "", worldFrame, registry);
   private final YoFrameVector2d yoDesiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", worldFrame, registry);
   private final EnumYoVariable<RobotSide> supportLeg = new EnumYoVariable<>("supportLeg", registry, RobotSide.class, true);
   private final DoubleYoVariable controlledCoMHeightAcceleration = new DoubleYoVariable("controlledCoMHeightAcceleration", registry);
   private final YoFramePoint yoCapturePoint = new YoFramePoint("capturePoint", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable("omega0", registry);

   private final FramePoint centerOfMassPosition = new FramePoint(worldFrame);
   private final FrameVector centerOfMassVelocity = new FrameVector(worldFrame);
   private final FramePoint2d centerOfMassPosition2d = new FramePoint2d(worldFrame);
   private final FrameVector2d centerOfMassVelocity2d = new FrameVector2d(worldFrame);

   private final FramePoint2d capturePoint2d = new FramePoint2d(worldFrame);
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity2d = new FrameVector2d();

   private final ControllerStatusOutputManager statusOutputManager;
   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

   private final ICPBasedLinearMomentumRateOfChangeControlModule icpBasedLinearMomentumRateOfChangeControlModule;

   public ICPAndMomentumBasedController(MomentumBasedController momentumBasedController, double omega0,
         ICPBasedLinearMomentumRateOfChangeControlModule icpBasedLinearMomentumRateOfChangeControlModule, BipedSupportPolygons bipedSupportPolygons,
         ControllerStatusOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      this.momentumBasedController = momentumBasedController;
      this.statusOutputManager = statusOutputManager;
      this.icpBasedLinearMomentumRateOfChangeControlModule = icpBasedLinearMomentumRateOfChangeControlModule;
      this.omega0.set(omega0);

      centerOfMassFrame = momentumBasedController.getCenterOfMassFrame();

      this.bipedSupportPolygons = bipedSupportPolygons;

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
         yoGraphicsListRegistry.registerYoGraphic("Capture Point", capturePointViz);
         yoGraphicsListRegistry.registerArtifact("Capture Point", capturePointViz.createArtifact());
      }
   }

   public void initialize()
   {
      update();
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   /**
    * Update the basics: capture point, omega0, and the support polygons.
    */
   public void update()
   {
      computeCapturePoint();
      icpBasedLinearMomentumRateOfChangeControlModule.updateCenterOfMassViz();
   }

   private void computeCapturePoint()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);
      momentumBasedController.getCenterOfMassVelocity(centerOfMassVelocity);

      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMassPosition2d.setByProjectionOntoXYPlane(centerOfMassPosition);
      centerOfMassVelocity2d.setByProjectionOntoXYPlane(centerOfMassVelocity);

      CapturePointCalculator.computeCapturePoint(capturePoint2d, centerOfMassPosition2d, centerOfMassVelocity2d, getOmega0());

      capturePoint2d.changeFrame(yoCapturePoint.getReferenceFrame());
      yoCapturePoint.setXY(capturePoint2d);
   }

   public void updateBipedSupportPolygons(SideDependentList<? extends PlaneContactState> footContactStates)
   {
      bipedSupportPolygons.updateUsingContactStates(footContactStates);

      reportCapturabilityBasedStatus();
   }

   private void reportCapturabilityBasedStatus()
   {
      yoDesiredCapturePoint.getFrameTuple2dIncludingFrame(desiredCapturePoint2d);
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassPosition.changeFrame(worldFrame);

      capturePoint2d.checkReferenceFrameMatch(worldFrame);
      desiredCapturePoint2d.checkReferenceFrameMatch(worldFrame);

      SideDependentList<FrameConvexPolygon2d> footSupportPolygons = bipedSupportPolygons.getFootPolygonsInWorldFrame();

      capturePoint2d.get(capturabilityBasedStatus.capturePoint);
      desiredCapturePoint2d.get(capturabilityBasedStatus.desiredCapturePoint);
      centerOfMassPosition.get(capturabilityBasedStatus.centerOfMass);
      for (RobotSide robotSide : RobotSide.values)
      {
         capturabilityBasedStatus.setSupportPolygon(robotSide, footSupportPolygons.get(robotSide));
      }

      statusOutputManager.reportCapturabilityBasedStatus(capturabilityBasedStatus);
   }

   public void compute(FramePoint2d finalDesiredCapturePoint2d, boolean keepCMPInsideSupportPolygon)
   {
      yoCapturePoint.getFrameTuple2dIncludingFrame(capturePoint2d);
      yoDesiredCapturePoint.getFrameTuple2dIncludingFrame(desiredCapturePoint2d);
      yoDesiredICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocity2d);

      icpBasedLinearMomentumRateOfChangeControlModule.keepCMPInsideSupportPolygon(keepCMPInsideSupportPolygon);

      icpBasedLinearMomentumRateOfChangeControlModule.setCapturePoint(capturePoint2d);
      icpBasedLinearMomentumRateOfChangeControlModule.setOmega0(omega0.getDoubleValue());

      icpBasedLinearMomentumRateOfChangeControlModule.setDesiredCapturePoint(desiredCapturePoint2d);
      icpBasedLinearMomentumRateOfChangeControlModule.setFinalDesiredCapturePoint(finalDesiredCapturePoint2d);
      icpBasedLinearMomentumRateOfChangeControlModule.setDesiredCapturePointVelocity(desiredCapturePointVelocity2d);

      icpBasedLinearMomentumRateOfChangeControlModule.setSupportLeg(supportLeg.getEnumValue());

      icpBasedLinearMomentumRateOfChangeControlModule.setDesiredCenterOfMassHeightAcceleration(controlledCoMHeightAcceleration.getDoubleValue());

      icpBasedLinearMomentumRateOfChangeControlModule.compute();
   }

   public EnumYoVariable<RobotSide> getYoSupportLeg()
   {
      return supportLeg;
   }

   public DoubleYoVariable getControlledCoMHeightAcceleration()
   {
      return controlledCoMHeightAcceleration;
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public YoFramePoint getCapturePoint()
   {
      return yoCapturePoint;
   }

   public void getCapturePoint(FramePoint2d capturePointToPack)
   {
      yoCapturePoint.getFrameTuple2dIncludingFrame(capturePointToPack);
   }

   public YoFramePoint2d getDesiredICP()
   {
      return yoDesiredCapturePoint;
   }

   public void setDesiredICP(FramePoint2d desiredCapturePoint)
   {
      yoDesiredCapturePoint.set(desiredCapturePoint);
   }

   public void getDesiredICP(FramePoint2d pointToPack)
   {
      this.yoDesiredCapturePoint.getFrameTuple2dIncludingFrame(pointToPack);
   }

   public YoFrameVector2d getDesiredICPVelocity()
   {
      return yoDesiredICPVelocity;
   }

   public void getDesiredICPVelocity(FrameVector2d desiredICPToPack)
   {
      yoDesiredICPVelocity.getFrameTuple2dIncludingFrame(desiredICPToPack);
   }

   public void setDesiredICPVelocity(FrameVector2d desiredICP)
   {
      yoDesiredICPVelocity.set(desiredICP);
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      icpBasedLinearMomentumRateOfChangeControlModule.getDesiredCMP(desiredCMPToPack);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return icpBasedLinearMomentumRateOfChangeControlModule.getMomentumRateCommand();
   }
}
