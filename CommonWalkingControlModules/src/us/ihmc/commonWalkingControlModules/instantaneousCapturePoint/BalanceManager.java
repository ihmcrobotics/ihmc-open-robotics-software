package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableGoHomeMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableStopAllTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.output.ControllerStatusOutputManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class BalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPPlannerWithTimeFreezer icpPlanner;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   private final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;

   private final YoFramePoint2d finalDesiredICPInWorld = new YoFramePoint2d("finalDesiredICPInWorld", "", worldFrame, registry);

   private final YoFramePoint2d desiredECMP = new YoFramePoint2d("desiredECMP", "", worldFrame, registry);
   private final YoFramePoint ecmpViz = new YoFramePoint("ecmpViz", worldFrame, registry);

   public BalanceManager(ControllerStatusOutputManager statusOutputManager, MomentumBasedController momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, CapturePointPlannerParameters capturePointPlannerParameters,
         YoVariableRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();

      SideDependentList<ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      SideDependentList<ReferenceFrame> soleZUpFrames = referenceFrames.getSoleZUpFrames();
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      SideDependentList<? extends ContactablePlaneBody> contactableFeet = momentumBasedController.getContactableFeet();
      double omega0 = walkingControllerParameters.getOmega0();
      ICPControlGains icpControlGains = walkingControllerParameters.getICPControlGains();
      double controlDT = momentumBasedController.getControlDT();
      double gravityZ = momentumBasedController.getGravityZ();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double minimumSwingTimeForDisturbanceRecovery = walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery();

      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, registry, yoGraphicsListRegistry);

      ICPBasedLinearMomentumRateOfChangeControlModule icpBasedLinearMomentumRateOfChangeControlModule = new ICPBasedLinearMomentumRateOfChangeControlModule(
            referenceFrames, bipedSupportPolygons, controlDT, totalMass, gravityZ, icpControlGains, registry, yoGraphicsListRegistry);

      icpPlanner = new ICPPlannerWithTimeFreezer(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, registry, yoGraphicsListRegistry);
      icpPlanner.setMinimumSingleSupportTimeForDisturbanceRecovery(minimumSwingTimeForDisturbanceRecovery);
      icpPlanner.setOmega0(omega0);
      icpPlanner.setSingleSupportTime(walkingControllerParameters.getDefaultSwingTime());
      icpPlanner.setDoubleSupportTime(walkingControllerParameters.getDefaultTransferTime());
      icpAndMomentumBasedController = new ICPAndMomentumBasedController(momentumBasedController, omega0, icpBasedLinearMomentumRateOfChangeControlModule,
            bipedSupportPolygons, statusOutputManager, parentRegistry);
      YoPDGains pelvisXYControlGains = walkingControllerParameters.createPelvisICPBasedXYControlGains(registry);
      pelvisICPBasedTranslationManager = new PelvisICPBasedTranslationManager(momentumBasedController, pelvisXYControlGains, registry);

      YoGraphicPosition dynamicGraphicPositionECMP = new YoGraphicPosition("ecmpviz", ecmpViz, 0.002, YoAppearance.BlueViolet());
      yoGraphicsListRegistry.registerYoGraphic("ecmpviz", dynamicGraphicPositionECMP);
      yoGraphicsListRegistry.registerArtifact("ecmpviz", dynamicGraphicPositionECMP.createArtifact());

      parentRegistry.addChild(registry);
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      icpPlanner.addFootstepToPlan(footstep);
   }

   public void disablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.disable();
   }

   public void enablePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.enable();
   }

   public void freezePelvisXYControl()
   {
      pelvisICPBasedTranslationManager.freeze();
   }

   public void clearPlan()
   {
      icpPlanner.clearPlan();
   }

   public void computePelvisXY(FramePoint2d desiredICPToModify, FrameVector2d desiredICPVelocityToModify, FrameConvexPolygon2d supportPolygon)
   {
      RobotSide supportLeg = getYoSupportLeg().getEnumValue();
      icpAndMomentumBasedController.getCapturePoint(actualCapturePointPosition);
      pelvisICPBasedTranslationManager.compute(supportLeg, actualCapturePointPosition);
      pelvisICPBasedTranslationManager.addICPOffset(desiredICPToModify, desiredICPVelocityToModify, supportPolygon);
   }

   private final FramePoint2d finalDesiredCapturePoint2d = new FramePoint2d();

   public void compute(boolean keepCMPInsideSupportPolygon)
   {
      finalDesiredICPInWorld.getFrameTuple2dIncludingFrame(finalDesiredCapturePoint2d);
      icpAndMomentumBasedController.compute(finalDesiredCapturePoint2d, keepCMPInsideSupportPolygon);
   }

   public double computeAndReturnTimeRemaining(double time)
   {
      return icpPlanner.computeAndReturnTimeInCurrentState(time);
   }

   private final FramePoint2d actualCapturePointPosition = new FramePoint2d();

   public double estimateTimeRemainingForStateUnderDisturbance(double time)
   {
      icpAndMomentumBasedController.getCapturePoint(actualCapturePointPosition);
      return icpPlanner.estimateTimeRemainingForStateUnderDisturbance(time, actualCapturePointPosition);
   }

   public void handlePelvisTrajectoryMessage(ModifiablePelvisTrajectoryMessage message)
   {
      pelvisICPBasedTranslationManager.handlePelvisTrajectoryMessage(message);
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public YoFramePoint getCapturePoint()
   {
      return icpAndMomentumBasedController.getCapturePoint();
   }

   public void getCapturePoint(FramePoint2d capturePointToPack)
   {
      icpAndMomentumBasedController.getCapturePoint(capturePointToPack);
   }

   public DoubleYoVariable getControlledCoMHeightAcceleration()
   {
      return icpAndMomentumBasedController.getControlledCoMHeightAcceleration();
   }

   public void getDesiredCapturePointPositionAndVelocity(FramePoint2d desiredCapturePointPositionToPack, FrameVector2d desiredCapturePointVelocityToPack,
         double time)
   {
      icpAndMomentumBasedController.getCapturePoint(actualCapturePointPosition);
      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, actualCapturePointPosition,
            time);
   }

   public void getDesiredCMP(FramePoint2d desiredCMP)
   {
      icpAndMomentumBasedController.getDesiredCMP(desiredCMP);
   }

   public void setDesiredICP(FramePoint2d desiredCapturePoint)
   {
      icpAndMomentumBasedController.setDesiredICP(desiredCapturePoint);
   }

   public YoFramePoint2d getDesiredICP()
   {
      return icpAndMomentumBasedController.getDesiredICP();
   }

   public void getDesiredICP(FramePoint2d desiredICP)
   {
      icpAndMomentumBasedController.getDesiredICP(desiredICP);
   }

   public YoFrameVector2d getDesiredICPVelocity()
   {
      return icpAndMomentumBasedController.getDesiredICPVelocity();
   }

   public void getDesiredICPVelocity(FrameVector2d desiredICPToPack)
   {
      icpAndMomentumBasedController.getDesiredICPVelocity(desiredICPToPack);
   }

   public void setDesiredICPVelocity(FrameVector2d desiredICP)
   {
      icpAndMomentumBasedController.setDesiredICPVelocity(desiredICP);
   }

   public void getFinalDesiredCapturePointPosition(FramePoint2d finalDesiredCapturePointPositionToPack)
   {
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePointPositionToPack);
   }

   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePointPositionToPack);
   }

   public double getInitialTransferDuration()
   {
      return icpPlanner.getInitialTransferDuration();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return icpAndMomentumBasedController.getInverseDynamicsCommand();
   }

   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      icpPlanner.getNextExitCMP(entryCMPToPack);
   }

   public double getOmega0()
   {
      return icpAndMomentumBasedController.getOmega0();
   }

   public EnumYoVariable<RobotSide> getYoSupportLeg()
   {
      return icpAndMomentumBasedController.getYoSupportLeg();
   }

   public void holdCurrentICP(double doubleValue, FramePoint tmpFramePoint)
   {
      icpPlanner.holdCurrentICP(doubleValue, tmpFramePoint);
   }

   public void initialize()
   {
      finalDesiredICPInWorld.set(Double.NaN, Double.NaN);
      icpAndMomentumBasedController.getDesiredICP().setByProjectionOntoXYPlane(icpAndMomentumBasedController.getCapturePoint());
   }

   public void initializeDoubleSupport(double initialTime, RobotSide transferToSide)
   {
      icpPlanner.initializeDoubleSupport(initialTime, transferToSide);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredICPInWorld);
   }

   public void initializeSingleSupport(double initialTime, RobotSide supportSide)
   {
      icpPlanner.initializeSingleSupport(initialTime, supportSide);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredICPInWorld);
   }

   public boolean isDone(double doubleValue)
   {
      return icpPlanner.isDone(doubleValue);
   }

   public boolean isOnExitCMP()
   {
      return icpPlanner.isOnExitCMP();
   }

   public void reset(double time)
   {
      icpPlanner.reset(time);
   }

   public void setDoubleSupportTime(double time)
   {
      icpPlanner.setDoubleSupportTime(time);
   }

   public void setSingleSupportTime(double time)
   {
      icpPlanner.setSingleSupportTime(time);
   }

   public void update()
   {
      YoFramePoint2d desiredICP = icpAndMomentumBasedController.getDesiredICP();
      YoFrameVector2d desiredICPVelocity = icpAndMomentumBasedController.getDesiredICPVelocity();
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, getOmega0(), desiredECMP);
      ecmpViz.setXY(desiredECMP);
      icpAndMomentumBasedController.update();
   }

   public void updateBipedSupportPolygons()
   {
      icpAndMomentumBasedController.updateBipedSupportPolygons();
   }

   public void updatePlanForSingleSupportDisturbances(double time)
   {
      icpAndMomentumBasedController.getCapturePoint(actualCapturePointPosition);
      icpPlanner.updatePlanForSingleSupportDisturbances(time, actualCapturePointPosition);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredICPInWorld);
   }

   public void handleGoHomeMessage(ModifiableGoHomeMessage message)
   {
      pelvisICPBasedTranslationManager.handleGoHomeMessage(message);
   }

   public void handleStopAllTrajectoryMessage(ModifiableStopAllTrajectoryMessage message)
   {
      pelvisICPBasedTranslationManager.handleStopAllTrajectoryMessage(message);
   }
}
