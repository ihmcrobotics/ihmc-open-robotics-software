package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.output.ControllerStatusOutputManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
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
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class BalanceManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPPlannerWithTimeFreezer icpPlanner;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;

   public BalanceManager(ControllerStatusOutputManager statusOutputManager, MomentumBasedController momentumBasedController,
         WalkingControllerParameters walkingControllerParameters, CapturePointPlannerParameters capturePointPlannerParameters,
         YoVariableRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();

      SideDependentList<ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      SideDependentList<? extends ContactablePlaneBody> contactableFeet = momentumBasedController.getContactableFeet();
      double omega0 = walkingControllerParameters.getOmega0();
      ICPControlGains icpControlGains = walkingControllerParameters.getICPControlGains();
      double controlDT = momentumBasedController.getControlDT();
      double gravityZ = momentumBasedController.getGravityZ();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double minimumSwingTimeForDisturbanceRecovery = walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery();

      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, registry, yoGraphicsListRegistry);

      ICPBasedLinearMomentumRateOfChangeControlModule icpBasedLinearMomentumRateOfChangeControlModule = new ICPBasedLinearMomentumRateOfChangeControlModule(
            referenceFrames, bipedSupportPolygons, controlDT, totalMass, gravityZ, icpControlGains, registry, yoGraphicsListRegistry);

      icpPlanner = new ICPPlannerWithTimeFreezer(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, registry, yoGraphicsListRegistry);
      icpPlanner.setMinimumSingleSupportTimeForDisturbanceRecovery(minimumSwingTimeForDisturbanceRecovery);
      icpPlanner.setOmega0(omega0);
      icpAndMomentumBasedController = new ICPAndMomentumBasedController(momentumBasedController, omega0, icpBasedLinearMomentumRateOfChangeControlModule,
            bipedSupportPolygons, statusOutputManager, parentRegistry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {

   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public EnumYoVariable<RobotSide> getYoSupportLeg()
   {
      return icpAndMomentumBasedController.getYoSupportLeg();
   }

   public YoFramePoint getCapturePoint()
   {
      return icpAndMomentumBasedController.getCapturePoint();
   }

   public YoFramePoint2d getDesiredICP()
   {
      return icpAndMomentumBasedController.getDesiredICP();
   }

   public YoFrameVector2d getDesiredICPVelocity()
   {
      return icpAndMomentumBasedController.getDesiredICPVelocity();
   }

   public DoubleYoVariable getControlledCoMHeightAcceleration()
   {
      return icpAndMomentumBasedController.getControlledCoMHeightAcceleration();
   }

   public void updateBipedSupportPolygons()
   {
      icpAndMomentumBasedController.updateBipedSupportPolygons();
   }

   public void holdCurrentICP(double doubleValue, FramePoint tmpFramePoint)
   {
      icpPlanner.holdCurrentICP(doubleValue, tmpFramePoint);
   }

   public void getDesiredCapturePointPositionAndVelocity(FramePoint2d desiredCapturePointPositionToPack, FrameVector2d desiredCapturePointVelocityToPack,
         FramePoint2d currentCapturePointPosition, double time)
   {
      icpPlanner.getDesiredCapturePointPositionAndVelocity(desiredCapturePointPositionToPack, desiredCapturePointVelocityToPack, currentCapturePointPosition,
            time);
   }

   public double getOmega0()
   {
      return icpAndMomentumBasedController.getOmega0();
   }

   public boolean isDone(double doubleValue)
   {
      return icpPlanner.isDone(doubleValue);
   }

   public void getDesiredCMP(FramePoint2d desiredCMP)
   {
      icpAndMomentumBasedController.getDesiredCMP(desiredCMP);
   }

   public double computeAndReturnTimeRemaining(double time)
   {
      return icpPlanner.computeAndReturnTimeInCurrentState(time);
   }

   public void clearPlan()
   {
      icpPlanner.clearPlan();
   }

   public double getInitialTransferDuration()
   {
      return icpPlanner.getInitialTransferDuration();
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      icpPlanner.addFootstepToPlan(footstep);
   }

   public void initializeDoubleSupport(double initialTime, RobotSide transferToSide)
   {
      icpPlanner.initializeDoubleSupport(initialTime, transferToSide);
   }

   public void setSingleSupportTime(double time)
   {
      icpPlanner.setSingleSupportTime(time);
   }

   public void reset(double time)
   {
      icpPlanner.reset(time);
   }

   public void getFinalDesiredCapturePointPosition(FramePoint2d finalDesiredCapturePointPositionToPack)
   {
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePointPositionToPack);
   }

   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePointPositionToPack);
   }

   public void updatePlanForSingleSupportDisturbances(double time, FramePoint actualCapturePointPosition)
   {
      icpPlanner.updatePlanForSingleSupportDisturbances(time, actualCapturePointPosition);
   }

   public boolean isOnExitCMP()
   {
      return icpPlanner.isOnExitCMP();
   }

   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      icpPlanner.getNextExitCMP(entryCMPToPack);
   }

   public double estimateTimeRemainingForStateUnderDisturbance(double time, FramePoint actualCapturePointPosition)
   {
      return icpPlanner.estimateTimeRemainingForStateUnderDisturbance(time, actualCapturePointPosition);
   }

   public void initializeSingleSupport(double initialTime, RobotSide supportSide)
   {
      icpPlanner.initializeSingleSupport(initialTime, supportSide);
   }

   public void setDoubleSupportTime(double time)
   {
      icpPlanner.setDoubleSupportTime(time);
   }

   public void update()
   {
      icpAndMomentumBasedController.update();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return icpAndMomentumBasedController.getInverseDynamicsCommand();
   }

   public void compute(FramePoint2d finalDesiredCapturePoint2d, boolean keepCMPInsideSupportPolygon)
   {
      icpAndMomentumBasedController.compute(finalDesiredCapturePoint2d, keepCMPInsideSupportPolygon);
   }
}
