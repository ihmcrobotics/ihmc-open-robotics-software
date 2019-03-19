package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkRed;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class LinearMomentumRateControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final Vector3DReadOnly linearMomentumRateWeight;
   private final Vector3DReadOnly angularMomentumRateWeight;

   private final YoBoolean minimizingAngularMomentumRateZ = new YoBoolean("MinimizingAngularMomentumRateZ", registry);

   private final YoFrameVector3D controlledCoMAcceleration;

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private double omega0;
   private double totalMass;
   private double gravityZ;

   private final ReferenceFrame centerOfMassFrame;
   private final FramePoint3D centerOfMass;
   private final FramePoint2D centerOfMass2d = new FramePoint2D();

   private final FramePoint2D capturePoint = new FramePoint2D();
   private final FrameVector2D capturePointVelocity = new FrameVector2D();
   private final FramePoint2D desiredCapturePoint = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity = new FrameVector2D();

   private final FixedFramePoint2DBasics perfectCMP = new FramePoint2D();
   private final FixedFramePoint2DBasics perfectCoP = new FramePoint2D();
   private final FixedFramePoint2DBasics desiredCMP = new FramePoint2D();
   private final FixedFramePoint2DBasics desiredCoP = new FramePoint2D();
   private final FixedFramePoint2DBasics achievedCMP = new FramePoint2D();
   private final FixedFramePoint2DBasics desiredCoPInMidFeet;

   private boolean controlHeightWithMomentum;

   private final FrameVector3D achievedLinearMomentumRate = new FrameVector3D();
   private final FrameVector2D achievedCoMAcceleration2d = new FrameVector2D();

   private double desiredCoMHeightAcceleration = 0.0;

   private final FramePoint3D cmp3d = new FramePoint3D();
   private final FrameVector3D linearMomentumRateOfChange = new FrameVector3D();

   private boolean desiredCMPcontainedNaN = false;
   private boolean desiredCoPcontainedNaN = false;

   private final ICPOptimizationControllerInterface icpOptimizationController;
   private final ICPControlPlane icpControlPlane;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPControlPolygons icpControlPolygons;

   private final YoDouble yoTime;

   private final FixedFrameVector2DBasics perfectCMPDelta = new FrameVector2D();

   private RobotSide supportSide = null;
   private RobotSide transferToSide = null;

   private final YoFramePoint2D yoDesiredCMP = new YoFramePoint2D("desiredCMP", worldFrame, registry);
   private final YoFramePoint2D yoAchievedCMP = new YoFramePoint2D("achievedCMP", worldFrame, registry);
   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);

   private final DoubleParameter centerOfPressureWeight = new DoubleParameter("CenterOfPressureObjectiveWeight", registry, 0.0);
   private final CenterOfPressureCommand centerOfPressureCommand = new CenterOfPressureCommand();
   private final ReferenceFrame midFootZUpFrame;

   private boolean initializeForStanding;
   private boolean initializeForSingleSupport;
   private boolean initializeForTransfer;
   private boolean keepCoPInsideSupportPolygon;
   private boolean updatePlanarRegions;
   private double finalTransferDuration;
   private double remainingTimeInSwingUnderDisturbance;
   private final RecyclingArrayList<Footstep> footsteps = new RecyclingArrayList<>(Footstep.class);
   private final RecyclingArrayList<FootstepTiming> footstepTimings = new RecyclingArrayList<>(FootstepTiming.class);
   private final RecyclingArrayList<PlanarRegion> planarRegions = new RecyclingArrayList<>(PlanarRegion.class);

   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());

   private final LinearMomentumRateControlModuleOutput output = new LinearMomentumRateControlModuleOutput();

   public LinearMomentumRateControlModule(CommonHumanoidReferenceFrames referenceFrames, SideDependentList<ContactableFoot> contactableFeet,
                                          WalkingControllerParameters walkingControllerParameters, YoDouble yoTime, double totalMass, double gravityZ,
                                          double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.totalMass = totalMass;
      this.gravityZ = gravityZ;
      this.yoTime = yoTime;

      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      linearMomentumRateWeight = new ParameterVector3D("LinearMomentumRateWeight", momentumOptimizationSettings.getLinearMomentumWeight(), registry);
      angularMomentumRateWeight = new ParameterVector3D("AngularMomentumRateWeight", momentumOptimizationSettings.getAngularMomentumWeight(), registry);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      midFootZUpFrame = referenceFrames.getMidFootZUpGroundFrame();
      centerOfMass = new FramePoint3D(centerOfMassFrame);
      controlledCoMAcceleration = new YoFrameVector3D("ControlledCoMAcceleration", "", centerOfMassFrame, registry);
      desiredCoPInMidFeet = new FramePoint2D(midFootZUpFrame);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", yoDesiredCMP, 0.012, Purple(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition achievedCMPViz = new YoGraphicPosition("Achieved CMP", yoAchievedCMP, 0.005, DarkRed(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition centerOfMassViz = new YoGraphicPosition("Center Of Mass", yoCenterOfMass, 0.006, Black(), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", desiredCMPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", achievedCMPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", centerOfMassViz.createArtifact());
      }
      yoDesiredCMP.setToNaN();
      yoAchievedCMP.setToNaN();
      yoCenterOfMass.setToNaN();

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>(referenceFrames.getSoleZUpFrames());
      icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);
      icpControlPolygons = new ICPControlPolygons(icpControlPlane, midFeetZUpFrame, registry, yoGraphicsListRegistry);
      bipedSupportPolygons = new BipedSupportPolygons(referenceFrames, registry, null); // TODO: This is not being visualized since it is a duplicate for now.
      icpOptimizationController = new ICPOptimizationController(walkingControllerParameters, soleZUpFrames, bipedSupportPolygons, icpControlPolygons,
                                                                contactableFeet, controlDT, registry, yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void setInput(LinearMomentumRateControlModuleInput input)
   {
      this.omega0 = input.getOmega0();
      this.capturePoint.setIncludingFrame(input.getCapturePoint());
      this.capturePointVelocity.setIncludingFrame(input.getCapturePointVelocity());
      this.desiredCapturePoint.setIncludingFrame(input.getDesiredCapturePoint());
      this.desiredCapturePointVelocity.setIncludingFrame(input.getDesiredCapturePointVelocity());
      this.desiredCoMHeightAcceleration = input.getDesiredCoMHeightAcceleration();
      this.minimizingAngularMomentumRateZ.set(input.getMinimizeAngularMomentumRateZ());
      this.perfectCMP.setMatchingFrame(input.getPerfectCMP());
      this.perfectCoP.setMatchingFrame(input.getPerfectCoP());
      this.controlHeightWithMomentum = input.getControlHeightWithMomentum();
      this.supportSide = input.getSupportSide();
      this.transferToSide = input.getTransferToSide();
      this.footsteps.clear();
      for (int i = 0; i < input.getFootsteps().size(); i++)
      {
         this.footsteps.add().set(input.getFootsteps().get(i));
      }
      this.footstepTimings.clear();
      for (int i = 0; i < input.getFootstepTimings().size(); i++)
      {
         this.footstepTimings.add().set(input.getFootstepTimings().get(i));
      }
      this.finalTransferDuration = input.getFinalTransferDuration();
      this.initializeForStanding = input.getInitializeForStanding();
      this.initializeForSingleSupport = input.getInitializeForSingleSupport();
      this.initializeForTransfer = input.getInitializeForTransfer();
      this.remainingTimeInSwingUnderDisturbance = input.getRemainingTimeInSwingUnderDisturbance();
      this.updatePlanarRegions = input.getUpdatePlanarRegions();
      this.planarRegions.clear();
      for (int i = 0; i < input.getPlanarRegions().size(); i++)
      {
         this.planarRegions.add().set(input.getPlanarRegions().get(i));
      }
      this.keepCoPInsideSupportPolygon = input.getKeepCoPInsideSupportPolygon();
      for (RobotSide robotSide : RobotSide.values)
      {
         this.contactStateCommands.get(robotSide).set(input.getContactStateCommands().get(robotSide));
      }
   }

   public void setInput(ControllerCoreOutput controllerCoreOutput)
   {
      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
   }

   public LinearMomentumRateControlModuleOutput getOutput()
   {
      return output;
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   public CenterOfPressureCommand getCenterOfPressureCommand()
   {
      return centerOfPressureCommand;
   }

   public boolean compute()
   {
      boolean success = checkInputs(capturePoint, desiredCapturePoint, desiredCapturePointVelocity, perfectCoP, perfectCMP);

      updatePolygons();
      updateICPControllerState();
      computeICPController();

      checkAndPackOutputs();

      yoDesiredCMP.set(desiredCMP);
      yoAchievedCMP.set(achievedCMP);
      yoCenterOfMass.setFromReferenceFrame(centerOfMassFrame);

      success = success && computeDesiredLinearMomentumRateOfChange();

      selectionMatrix.setToLinearSelectionOnly();
      selectionMatrix.selectLinearZ(controlHeightWithMomentum);
      selectionMatrix.selectAngularZ(minimizingAngularMomentumRateZ.getValue());
      momentumRateCommand.setLinearMomentumRate(linearMomentumRateOfChange);
      momentumRateCommand.setSelectionMatrix(selectionMatrix);
      momentumRateCommand.setWeights(angularMomentumRateWeight, linearMomentumRateWeight);

      desiredCoPInMidFeet.setMatchingFrame(desiredCoP);
      centerOfPressureCommand.setDesiredCoP(desiredCoP);
      centerOfPressureCommand.setWeight(midFootZUpFrame, centerOfPressureWeight.getValue(), centerOfPressureWeight.getValue());

      return success;
   }

   private void updatePolygons()
   {
      icpControlPlane.setOmega0(omega0);
      icpControlPolygons.updateUsingContactStateCommand(contactStateCommands);
      bipedSupportPolygons.updateUsingContactStateCommand(contactStateCommands);
   }

   private void updateICPControllerState()
   {
      if ((initializeForStanding && initializeForTransfer) || (initializeForTransfer && initializeForSingleSupport)
            || (initializeForSingleSupport && initializeForStanding))
      {
         throw new RuntimeException("Can only initialize once per compute.");
      }

      if (initializeForStanding || initializeForTransfer || initializeForSingleSupport)
      {
         icpOptimizationController.clearPlan();
         icpOptimizationController.setFinalTransferDuration(finalTransferDuration);
         for (int i = 0; i < footsteps.size(); i++)
         {
            icpOptimizationController.addFootstepToPlan(footsteps.get(i), footstepTimings.get(i));
         }
      }

      if (initializeForStanding)
      {
         icpOptimizationController.initializeForStanding(yoTime.getValue());
      }
      if (initializeForSingleSupport)
      {
         icpOptimizationController.initializeForSingleSupport(yoTime.getValue(), supportSide, omega0);
      }
      if (initializeForTransfer)
      {
         icpOptimizationController.initializeForTransfer(yoTime.getValue(), transferToSide);
      }

      icpOptimizationController.setKeepCoPInsideSupportPolygon(keepCoPInsideSupportPolygon);

      if (!Double.isNaN(remainingTimeInSwingUnderDisturbance) && remainingTimeInSwingUnderDisturbance > 0.0)
      {
         icpOptimizationController.submitRemainingTimeInSwingUnderDisturbance(remainingTimeInSwingUnderDisturbance);
      }

      if (updatePlanarRegions)
      {
         icpOptimizationController.submitCurrentPlanarRegions(planarRegions);
      }
   }

   private void computeICPController()
   {
      if (perfectCoP.containsNaN())
      {
         perfectCMPDelta.setToZero();
         icpOptimizationController.compute(yoTime.getValue(), desiredCapturePoint, desiredCapturePointVelocity, perfectCMP, capturePoint, capturePointVelocity,
                                           omega0);
      }
      else
      {
         perfectCMPDelta.sub(perfectCMP, perfectCoP);
         icpOptimizationController.compute(yoTime.getValue(), desiredCapturePoint, desiredCapturePointVelocity, perfectCoP, perfectCMPDelta, capturePoint,
                                           capturePointVelocity, omega0);
      }
      icpOptimizationController.getDesiredCMP(desiredCMP);
      icpOptimizationController.getDesiredCoP(desiredCoP);
   }

   private void checkAndPackOutputs()
   {
      if (desiredCMP.containsNaN())
      {
         if (!desiredCMPcontainedNaN)
            LogTools.error("Desired CMP contains NaN, setting it to the ICP - only showing this error once");
         desiredCMP.set(capturePoint);
         desiredCMPcontainedNaN = true;
      }
      else
      {
         desiredCMPcontainedNaN = false;
      }

      if (desiredCoP.containsNaN())
      {
         if (!desiredCoPcontainedNaN)
            LogTools.error("Desired CoP contains NaN, setting it to the desiredCMP - only showing this error once");
         desiredCoP.set(desiredCMP);
         desiredCoPcontainedNaN = true;
      }
      else
      {
         desiredCoPcontainedNaN = false;
      }

      output.setDesiredCMP(desiredCMP);
      output.setEffectiveICPAdjustment(icpOptimizationController.getICPShiftFromStepAdjustment());
      output.setFootstepSolution(icpOptimizationController.getFootstepSolution());
      output.setFootstepWasAdjusted(icpOptimizationController.wasFootstepAdjusted());
      output.setUsingStepAdjustment(icpOptimizationController.useStepAdjustment());
   }

   private boolean computeDesiredLinearMomentumRateOfChange()
   {
      boolean success = true;

      double fZ = WrenchDistributorTools.computeFz(totalMass, gravityZ, desiredCoMHeightAcceleration);
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, desiredCMP, fZ, totalMass, omega0);
      WrenchDistributorTools.computeForce(linearMomentumRateOfChange, centerOfMass, cmp3d, fZ);
      linearMomentumRateOfChange.checkReferenceFrameMatch(centerOfMassFrame);
      linearMomentumRateOfChange.setZ(linearMomentumRateOfChange.getZ() - totalMass * gravityZ);

      if (linearMomentumRateOfChange.containsNaN())
      {
         LogTools.error("Desired LinearMomentumRateOfChange contained NaN, setting it to zero and failing.");
         linearMomentumRateOfChange.setToZero();
         success = false;
      }

      controlledCoMAcceleration.set(linearMomentumRateOfChange);
      controlledCoMAcceleration.scale(1.0 / totalMass);
      linearMomentumRateOfChange.changeFrame(worldFrame);

      return success;
   }

   public void computeAchievedCMP()
   {
      if (achievedLinearMomentumRate.containsNaN())
      {
         yoAchievedCMP.setToNaN();
         return;
      }

      centerOfMass2d.setToZero(centerOfMassFrame);
      centerOfMass2d.changeFrame(worldFrame);

      achievedCoMAcceleration2d.setIncludingFrame(achievedLinearMomentumRate);
      achievedCoMAcceleration2d.scale(1.0 / totalMass);
      achievedCoMAcceleration2d.changeFrame(worldFrame);

      achievedCMP.set(achievedCoMAcceleration2d);
      achievedCMP.scale(-1.0 / (omega0 * omega0));
      achievedCMP.add(centerOfMass2d);
   }

   private static boolean checkInputs(FramePoint2DReadOnly capturePoint, FramePoint2DBasics desiredCapturePoint,
                                      FixedFrameVector2DBasics desiredCapturePointVelocity, FixedFramePoint2DBasics perfectCoP,
                                      FixedFramePoint2DBasics perfectCMP)
   {
      boolean inputsAreOk = true;
      if (desiredCapturePoint.containsNaN())
      {
         LogTools.error("Desired ICP contains NaN, setting it to the current ICP and failing.");
         desiredCapturePoint.set(capturePoint);
         inputsAreOk = false;
      }

      if (desiredCapturePointVelocity.containsNaN())
      {
         LogTools.error("Desired ICP Velocity contains NaN, setting it to zero and failing.");
         desiredCapturePointVelocity.setToZero();
         inputsAreOk = false;
      }

      if (perfectCoP.containsNaN())
      {
         LogTools.error("Perfect CoP contains NaN, setting it to the current ICP and failing.");
         perfectCoP.set(capturePoint);
         inputsAreOk = false;
      }

      if (perfectCMP.containsNaN())
      {
         LogTools.error("Perfect CMP contains NaN, setting it to the current ICP and failing.");
         perfectCMP.set(capturePoint);
         inputsAreOk = false;
      }

      return inputsAreOk;
   }
}
