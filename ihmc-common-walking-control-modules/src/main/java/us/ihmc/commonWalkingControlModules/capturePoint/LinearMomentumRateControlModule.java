package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.HeuristicICPController;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPController;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerInterface;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumControlCore.CoMHeightController;
import us.ihmc.commonWalkingControlModules.momentumControlCore.HeightController;
import us.ihmc.commonWalkingControlModules.momentumControlCore.PelvisHeightController;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector2d;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint2D;

public class LinearMomentumRateControlModule implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Vector3DReadOnly linearMomentumRateWeight;
   private final Vector3DReadOnly recoveryLinearMomentumRateWeight;
   private final Vector3DReadOnly angularMomentumRateWeight;

   private final BooleanProvider allowMomentumRecoveryWeight;
   private final YoBoolean useRecoveryMomentumWeight;
   private final DoubleParameter maxMomentumRateWeightChangeRate;
   private final RateLimitedYoFrameVector desiredLinearMomentumRateWeight;

   private final YoBoolean minimizingAngularMomentumRateZ = new YoBoolean("MinimizingAngularMomentumRateZ", registry);

   private final YoFrameVector3D controlledCoMAcceleration;

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final PelvisHeightController pelvisHeightController;
   private final CoMHeightController comHeightController;

   private boolean hasHeightCommand = true;
   private FeedbackControlCommand<?> heightControlCommand;

   private double omega0;
   private final DoubleProvider totalMassProvider;
   private final double gravityZ;

   private final ReferenceFrame centerOfMassFrame;
   private final FramePoint3D centerOfMass;
   private final FramePoint2D centerOfMass2d = new FramePoint2D();

   private final FramePoint2D capturePoint = new FramePoint2D();
   private final CapturePointCalculator capturePointCalculator;

   private final FixedFramePoint2DBasics desiredCapturePoint = new FramePoint2D();
   private final FixedFrameVector2DBasics desiredCapturePointVelocity = new FrameVector2D();
   private final FixedFramePoint2DBasics desiredCapturePointAtEndOfState = new FramePoint2D();

   private final FixedFramePoint2DBasics perfectCMP = new FramePoint2D();
   private final FixedFramePoint2DBasics perfectCoP = new FramePoint2D();
   private final FixedFramePoint2DBasics desiredCMP = new FramePoint2D();
   private final FixedFramePoint2DBasics desiredCoP = new FramePoint2D();
   private final FixedFramePoint2DBasics achievedCMP = new FramePoint2D();

   private final FrameVector3D achievedLinearMomentumRate = new FrameVector3D();
   private final FrameVector2D achievedCoMAcceleration2d = new FrameVector2D();

   private double desiredCoMHeightAcceleration = 0.0;

   private final FramePoint3D cmp3d = new FramePoint3D();
   private final FrameVector3D linearMomentumRateOfChange = new FrameVector3D();

   private boolean desiredCMPcontainedNaN = false;
   private boolean desiredCoPcontainedNaN = false;

   private final ICPControllerInterface icpController;

   private final ICPControlPlane icpControlPlane;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPControlPolygons icpControlPolygons;

   private final FixedFrameVector2DBasics perfectCMPDelta = new FrameVector2D();

   private final YoBoolean controlHeightWithMomentum = new YoBoolean("controlHeightWithMomentum", registry);
   private final YoFramePoint2D yoDesiredCMP = new YoFramePoint2D("desiredCMP", worldFrame, registry);
   private final YoFramePoint2D yoDesiredCoP = new YoFramePoint2D("desiredCoP", worldFrame, registry);
   private final YoFramePoint2D yoAchievedCMP = new YoFramePoint2D("achievedCMP", worldFrame, registry);
   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);
   private final YoFramePoint2D yoCapturePoint = new YoFramePoint2D("capturePoint", worldFrame, registry);

   private final FilteredVelocityYoFrameVector2d capturePointVelocity;
   private final BooleanProvider useCenterOfPressureCommandOnly = new BooleanParameter("useCenterOfPressureCommandOnly", registry, false);
   private final DoubleProvider capturePointVelocityBreakFrequency = new DoubleParameter("capturePointVelocityBreakFrequency", registry, 26.5);

   private final CenterOfPressureCommand centerOfPressureCommand = new CenterOfPressureCommand();

   private boolean initializeOnStateChange;
   private FrameConvexPolygon2DReadOnly multiContactStabilityRegion;

   private final CenterOfPressureCommandCalculator centerOfPressureCommandCalculator;

   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());

   private final LinearMomentumRateControlModuleOutput output = new LinearMomentumRateControlModuleOutput();

   public LinearMomentumRateControlModule(HighLevelHumanoidControllerToolbox controllerToolbox,
                                          WalkingControllerParameters walkingControllerParameters,
                                          YoRegistry parentRegistry)
   {
      this(controllerToolbox,
           controllerToolbox.getReferenceFrames(),
           controllerToolbox.getContactableFeet(),
           controllerToolbox.getFullRobotModel().getElevator(),
           walkingControllerParameters,
           controllerToolbox.getTotalMassProvider(),
           controllerToolbox.getGravityZ(),
           controllerToolbox.getControlDT(),
           parentRegistry,
           controllerToolbox.getYoGraphicsListRegistry());
   }

   public LinearMomentumRateControlModule(CenterOfMassStateProvider centerOfMassStateProvider,
                                          CommonHumanoidReferenceFrames referenceFrames,
                                          SideDependentList<ContactableFoot> contactableFeet,
                                          RigidBodyBasics elevator,
                                          WalkingControllerParameters walkingControllerParameters,
                                          DoubleProvider totalMassProvider,
                                          double gravityZ,
                                          double controlDT,
                                          YoRegistry parentRegistry,
                                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.totalMassProvider = totalMassProvider;
      this.gravityZ = gravityZ;

      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      linearMomentumRateWeight = new ParameterVector3D("LinearMomentumRateWeight", momentumOptimizationSettings.getLinearMomentumWeight(), registry);
      recoveryLinearMomentumRateWeight = new ParameterVector3D("RecoveryLinearMomentumRateWeight",
                                                               momentumOptimizationSettings.getRecoveryLinearMomentumWeight(),
                                                               registry);
      angularMomentumRateWeight = new ParameterVector3D("AngularMomentumRateWeight", momentumOptimizationSettings.getAngularMomentumWeight(), registry);

      allowMomentumRecoveryWeight = new BooleanParameter("allowMomentumRecoveryWeight", registry, false);
      maxMomentumRateWeightChangeRate = new DoubleParameter("maxMomentumRateWeightChangeRate", registry, 10.0);
      useRecoveryMomentumWeight = new YoBoolean("useRecoveryMomentumWeight", registry);
      useRecoveryMomentumWeight.set(false);
      desiredLinearMomentumRateWeight = new RateLimitedYoFrameVector("desiredLinearMomentumRateWeight",
                                                                     "",
                                                                     registry,
                                                                     maxMomentumRateWeightChangeRate,
                                                                     controlDT,
                                                                     worldFrame);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      centerOfMass = new FramePoint3D(centerOfMassFrame);
      controlledCoMAcceleration = new YoFrameVector3D("ControlledCoMAcceleration", "", centerOfMassFrame, registry);

      capturePointCalculator = new CapturePointCalculator(centerOfMassStateProvider);
      centerOfPressureCommandCalculator = new CenterOfPressureCommandCalculator(referenceFrames.getMidFeetZUpFrame(), contactableFeet, registry);

      pelvisHeightController = new PelvisHeightController(referenceFrames.getPelvisFrame(), elevator.getBodyFixedFrame(), registry);
      comHeightController = new CoMHeightController(centerOfMassStateProvider, registry);

      DoubleProvider capturePointVelocityAlpha = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(capturePointVelocityBreakFrequency.getValue(),
                                                                                                                       controlDT);
      capturePointVelocity = new FilteredVelocityYoFrameVector2d("capturePointVelocity", "", capturePointVelocityAlpha, controlDT, registry, worldFrame);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", yoDesiredCMP, 0.012, Purple(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition desiredCoPViz = new YoGraphicPosition("Desired CoP", yoDesiredCoP, 0.0075, Purple(), GraphicType.SOLID_BALL);
         YoGraphicPosition achievedCMPViz = new YoGraphicPosition("Achieved CMP", yoAchievedCMP, 0.005, DarkRed(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition centerOfMassViz = new YoGraphicPosition("Center Of Mass", yoCenterOfMass, 0.006, Black(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", desiredCMPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", desiredCoPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", achievedCMPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", centerOfMassViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", capturePointViz.createArtifact());
      }
      yoDesiredCMP.setToNaN();
      yoDesiredCoP.setToNaN();
      yoAchievedCMP.setToNaN();
      yoCenterOfMass.setToNaN();
      yoCenterOfMassVelocity.setToNaN();
      yoCapturePoint.setToNaN();

      icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);
      icpControlPolygons = new ICPControlPolygons(icpControlPlane, registry, yoGraphicsListRegistry);
      /*
       * TODO: The following is not being visualized since it is a duplicate for now.
       */
      bipedSupportPolygons = new BipedSupportPolygons(referenceFrames, registry, null);

      ICPControllerParameters icpControllerParameters = walkingControllerParameters.getICPControllerParameters();

      if (icpControllerParameters.getUseHeuristicICPController())
      {
         icpController = new HeuristicICPController(icpControllerParameters, controlDT, registry, yoGraphicsListRegistry);
      }

      else
      {
         icpController = new ICPController(walkingControllerParameters, icpControlPolygons, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      desiredLinearMomentumRateWeight.set(linearMomentumRateWeight);

      capturePointVelocity.reset();
      yoDesiredCMP.setToNaN();
      yoDesiredCoP.setToNaN();
      yoAchievedCMP.setToNaN();
      yoCenterOfMass.setToNaN();
      yoCenterOfMassVelocity.setToNaN();
      yoCapturePoint.setToNaN();
   }

   /**
    * Sets the input to this module that is being computed by the walking state machine.
    * <p>
    * Must be called before {@link #computeControllerCoreCommands()}.
    *
    * @param input containing quantities such as the desired ICP.
    */
   public void setInputFromWalkingStateMachine(LinearMomentumRateControlModuleInput input)
   {
      this.omega0 = input.getOmega0();
      heightControlCommand = null;
      hasHeightCommand = input.getHasHeightCommand();
      if (hasHeightCommand)
      {
         if (input.getUsePelvisHeightCommand())
            heightControlCommand = input.getPelvisHeightControlCommand();
         else
            heightControlCommand = input.getCenterOfMassHeightControlCommand();
      }
      this.useRecoveryMomentumWeight.set(input.getUseMomentumRecoveryMode());
      this.desiredCapturePoint.setMatchingFrame(input.getDesiredCapturePoint());
      this.desiredCapturePointVelocity.setMatchingFrame(input.getDesiredCapturePointVelocity());
      this.desiredCapturePointAtEndOfState.setMatchingFrame(input.getDesiredCapturePointAtEndOfState());
      this.minimizingAngularMomentumRateZ.set(input.getMinimizeAngularMomentumRateZ());
      this.perfectCMP.setMatchingFrame(input.getPerfectCMP());
      this.perfectCoP.setMatchingFrame(input.getPerfectCoP());
      this.controlHeightWithMomentum.set(input.getControlHeightWithMomentum());
      this.initializeOnStateChange = input.getInitializeOnStateChange();
      this.multiContactStabilityRegion = input.getMultiContactStabilityRegion();
      for (RobotSide robotSide : RobotSide.values)
      {
         this.contactStateCommands.get(robotSide).set(input.getContactStateCommands().get(robotSide));
      }
   }

   /**
    * Sets the input to this module that is being computed by the controller core.
    * <p>
    * Must be called before {@link #computeAchievedCMP()}.
    *
    * @param controllerCoreOutput containing the achieved linear momentum rate.
    */
   public void setInputFromControllerCore(ControllerCoreOutput controllerCoreOutput)
   {
      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
   }

   /**
    * Gets the output of this module that will be used by the walking state machine to adjust footsteps
    * and check transition conditions.
    *
    * @return output data of this module meant for the walking state machine.
    */
   public LinearMomentumRateControlModuleOutput getOutputForWalkingStateMachine()
   {
      return output;
   }

   /**
    * Gets the momentum rate command for the controller core.
    *
    * @return momentum rate command.
    */
   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   /**
    * Gets the center of pressure command for the controller core.
    *
    * @return center of pressure command.
    */
   public CenterOfPressureCommand getCenterOfPressureCommand()
   {
      return centerOfPressureCommand;
   }

   /**
    * Computes the {@link MomentumRateCommand} and the {@link CenterOfPressureCommand} for the
    * controller core.
    * <p>
    * This methods requires that the input to this module from the walking state machine is set via
    * {@link #setInputFromWalkingStateMachine(LinearMomentumRateControlModuleInput)} which provides
    * quantities such as the desired ICP.
    *
    * @return whether the computation was successful.
    */
   public boolean computeControllerCoreCommands()
   {
      capturePointCalculator.compute(capturePoint, omega0);
      capturePointVelocity.update(capturePoint);

      boolean success = checkInputs(capturePoint, desiredCapturePoint, desiredCapturePointVelocity, perfectCoP, perfectCMP);

      updatePolygons();
      updateHeightController();
      updateICPControllerState();
      computeICPController();

      checkAndPackOutputs();

      if (allowMomentumRecoveryWeight.getValue() && useRecoveryMomentumWeight.getBooleanValue())
         desiredLinearMomentumRateWeight.update(recoveryLinearMomentumRateWeight);
      else
         desiredLinearMomentumRateWeight.update(linearMomentumRateWeight);

      yoDesiredCMP.set(desiredCMP);
      yoDesiredCoP.set(desiredCoP);
      yoCenterOfMass.setFromReferenceFrame(centerOfMassFrame);
      yoCenterOfMassVelocity.set(capturePointCalculator.getCenterOfMassVelocity());
      yoCapturePoint.set(capturePoint);

      success = success && computeDesiredLinearMomentumRateOfChange();

      selectionMatrix.setToLinearSelectionOnly();
      selectionMatrix.selectLinearX(!useCenterOfPressureCommandOnly.getValue());
      selectionMatrix.selectLinearY(!useCenterOfPressureCommandOnly.getValue());
      selectionMatrix.selectLinearZ(controlHeightWithMomentum.getBooleanValue());
      selectionMatrix.selectAngularZ(minimizingAngularMomentumRateZ.getValue());
      momentumRateCommand.setLinearMomentumRate(linearMomentumRateOfChange);
      momentumRateCommand.setSelectionMatrix(selectionMatrix);
      momentumRateCommand.setWeights(angularMomentumRateWeight, desiredLinearMomentumRateWeight);

      centerOfPressureCommandCalculator.computeCenterOfPressureCommand(desiredCoP, contactStateCommands, bipedSupportPolygons.getFootPolygonsInSoleFrame());

      return success;
   }

   /**
    * Computes the achieved CMP location.
    * <p>
    * This method requires that the input to this module from the controller core is set via
    * {@link #setInputFromControllerCore(ControllerCoreOutput)} to provide the achieved linear momentum
    * rate.
    */
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
      achievedCoMAcceleration2d.scale(1.0 / totalMassProvider.getValue());
      achievedCoMAcceleration2d.changeFrame(worldFrame);

      achievedCMP.set(achievedCoMAcceleration2d);
      achievedCMP.scale(-1.0 / (omega0 * omega0));
      achievedCMP.add(centerOfMass2d);

      yoAchievedCMP.set(achievedCMP);
   }

   private void updatePolygons()
   {
      icpControlPlane.setOmega0(omega0);
      icpControlPolygons.updateUsingContactStateCommand(contactStateCommands);
      bipedSupportPolygons.updateUsingContactStateCommand(contactStateCommands);
   }

   private void updateHeightController()
   {
      if (!hasHeightCommand || heightControlCommand == null)
      {
         desiredCoMHeightAcceleration = 0.0;
         return;
      }

      switch (heightControlCommand.getCommandType())
      {
         case POINT:
            desiredCoMHeightAcceleration = handleHeightControlCommand((PointFeedbackControlCommand) heightControlCommand, pelvisHeightController);
            break;
         case MOMENTUM:
            desiredCoMHeightAcceleration = handleHeightControlCommand((CenterOfMassFeedbackControlCommand) heightControlCommand, comHeightController);
            break;
         default:
            throw new IllegalArgumentException("This command type has not been set up for height control.");
      }
   }

   private static <T extends FeedbackControlCommand<T>> double handleHeightControlCommand(T command, HeightController<T> controller)
   {
      controller.compute(command);
      return controller.getHeightAcceleration();
   }

   private void updateICPControllerState()
   {
      if (initializeOnStateChange)
      {
         icpController.initialize();
      }

      icpController.setMultiContactStabilityRegion(multiContactStabilityRegion);
   }

   private void computeICPController()
   {
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();
      if (perfectCoP.containsNaN())
      {
         perfectCMPDelta.setToZero();
         icpController.compute(supportPolygonInWorld,
                               desiredCapturePoint,
                               desiredCapturePointVelocity,
                               desiredCapturePointAtEndOfState,
                               perfectCMP,
                               capturePoint,
                               centerOfMass2d,
                               omega0);
      }
      else
      {
         perfectCMPDelta.sub(perfectCMP, perfectCoP);
         icpController.compute(supportPolygonInWorld,
                               desiredCapturePoint,
                               desiredCapturePointVelocity,
                               desiredCapturePointAtEndOfState,
                               perfectCoP,
                               perfectCMPDelta,
                               capturePoint,
                               centerOfMass2d,
                               omega0);
      }

      desiredCMP.set(icpController.getDesiredCMP());
      desiredCoP.set(icpController.getDesiredCoP());
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
   }

   private boolean computeDesiredLinearMomentumRateOfChange()
   {
      boolean success = true;

      double totalMass = totalMassProvider.getValue();

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

   private static boolean checkInputs(FramePoint2DReadOnly capturePoint,
                                      FixedFramePoint2DBasics desiredCapturePoint,
                                      FixedFrameVector2DBasics desiredCapturePointVelocity,
                                      FixedFramePoint2DBasics perfectCoP,
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(newYoGraphicPoint2D("Desired CMP", yoDesiredCMP, 0.024, ColorDefinitions.Purple(), DefaultPoint2DGraphic.CIRCLE_PLUS));
      group.addChild(newYoGraphicPoint2D("Achieved CMP", yoAchievedCMP, 0.010, ColorDefinitions.DarkRed(), DefaultPoint2DGraphic.CIRCLE_PLUS));
      group.addChild(newYoGraphicPoint2D("Center Of Mass", yoCenterOfMass, 0.012, ColorDefinitions.Black(), DefaultPoint2DGraphic.CIRCLE_PLUS));
      group.addChild(newYoGraphicPoint2D("Capture Point", yoCapturePoint, 0.02, ColorDefinitions.Blue(), DefaultPoint2DGraphic.CIRCLE_CROSS));
      group.addChild(icpControlPolygons.getSCS2YoGraphics());
      group.addChild(icpController.getSCS2YoGraphics());
      return group;
   }
}
