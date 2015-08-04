package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.DoNothingBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CapturePointPlannerAdapter;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPBasedLinearMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OldMomentumControlModule;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchAndContactSensorFusedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.TransferTimeCalculationProvider;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureListener;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerStateChangedListener;
import us.ihmc.robotics.humanoidRobot.model.RobotMotionStatusChangedListener;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.humanoidRobot.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.humanoidRobot.model.ContactSensorHolder;
import us.ihmc.robotics.humanoidRobot.model.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.humanoidRobot.model.ForceSensorDataReadOnly;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class MomentumBasedControllerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final WalkingControllerParameters walkingControllerParameters;
   private final ArmControllerParameters armControllerParameters;
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   private final double swingTime;
   private final double transferTime;

   private final ConstantSwingTimeCalculator swingTimeCalculator;
   private final ConstantTransferTimeCalculator transferTimeCalculator;
   private final HighLevelState initialBehavior;

   private MomentumBasedController momentumBasedController = null;
   private ICPAndMomentumBasedController icpAndMomentumBasedController = null;

   private boolean isListeningToHighLevelStatePackets = true;
   private HighLevelHumanoidControllerManager highLevelHumanoidControllerManager = null;
   private final ArrayList<HighLevelBehavior> highLevelBehaviors = new ArrayList<>();

   private VariousWalkingProviderFactory variousWalkingProviderFactory;
   private VariousWalkingProviders variousWalkingProviders;
   private VariousWalkingManagers variousWalkingManagers;

   private ArrayList<HighLevelBehaviorFactory> highLevelBehaviorFactories = new ArrayList<>();

   private final SideDependentList<String> footSensorNames;
   private final SideDependentList<String> footContactSensorNames;
   private final SideDependentList<String> wristSensorNames;
   private final ContactableBodiesFactory contactableBodiesFactory;
   
   private WalkingHighLevelHumanoidController walkingBehavior;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListenersToAttach = new ArrayList<>();
   private final ArrayList<ControllerFailureListener> controllerFailureListenersToAttach = new ArrayList<>();

   public MomentumBasedControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
         SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames, WalkingControllerParameters walkingControllerParameters, ArmControllerParameters armControllerParameters,
         CapturePointPlannerParameters capturePointPlannerParameters, HighLevelState initialBehavior)
   {
      this.footSensorNames = footForceSensorNames;
      this.footContactSensorNames = footContactSensorNames;
      this.wristSensorNames = wristSensorNames;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.initialBehavior = initialBehavior;

      this.walkingControllerParameters = walkingControllerParameters;
      this.armControllerParameters = armControllerParameters;
      this.capturePointPlannerParameters = capturePointPlannerParameters;

      this.transferTime = walkingControllerParameters.getDefaultTransferTime();
      this.swingTime = walkingControllerParameters.getDefaultSwingTime();

      this.swingTimeCalculator = new ConstantSwingTimeCalculator(swingTime, registry); // new PiecewiseLinearStepTimeCalculator(stepTime, 0.7, 0.6);
      this.transferTimeCalculator = new ConstantTransferTimeCalculator(transferTime, registry);
   }

   public void setVariousWalkingProviderFactory(VariousWalkingProviderFactory variousWalkingProviderFactory)
   {
      this.variousWalkingProviderFactory = variousWalkingProviderFactory;
   }

   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   public HighLevelHumanoidControllerManager getHighLevelHumanoidControllerManager()
   {
      return highLevelHumanoidControllerManager;
   }
   
   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      momentumBasedController.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }

   public RobotController getController(FullRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames, double controlDT, double gravity,
         DoubleYoVariable yoTime, YoGraphicsListRegistry yoGraphicsListRegistry, TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian,
         ForceSensorDataHolderReadOnly forceSensorDataHolder, ContactSensorHolder contactSensorHolder,
         CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, GlobalDataProducer dataProducer, InverseDynamicsJoint... jointsToIgnore)
   {
      SideDependentList<ContactablePlaneBody> feet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, referenceFrames);

      double gravityZ = Math.abs(gravity);
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double totalRobotWeight = totalMass * gravityZ;

      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(feet, forceSensorDataHolder, contactSensorHolder, totalRobotWeight, yoGraphicsListRegistry, registry);
      SideDependentList<ForceSensorDataReadOnly> wristForceSensors = createWristForceSensors(forceSensorDataHolder);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the different ContactablePlaneBodies ///////////////////////////////////////////////

      RigidBody rootBody = fullRobotModel.getRootJoint().getSuccessor();
      SideDependentList<ContactablePlaneBody> thighs = contactableBodiesFactory.createThighContactableBodies(rootBody);
      ContactablePlaneBody pelvisContactablePlaneBody = contactableBodiesFactory.createPelvisContactableBody(fullRobotModel.getPelvis());
      ContactablePlaneBody pelvisBackContactablePlaneBody = contactableBodiesFactory.createPelvisBackContactableBody(fullRobotModel.getPelvis());
      SideDependentList<ContactablePlaneBody> handContactableBodies = contactableBodiesFactory.createHandContactableBodies(rootBody);

      // No longer need old one. Don't create it.
      // TODO: Remove OldMomentumControlModule completely once QP stuff is solidified.
      OldMomentumControlModule oldMomentumControlModule = null;

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup different things relative to GUI communication and WalkingProviders ////////////////

      if (variousWalkingProviderFactory == null)
      {
         variousWalkingProviderFactory = new DoNothingVariousWalkingProviderFactory(controlDT);
      }

      variousWalkingProviders = variousWalkingProviderFactory.createVariousWalkingProviders(yoTime, fullRobotModel, walkingControllerParameters,
            referenceFrames, feet, transferTimeCalculator, swingTimeCalculator, updatables, registry, yoGraphicsListRegistry);
      if (variousWalkingProviders == null)
         throw new RuntimeException("Couldn't create various walking providers!");

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(referenceFrames.getAnkleZUpReferenceFrames(), referenceFrames.getMidFeetZUpFrame(),
            registry, yoGraphicsListRegistry, false);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup different things for walking ///////////////////////////////////////////////////////
      double doubleSupportPercentageIn = 0.3; // NOTE: used to be 0.35, jojo
      double minimumHeightAboveGround = walkingControllerParameters.minimumHeightAboveAnkle();
      double nominalHeightAboveGround = walkingControllerParameters.nominalHeightAboveAnkle();
      double maximumHeightAboveGround = walkingControllerParameters.maximumHeightAboveAnkle();
      double defaultOffsetHeightAboveGround = walkingControllerParameters.defaultOffsetHeightAboveAnkle();

      DesiredComHeightProvider desiredComHeightProvider = variousWalkingProviders.getDesiredComHeightProvider();

      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      SideDependentList<ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      LookAheadCoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator = new LookAheadCoMHeightTrajectoryGenerator(desiredComHeightProvider,
            minimumHeightAboveGround, nominalHeightAboveGround, maximumHeightAboveGround, defaultOffsetHeightAboveGround, doubleSupportPercentageIn,
            pelvisFrame, ankleZUpFrames, yoTime, yoGraphicsListRegistry, registry);
      centerOfMassHeightTrajectoryGenerator.setCoMHeightDriftCompensation(walkingControllerParameters.getCoMHeightDriftCompensation());

      CapturePointPlannerAdapter instantaneousCapturePointPlanner = new CapturePointPlannerAdapter(capturePointPlannerParameters, walkingControllerParameters, registry,
            yoGraphicsListRegistry, controlDT, feet, bipedSupportPolygons);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the MomentumBasedController ////////////////////////////////////////////////////////
      momentumBasedController = new MomentumBasedController(fullRobotModel, centerOfMassJacobian, referenceFrames, footSwitches, wristForceSensors, yoTime, gravityZ,
            twistCalculator, feet, handContactableBodies, thighs, pelvisContactablePlaneBody, pelvisBackContactablePlaneBody, controlDT,
            oldMomentumControlModule, updatables, armControllerParameters, walkingControllerParameters, yoGraphicsListRegistry, jointsToIgnore);
      momentumBasedController.attachControllerStateChangedListeners(controllerStateChangedListenersToAttach);
      attachControllerFailureListeners(controllerFailureListenersToAttach);

      TransferTimeCalculationProvider transferTimeCalculationProvider = new TransferTimeCalculationProvider("providedTransferTime", registry,
            transferTimeCalculator, transferTime);
      SwingTimeCalculationProvider swingTimeCalculationProvider = new SwingTimeCalculationProvider("providedSwingTime", registry, swingTimeCalculator,
            swingTime);

      variousWalkingManagers = VariousWalkingManagers.create(momentumBasedController, variousWalkingProviders, walkingControllerParameters,
            armControllerParameters, registry, swingTimeCalculationProvider);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the ICPBasedLinearMomentumRateOfChangeControlModule ////////////////////////////////
      ICPControlGains icpControlGains = walkingControllerParameters.getICPControlGains();
      ICPBasedLinearMomentumRateOfChangeControlModule iCPBasedLinearMomentumRateOfChangeControlModule = new ICPBasedLinearMomentumRateOfChangeControlModule(
            referenceFrames, bipedSupportPolygons, controlDT, totalMass, gravityZ, icpControlGains, registry, yoGraphicsListRegistry);

      CapturabilityBasedStatusProducer capturabilityBasedStatusProducer = variousWalkingProviders.getCapturabilityBasedStatusProducer();

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the ICPAndMomentumBasedController //////////////////////////////////////////////////
      double omega0 = walkingControllerParameters.getOmega0();
      icpAndMomentumBasedController = new ICPAndMomentumBasedController(momentumBasedController, omega0, iCPBasedLinearMomentumRateOfChangeControlModule,
            bipedSupportPolygons, capturabilityBasedStatusProducer, registry);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WalkingHighLevelHumanoidController /////////////////////////////////////////////

      walkingBehavior = new WalkingHighLevelHumanoidController(variousWalkingProviders, variousWalkingManagers,
            centerOfMassHeightTrajectoryGenerator, transferTimeCalculationProvider, swingTimeCalculationProvider, walkingControllerParameters,
            instantaneousCapturePointPlanner, icpAndMomentumBasedController, momentumBasedController);
      highLevelBehaviors.add(walkingBehavior);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the DoNothingController ////////////////////////////////////////////////////////////
      // Useful as a transition state on the real robot
      DoNothingBehavior doNothingBehavior = new DoNothingBehavior(momentumBasedController, bipedSupportPolygons);
      highLevelBehaviors.add(doNothingBehavior);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the HighLevelHumanoidControllerManager /////////////////////////////////////////////
      // This is the "highest level" controller that enables switching between
      // the different controllers (walking, multi-contact, driving, etc.)
      highLevelHumanoidControllerManager = new HighLevelHumanoidControllerManager(initialBehavior, highLevelBehaviors, momentumBasedController,
            variousWalkingProviders, centerOfPressureDataHolderForEstimator, dataProducer);
      highLevelHumanoidControllerManager.setFallbackControllerForFailure(HighLevelState.DO_NOTHING_BEHAVIOR);
      highLevelHumanoidControllerManager.addYoVariableRegistry(registry);
      highLevelHumanoidControllerManager.setListenToHighLevelStatePackets(isListeningToHighLevelStatePackets);

      createRegisteredControllers();

      return highLevelHumanoidControllerManager;
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<ContactablePlaneBody> bipedFeet,
         ForceSensorDataHolderReadOnly forceSensorDataHolder, ContactSensorHolder contactSensorHolder, double totalRobotWeight,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSwitchInterface footSwitch = null;
         String footName = bipedFeet.get(robotSide).getName();
         ForceSensorDataReadOnly footForceSensor = forceSensorDataHolder.getByName(footSensorNames.get(robotSide));
         double contactThresholdForce = walkingControllerParameters.getContactThresholdForce();
         double footSwitchCoPThresholdFraction = walkingControllerParameters.getCoPThresholdFraction();
         
         switch (walkingControllerParameters.getFootSwitchType())
         {
         case KinematicBased:
            footSwitch = new KinematicsBasedFootSwitch(footName, bipedFeet, walkingControllerParameters.getContactThresholdHeight(), totalRobotWeight,
                  robotSide, registry); //controller switch doesnt need com
            break;

         case WrenchBased:
            WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(footName, footForceSensor, footSwitchCoPThresholdFraction, totalRobotWeight, bipedFeet.get(robotSide),
                     yoGraphicsListRegistry, contactThresholdForce, registry);
            wrenchBasedFootSwitch.setSecondContactThresholdForce(walkingControllerParameters.getSecondContactThresholdForceIgnoringCoP());
            footSwitch = wrenchBasedFootSwitch;
            break;
            
         case WrenchAndContactSensorFused:
            footSwitch = new WrenchAndContactSensorFusedFootSwitch(footName, footForceSensor, contactSensorHolder.getByName(footContactSensorNames.get(robotSide)), 
                  footSwitchCoPThresholdFraction, totalRobotWeight, bipedFeet.get(robotSide), yoGraphicsListRegistry, contactThresholdForce, registry);
            break;
         }

         assert footSwitch != null;
         footSwitches.put(robotSide, footSwitch);
      }

      return footSwitches;
   }

   private SideDependentList<ForceSensorDataReadOnly> createWristForceSensors(ForceSensorDataHolderReadOnly forceSensorDataHolder)
   {
      if (wristSensorNames == null) return null;

      SideDependentList<ForceSensorDataReadOnly> wristForceSensors = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         if(wristSensorNames.get(robotSide) == null)
         {
            return null;
         }
         ForceSensorDataReadOnly wristForceSensor = forceSensorDataHolder.getByName(wristSensorNames.get(robotSide));
         wristForceSensors.put(robotSide, wristForceSensor);
      }
      return wristForceSensors;
   }

   private void createRegisteredControllers()
   {
      for (int i = 0; i < highLevelBehaviorFactories.size(); i++)
      {
         HighLevelBehaviorFactory highLevelBehaviorFactory = highLevelBehaviorFactories.get(i);
         HighLevelBehavior highLevelBehavior = highLevelBehaviorFactory.createHighLevelBehavior(variousWalkingProviders, variousWalkingManagers,
               momentumBasedController, icpAndMomentumBasedController);
         boolean transitionRequested = highLevelBehaviorFactory.isTransitionToBehaviorRequested();
         highLevelHumanoidControllerManager.addHighLevelBehavior(highLevelBehavior, transitionRequested);
      }
   }

   public void reinitializeWalking(boolean keepPosition)
   {
      highLevelHumanoidControllerManager.requestHighLevelState(HighLevelState.WALKING);
      if( keepPosition )
      {
         if ( walkingBehavior!= null )
         {
            walkingBehavior.initializeDesiredHeightToCurrent();
            walkingBehavior.requestICPPlannerToHoldCurrentCoM();
            walkingBehavior.reinitializePelvisOrientation(false);
         }
         
         if( variousWalkingManagers != null){
            variousWalkingManagers.getManipulationControlModule().initializeDesiredToCurrent(); 
            variousWalkingManagers.getPelvisOrientationManager().setToHoldCurrentInWorldFrame();
         }          
      }
   }
   
   public void reinitializePositionControl()
   {
      highLevelHumanoidControllerManager.requestHighLevelState(HighLevelState.JOINT_POSITION_CONTROL);
   }

   public VariousWalkingProviders getVariousWalkingProviders()
   {
      return variousWalkingProviders;
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      if (highLevelHumanoidControllerManager != null)
         highLevelHumanoidControllerManager.setListenToHighLevelStatePackets(isListening);
      else
         isListeningToHighLevelStatePackets = isListening;
   }

   public void addHighLevelBehaviorFactory(HighLevelBehaviorFactory highLevelBehaviorFactory)
   {
      if (momentumBasedController == null)
      {
         highLevelBehaviorFactories.add(highLevelBehaviorFactory);
      }
      else
      {
         HighLevelBehavior highLevelBehavior = highLevelBehaviorFactory.createHighLevelBehavior(variousWalkingProviders, variousWalkingManagers,
               momentumBasedController, icpAndMomentumBasedController);
         boolean transitionToBehaviorRequested = highLevelBehaviorFactory.isTransitionToBehaviorRequested();
         highLevelHumanoidControllerManager.addHighLevelBehavior(highLevelBehavior, transitionToBehaviorRequested);
      }
   }

   public void attachControllerFailureListeners(List<ControllerFailureListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
         attachControllerFailureListener(listeners.get(i));
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      if (momentumBasedController != null)
         momentumBasedController.attachControllerFailureListener(listener);
      else
         controllerFailureListenersToAttach.add(listener);
   }

   public void attachControllerStateChangedListeners(List<ControllerStateChangedListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
         attachControllerStateChangedListener(listeners.get(i));
   }

   public void attachControllerStateChangedListener(ControllerStateChangedListener listener)
   {
      if (momentumBasedController != null)
         momentumBasedController.attachControllerStateChangedListener(listener);
      else
         controllerStateChangedListenersToAttach.add(listener);
   }

   public void attachRobotMotionStatusChangedListener(RobotMotionStatusChangedListener listener)
   {
      momentumBasedController.attachRobotMotionStatusChangedListener(listener);
   }

   public void setFallbackControllerForFailure(HighLevelState fallbackController)
   {
      highLevelHumanoidControllerManager.setFallbackControllerForFailure(fallbackController);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
