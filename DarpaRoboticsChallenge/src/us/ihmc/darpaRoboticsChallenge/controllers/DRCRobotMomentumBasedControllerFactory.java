package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.RobotControllerUpdatablesAdapter;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelBehaviorFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.DoNothingBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPBasedLinearMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.InstantaneousCapturePointPlanner;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.InstantaneousCapturePointPlannerWithTimeFreezer;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.SmoothICPComputer2D;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OldMomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.WrenchBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TransferTimeCalculationProvider;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.ForceSensorData;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCRobotMomentumBasedControllerFactory
{
   private static final boolean CREATE_YOVARIABLE_WALKING_PROVIDERS = false;
   
   private final boolean USE_HEADING_AND_VELOCITY_SCRIPT;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FootstepTimingParameters footstepTimingParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ArmControllerParameters armControllerParameters;

   private final double swingTime;
   private final double transferTime;

   private final ConstantSwingTimeCalculator swingTimeCalculator;
   private final ConstantTransferTimeCalculator transferTimeCalculator;
   private final HighLevelState initialBehavior;

   private GlobalDataProducer objectCommunicator;
   private GroundProfile groundProfileForCheatingOnStepHeight;

   private MomentumBasedController momentumBasedController = null;
   private ICPAndMomentumBasedController icpAndMomentumBasedController = null;
   
   private HighLevelHumanoidControllerManager highLevelHumanoidControllerManager = null;
   private final ArrayList<HighLevelBehavior> highLevelBehaviors = new ArrayList<>();
   
   private VariousWalkingProviderFactory variousWalkingProviderFactory;
   private VariousWalkingProviders variousWalkingProviders;
   private VariousWalkingManagers variousWalkingManagers;

   private ArrayList<HighLevelBehaviorFactory> highLevelBehaviorFactories = new ArrayList<>();

   private final double contactTresholdForce;
   private final SideDependentList<String> footSensorNames;
   private final ContactableBodiesFactory contactableBodiesFactory;

   public DRCRobotMomentumBasedControllerFactory(ContactableBodiesFactory contactableBodiesFactory, double contactTresholdForce,
         SideDependentList<String> footSensorNames, FootstepTimingParameters footstepTimingParameters, WalkingControllerParameters walkingControllerParameters,
         ArmControllerParameters armControllerParameters, boolean USE_HEADING_AND_VELOCITY_SCRIPT, boolean useFastTouchdowns, HighLevelState initialBehavior)
   {
      this.contactTresholdForce = contactTresholdForce;
      this.footSensorNames = footSensorNames;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.initialBehavior = initialBehavior;

      this.footstepTimingParameters = footstepTimingParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.armControllerParameters = armControllerParameters;
      
      this.transferTime = walkingControllerParameters.getDefaultTransferTime();
      this.swingTime = walkingControllerParameters.getDefaultSwingTime();

      this.USE_HEADING_AND_VELOCITY_SCRIPT = USE_HEADING_AND_VELOCITY_SCRIPT;

      this.swingTimeCalculator = new ConstantSwingTimeCalculator(swingTime, registry);    // new PiecewiseLinearStepTimeCalculator(stepTime, 0.7, 0.6);
      this.transferTimeCalculator = new ConstantTransferTimeCalculator(transferTime, registry);
   }

   public void setupForNetworkedFootstepProvider(GlobalDataProducer objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }


   public void setupForCheatingUsingGroundHeightAtForFootstepProvider(GroundProfile groundProfileForCheatingOnStepHeight)
   {
      this.groundProfileForCheatingOnStepHeight = groundProfileForCheatingOnStepHeight;
   }

   public void setVariousWalkingProviderFactory(VariousWalkingProviderFactory variousWalkingProviderFactory)
   {
      this.variousWalkingProviderFactory = variousWalkingProviderFactory;
   }

   public RobotController getController(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, double controlDT, double gravity,
         DoubleYoVariable yoTime, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, TwistCalculator twistCalculator,
         CenterOfMassJacobian centerOfMassJacobian, ForceSensorDataHolder forceSensorDataHolder, LidarControllerInterface lidarControllerInterface,
         GlobalDataProducer dataProducer)
   {
      YoVariableRegistry specificRegistry = new YoVariableRegistry("specific");
      SideDependentList<ContactablePlaneBody> feet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, referenceFrames);

      double gravityZ = Math.abs(gravity);
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double totalRobotWeight = totalMass * gravityZ;

      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(feet, forceSensorDataHolder, totalRobotWeight,
            dynamicGraphicObjectsListRegistry, specificRegistry);


      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the different ContactablePlaneBodies ///////////////////////////////////////////////
      
      RigidBody rootBody = fullRobotModel.getRootJoint().getSuccessor();
      SideDependentList<ContactablePlaneBody> thighs = contactableBodiesFactory.createThighContactableBodies(rootBody);
      ContactablePlaneBody pelvisContactablePlaneBody = contactableBodiesFactory.createPelvisContactableBody(fullRobotModel.getPelvis());
      ContactablePlaneBody pelvisBackContactablePlaneBody = contactableBodiesFactory.createPelvisBackContactableBody(fullRobotModel.getPelvis());
      SideDependentList<ContactablePlaneBody> handContactableBodies = contactableBodiesFactory.createHandContactableBodies(rootBody);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the ICPBasedLinearMomentumRateOfChangeControlModule ////////////////////////////////
      ICPBasedLinearMomentumRateOfChangeControlModule iCPBasedLinearMomentumRateOfChangeControlModule =
         new ICPBasedLinearMomentumRateOfChangeControlModule(referenceFrames.getCenterOfMassFrame(), controlDT, totalMass, gravityZ, registry,
            dynamicGraphicObjectsListRegistry);

      iCPBasedLinearMomentumRateOfChangeControlModule.setGains(walkingControllerParameters.getCaptureKpParallelToMotion(),
              walkingControllerParameters.getCaptureKpOrthogonalToMotion(), walkingControllerParameters.getCaptureKi(),
              walkingControllerParameters.getCaptureKiBleedoff(),
              walkingControllerParameters.getCaptureFilterBreakFrequencyInHz(), walkingControllerParameters.getCMPRateLimit(),
              walkingControllerParameters.getCMPAccelerationLimit());

      MomentumOptimizationSettings momentumOptimizationSettings = HighLevelHumanoidControllerFactoryHelper.createMomentumOptimizationSettings(fullRobotModel,
                                                                     lidarControllerInterface, registry);

      // No longer need old one. Don't create it.
      // TODO: Remove OldMomentumControlModule completely once QP stuff is solidified.
      OldMomentumControlModule oldMomentumControlModule = null;

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup different things relative to GUI communication /////////////////////////////////////
      ArrayList<Updatable> updatables = new ArrayList<Updatable>();

      if (CREATE_YOVARIABLE_WALKING_PROVIDERS)
      {
         variousWalkingProviders = VariousWalkingProviders.createUsingYoVariables(fullRobotModel, walkingControllerParameters, referenceFrames, feet,
                 transferTimeCalculator, swingTimeCalculator, registry);
      }
      
      else if (variousWalkingProviderFactory != null)
      {
         variousWalkingProviders = variousWalkingProviderFactory.createVariousWalkingProviders(yoTime, fullRobotModel, walkingControllerParameters,
                 referenceFrames, feet, transferTimeCalculator, swingTimeCalculator, registry);
         if (variousWalkingProviders == null)
            throw new RuntimeException("Couldn't create various walking providers!");
      }
      
      else if (objectCommunicator != null)
      {
         variousWalkingProviders = VariousWalkingProviders.createUsingObjectCommunicator(footstepTimingParameters, objectCommunicator, fullRobotModel,
                 walkingControllerParameters, referenceFrames, feet, transferTimeCalculator, swingTimeCalculator, registry);
      }
      else
      {
         variousWalkingProviders = VariousWalkingProviders.createUsingComponentBasedDesiredFootstepCalculator(fullRobotModel, referenceFrames, feet, controlDT,
                 updatables, USE_HEADING_AND_VELOCITY_SCRIPT, groundProfileForCheatingOnStepHeight, walkingControllerParameters, registry);
      }

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup different things for walking ///////////////////////////////////////////////////////
      double doubleSupportPercentageIn = 0.3;    // NOTE: used to be 0.35, jojo
      double minimumHeightAboveGround = walkingControllerParameters.minimumHeightAboveAnkle();
      double nominalHeightAboveGround = walkingControllerParameters.nominalHeightAboveAnkle();
      double maximumHeightAboveGround = walkingControllerParameters.maximumHeightAboveAnkle();

      DesiredComHeightProvider desiredComHeightProvider = variousWalkingProviders.getDesiredComHeightProvider();

      LookAheadCoMHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator = new LookAheadCoMHeightTrajectoryGenerator(desiredComHeightProvider,
                                                                                       minimumHeightAboveGround, nominalHeightAboveGround,
                                                                                       maximumHeightAboveGround, doubleSupportPercentageIn, yoTime,
                                                                                       dynamicGraphicObjectsListRegistry, registry);
      
      double icpInFromCenter = 0.006; //0.01;
      double doubleSupportFirstStepFraction = 0.5;
      int maxNumberOfConsideredFootsteps = 4;

      SmoothICPComputer2D smoothICPComputer2D = new SmoothICPComputer2D(referenceFrames, controlDT, doubleSupportFirstStepFraction,
                                                   maxNumberOfConsideredFootsteps, registry, dynamicGraphicObjectsListRegistry);
      smoothICPComputer2D.setICPInFromCenter(icpInFromCenter);


      InstantaneousCapturePointPlanner instantaneousCapturePointPlanner = new InstantaneousCapturePointPlannerWithTimeFreezer(smoothICPComputer2D, registry);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the MomentumBasedController ////////////////////////////////////////////////////////
      momentumBasedController = new MomentumBasedController(fullRobotModel, centerOfMassJacobian, referenceFrames, footSwitches,
                                   yoTime, gravityZ, twistCalculator, feet, handContactableBodies, thighs, pelvisContactablePlaneBody,
                                   pelvisBackContactablePlaneBody, controlDT, momentumOptimizationSettings, oldMomentumControlModule,
                                   updatables, dynamicGraphicObjectsListRegistry);

      variousWalkingManagers = VariousWalkingManagers.create(momentumBasedController, yoTime, variousWalkingProviders, walkingControllerParameters,
            armControllerParameters, registry);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the ICPAndMomentumBasedController //////////////////////////////////////////////////
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(referenceFrames.getAnkleZUpReferenceFrames(), referenceFrames.getMidFeetZUpFrame(),
                                                     registry, dynamicGraphicObjectsListRegistry, false);


      icpAndMomentumBasedController = new ICPAndMomentumBasedController(momentumBasedController, fullRobotModel, feet, bipedSupportPolygons, registry);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WalkingHighLevelHumanoidController /////////////////////////////////////////////
      double desiredPelvisPitch = 0.0;
      TransferTimeCalculationProvider transferTimeCalculationProvider = new TransferTimeCalculationProvider("providedTransferTime", registry,
            transferTimeCalculator, transferTime);
      WalkingHighLevelHumanoidController walkingBehavior = new WalkingHighLevelHumanoidController(variousWalkingProviders, variousWalkingManagers, footSwitches,
                                                              centerOfMassHeightTrajectoryGenerator, transferTimeCalculationProvider, desiredPelvisPitch,
                                                              walkingControllerParameters, iCPBasedLinearMomentumRateOfChangeControlModule, null,
                                                              instantaneousCapturePointPlanner, icpAndMomentumBasedController, momentumBasedController, null);
      highLevelBehaviors.add(walkingBehavior);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the DoNothingController ////////////////////////////////////////////////////////////
      // Useful as a transition state on the real robot
      DoNothingBehavior doNothingBehavior = new DoNothingBehavior(momentumBasedController, bipedSupportPolygons);
      highLevelBehaviors.add(doNothingBehavior);
      
      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the HighLevelHumanoidControllerManager /////////////////////////////////////////////
      // This is the "highest level" controller that enables switching between the different controllers (walking, multi-contact, driving, etc.)
      highLevelHumanoidControllerManager = new HighLevelHumanoidControllerManager(initialBehavior, highLevelBehaviors, momentumBasedController, variousWalkingProviders);
      highLevelHumanoidControllerManager.addYoVariableRegistry(this.registry);

      createRegisteredControllers();

      RobotController ret = highLevelHumanoidControllerManager;
      
      ret.getYoVariableRegistry().addChild(specificRegistry);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         RobotControllerUpdatablesAdapter highLevelHumanoidControllerUpdatables = new RobotControllerUpdatablesAdapter(ret);

         ret = highLevelHumanoidControllerUpdatables;
      }
      return ret;
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<ContactablePlaneBody> bipedFeet,
         ForceSensorDataHolder forceSensorDataHolder, double totalRobotWeight, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         YoVariableRegistry registry)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorData footForceSensor = forceSensorDataHolder.getByName(footSensorNames.get(robotSide));
         WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(bipedFeet.get(robotSide).getName(), footForceSensor, 0.02, totalRobotWeight,
               bipedFeet.get(robotSide), dynamicGraphicObjectsListRegistry, contactTresholdForce, registry);
         footSwitches.put(robotSide, wrenchBasedFootSwitch);
      }

      return footSwitches;
   }

   private void createRegisteredControllers()
   {
      for (int i = 0; i < highLevelBehaviorFactories.size(); i++)
      {
         HighLevelBehaviorFactory highLevelBehaviorFactory = highLevelBehaviorFactories.get(i);
         HighLevelBehavior highLevelBehavior = highLevelBehaviorFactory.createHighLevelBehavior(variousWalkingProviders, variousWalkingManagers, momentumBasedController, icpAndMomentumBasedController);
         boolean transitionRequested = highLevelBehaviorFactory.isTransitionToBehaviorRequested();
         highLevelHumanoidControllerManager.addHighLevelBehavior(highLevelBehavior, transitionRequested);
      }
   }

   public void reinitializeWalking()
   {
      highLevelHumanoidControllerManager.requestHighLevelState(HighLevelState.WALKING);
   }

   public void addHighLevelBehaviorFactory(HighLevelBehaviorFactory highLevelBehaviorFactory)
   {
      if (momentumBasedController == null)
      {
         highLevelBehaviorFactories.add(highLevelBehaviorFactory);
      }
      else
      {
         HighLevelBehavior highLevelBehavior = highLevelBehaviorFactory.createHighLevelBehavior(variousWalkingProviders, variousWalkingManagers, momentumBasedController, icpAndMomentumBasedController);
         boolean transitionToBehaviorRequested = highLevelBehaviorFactory.isTransitionToBehaviorRequested();
         highLevelHumanoidControllerManager.addHighLevelBehavior(highLevelBehavior, transitionToBehaviorRequested);
      }
   }
}
