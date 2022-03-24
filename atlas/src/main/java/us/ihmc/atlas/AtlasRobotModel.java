package us.ihmc.atlas;

import java.io.InputStream;
import java.util.Arrays;
import java.util.function.Consumer;

import us.ihmc.atlas.diagnostic.AtlasDiagnosticParameters;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.AtlasCoPTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasFootstepPlannerParameters;
import us.ihmc.atlas.parameters.AtlasHighLevelControllerParameters;
import us.ihmc.atlas.parameters.AtlasICPSplitFractionCalculatorParameters;
import us.ihmc.atlas.parameters.AtlasKinematicsCollisionModel;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasPushRecoveryControllerParameters;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.parameters.AtlasSimulationCollisionModel;
import us.ihmc.atlas.parameters.AtlasStateEstimatorParameters;
import us.ihmc.atlas.parameters.AtlasSwingPlannerParameters;
import us.ihmc.atlas.parameters.AtlasUIParameters;
import us.ihmc.atlas.parameters.AtlasVisibilityGraphParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.atlas.ros.AtlasPPSTimestampOffsetProvider;
import us.ihmc.atlas.sensors.AtlasCollisionBoxProvider;
import us.ihmc.atlas.sensors.AtlasSensorSuiteManager;
import us.ihmc.avatar.DRCSimulationOutputWriterForControllerThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.avatar.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.avatar.reachabilityMap.footstep.StepReachabilityIOHelper;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.ros.RobotROSClockCalculatorFromPPSOffset;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.modelFileLoaders.SdfLoader.SDFModelLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotiq.model.RobotiqHandModel;
import us.ihmc.robotiq.simulatedHand.SimulatedRobotiqHandsControlThread;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.UIParameters;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;

public class AtlasRobotModel implements DRCRobotModel
{
   public final static boolean SCALE_ATLAS = false;
   private final static double DESIRED_ATLAS_HEIGHT = 0.66;
   private final static double DESIRED_ATLAS_WEIGHT = 15;

   private final double HARDSTOP_RESTRICTION_ANGLE = Math.toRadians(5.0);

   private final AtlasRobotVersion selectedVersion;
   private final RobotTarget target;

   private static final long ESTIMATOR_DT_IN_NS = 1000000;
   private static final double ESTIMATOR_DT = Conversions.nanosecondsToSeconds(ESTIMATOR_DT_IN_NS);
   private static final double CONTROL_DT = 0.004; // 0.006;

   private static final double ATLAS_ONBOARD_SAMPLINGFREQ = 1000.0;
   public static final double ATLAS_ONBOARD_DT = 1.0 / ATLAS_ONBOARD_SAMPLINGFREQ;

   public static final boolean BATTERY_MASS_SIMULATOR_IN_ROBOT = false;

   private final AtlasPhysicalProperties atlasPhysicalProperties;
   private final AtlasJointMap jointMap;
   private final AtlasContactPointParameters contactPointParameters;
   private final AtlasSensorInformation sensorInformation;
   private final AtlasWalkingControllerParameters walkingControllerParameters;
   private final AtlasPushRecoveryControllerParameters pushRecoveryControllerParameters;
   private final AtlasStateEstimatorParameters stateEstimatorParameters;
   private final AtlasHighLevelControllerParameters highLevelControllerParameters;

   private AtlasSensorSuiteManager sensorSuiteManager;

   private Consumer<RobotDefinition> robotDefinitionMutator;
   private RobotDefinition robotDefinition, robotDefinitionWithSDFCollision;
   private String simpleRobotName = "Atlas";
   private StepReachabilityData stepReachabilityData = null;

   public AtlasRobotModel(AtlasRobotVersion atlasVersion)
   {
      this(atlasVersion, RobotTarget.SCS);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, RobotTarget target)
   {
      this(atlasVersion, target, false);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless)
   {
      this(atlasVersion, target, headless, null, false, false);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless, boolean createAdditionalContactPoints)
   {
      this(atlasVersion, target, headless, null, createAdditionalContactPoints, false);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion,
                          RobotTarget target,
                          boolean headless,
                          boolean createAdditionalContactPoints,
                          boolean useShapeCollision)
   {
      this(atlasVersion, target, headless, null, createAdditionalContactPoints, useShapeCollision);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless, FootContactPoints<RobotSide> simulationContactPoints)
   {
      this(atlasVersion, target, headless, simulationContactPoints, false, false);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion,
                          RobotTarget target,
                          boolean headless,
                          FootContactPoints<RobotSide> simulationContactPoints,
                          boolean createAdditionalContactPointsn)
   {
      this(atlasVersion, target, headless, simulationContactPoints, createAdditionalContactPointsn, false);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion,
                          RobotTarget target,
                          boolean headless,
                          FootContactPoints<RobotSide> simulationContactPoints,
                          boolean createAdditionalContactPoints,
                          boolean useShapeCollision)
   {
      if (SCALE_ATLAS)
      {
         atlasPhysicalProperties = new AtlasPhysicalProperties(DESIRED_ATLAS_HEIGHT, DESIRED_ATLAS_WEIGHT);
      }
      else
      {
         atlasPhysicalProperties = new AtlasPhysicalProperties();
      }

      selectedVersion = atlasVersion;
      jointMap = new AtlasJointMap(selectedVersion, atlasPhysicalProperties);

      boolean createFootContactPoints = true;
      contactPointParameters = new AtlasContactPointParameters(jointMap,
                                                               atlasVersion,
                                                               createFootContactPoints,
                                                               simulationContactPoints,
                                                               createAdditionalContactPoints);

      this.target = target;

      sensorInformation = new AtlasSensorInformation(atlasVersion, target);

      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      highLevelControllerParameters = new AtlasHighLevelControllerParameters(runningOnRealRobot, jointMap);
      walkingControllerParameters = new AtlasWalkingControllerParameters(target, jointMap, contactPointParameters);
      pushRecoveryControllerParameters = new AtlasPushRecoveryControllerParameters(target, jointMap, contactPointParameters);
      stateEstimatorParameters = new AtlasStateEstimatorParameters(jointMap, sensorInformation, runningOnRealRobot, getEstimatorDT());
   }

   public RobotDefinition createRobotDefinition()
   {
      return createRobotDefinition(Double.NaN);
   }

   public RobotDefinition createRobotDefinition(double transparency)
   {
      if (Double.isNaN(transparency) || transparency < 0.0)
         return createRobotDefinition((MaterialDefinition) null);
      else
         return createRobotDefinition(ColorDefinitions.Orange().derive(0, 1, 1, 1.0 - transparency));
   }

   public RobotDefinition createRobotDefinition(ColorDefinition diffuseColor)
   {
      return createRobotDefinition(new MaterialDefinition(diffuseColor));
   }

   public RobotDefinition createRobotDefinition(MaterialDefinition materialDefinition)
   {
      return createRobotDefinition(materialDefinition, true);
   }

   public RobotDefinition createRobotDefinition(MaterialDefinition materialDefinition, boolean removeCollisions)
   {
      InputStream stream = selectedVersion.getSdfFileAsStream();
      if (stream == null)
         LogTools.error("Selected version {} could not be found: stream is null", selectedVersion);
      RobotDefinition robotDefinition = RobotDefinitionTools.loadSDFModel(stream,
                                                                          Arrays.asList(selectedVersion.getResourceDirectories()),
                                                                          getClass().getClassLoader(),
                                                                          selectedVersion.getModelName(),
                                                                          getContactPointParameters(),
                                                                          jointMap,
                                                                          removeCollisions);
      if (materialDefinition != null)
         RobotDefinitionTools.setRobotDefinitionMaterial(robotDefinition, materialDefinition);
      else
         RobotDefinitionTools.setDefaultMaterial(robotDefinition, new MaterialDefinition(ColorDefinitions.Black()));

      getRobotDefinitionMutator().accept(robotDefinition);

      return robotDefinition;
   }

   @Override
   public RobotDefinition getRobotDefinition()
   {
      if (robotDefinition == null)
         robotDefinition = createRobotDefinition();
      return robotDefinition;
   }

   public RobotDefinition getRobotDefinitionWithSDFCollision()
   {
      if (robotDefinitionWithSDFCollision == null)
         robotDefinitionWithSDFCollision = createRobotDefinition(null, false);
      return robotDefinitionWithSDFCollision;
   }

   public void disableOneDoFJointDamping()
   {
      setRobotDefinitionMutator(getRobotDefinitionMutator().andThen(def -> def.forEachOneDoFJointDefinition(joint -> joint.setDamping(0.0))));
   }

   public void setRobotDefinitionMutator(Consumer<RobotDefinition> robotDefinitionMutator)
   {
      if (robotDefinition != null)
         throw new IllegalArgumentException("Cannot set customModel once generalizedRobotModel has been created.");
      this.robotDefinitionMutator = robotDefinitionMutator;
   }

   public Consumer<RobotDefinition> getRobotDefinitionMutator()
   {
      if (robotDefinitionMutator == null)
         robotDefinitionMutator = new AtlasRobotDefinitionMutator(getJointMap(), getSensorInformation());
      return robotDefinitionMutator;
   }

   @Override
   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      return highLevelControllerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public PushRecoveryControllerParameters getPushRecoveryControllerParameters()
   {
      return pushRecoveryControllerParameters;
   }

   @Override
   public CoPTrajectoryParameters getCoPTrajectoryParameters()
   {
      return new AtlasCoPTrajectoryParameters();
   }

   @Override
   public SplitFractionCalculatorParametersReadOnly getSplitFractionCalculatorParameters()
   {
      return new AtlasICPSplitFractionCalculatorParameters();
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParameters;
   }

   public AtlasPhysicalProperties getPhysicalProperties()
   {
      return atlasPhysicalProperties;
   }

   @Override
   public RobotTarget getTarget()
   {
      return target;
   }

   @Override
   public AtlasJointMap getJointMap()
   {
      return jointMap;
   }

   public AtlasRobotVersion getAtlasVersion()
   {
      return selectedVersion;
   }

   @Override
   public String toString()
   {
      return selectedVersion.toString();
   }

   @Override
   public RobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup()
   {
      return new AtlasSimInitialSetup(getRobotDefinition(), getJointMap());
   }

   @Override
   public AtlasContactPointParameters getContactPointParameters()
   {
      return contactPointParameters;
   }

   public void setJointDamping(FloatingRootJointRobot simulatedRobot)
   {
      AtlasDampingParameters.setDampingParameters(simulatedRobot, getJointMap());
   }

   @Override
   public HandModel getHandModel()
   {
      if (selectedVersion.hasRobotiqHands())
         return new RobotiqHandModel();

      return null;
   }

   @Override
   public AtlasSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      FullHumanoidRobotModel fullRobotModel = new FullHumanoidRobotModelWrapper(getRobotDefinition(), getJointMap());
      for (RobotSide robotSide : RobotSide.values())
      {
         ArmJointName[] armJointNames = new ArmJointName[] {ArmJointName.FIRST_WRIST_PITCH, ArmJointName.WRIST_ROLL, ArmJointName.SECOND_WRIST_PITCH};

         for (ArmJointName armJointName : armJointNames)
         {
            OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
            if (armJoint == null)
               continue;

            double lowerLimit = armJoint.getJointLimitLower();
            double upperLimit = armJoint.getJointLimitUpper();

            double range = upperLimit - lowerLimit;

            if (range > 2.0 * HARDSTOP_RESTRICTION_ANGLE)
            {
               double safeLowerBound = lowerLimit + HARDSTOP_RESTRICTION_ANGLE;
               double safeUpperBound = upperLimit - HARDSTOP_RESTRICTION_ANGLE;

               armJoint.setJointLimitLower(safeLowerBound);
               armJoint.setJointLimitUpper(safeUpperBound);
            }
            else
            {
               System.out.println(this.getClass().getName() + ", createFullRobotModel(): range not large enough to reduce for side="
                     + robotSide.getLowerCaseName() + " joint=" + armJointName.getCamelCaseNameForStartOfExpression());
            }
         }
      }

      return fullRobotModel;
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;
      HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot = new HumanoidFloatingRootJointRobot(getRobotDefinition(),
                                                                                                         getJointMap(),
                                                                                                         enableJointDamping,
                                                                                                         enableTorqueVelocityLimits);
      return humanoidFloatingRootJointRobot;
   }

   @Override
   public double getSimulateDT()
   {
      return 0.0005;
   }

   @Override
   public double getEstimatorDT()
   {
      return ESTIMATOR_DT;
   }

   @Override
   public double getControllerDT()
   {
      return CONTROL_DT;
   }

   @Override
   public RobotROSClockCalculator getROSClockCalculator()
   {
      DRCROSPPSTimestampOffsetProvider timestampOffsetProvider = null;

      if (target == RobotTarget.REAL_ROBOT)
      {
         timestampOffsetProvider = AtlasPPSTimestampOffsetProvider.getInstance(sensorInformation);
      }

      if (AtlasSensorInformation.SEND_ROBOT_DATA_TO_ROS)
      {
         if (target == RobotTarget.SCS)
         {
            timestampOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
         }
      }

      if (timestampOffsetProvider == null)
         timestampOffsetProvider = new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();

      return new RobotROSClockCalculatorFromPPSOffset(timestampOffsetProvider);
   }

   @Override
   public String getStepReachabilityResourceName()
   {
      return "us/ihmc/atlas/parameters/StepReachabilityMap.json";
   }

   @Override
   public StepReachabilityData getStepReachabilityData()
   {
      if (stepReachabilityData == null)
      {
         stepReachabilityData = new StepReachabilityIOHelper().loadStepReachability(this);
      }

      return stepReachabilityData;
   }

   @Override
   public AtlasSensorSuiteManager getSensorSuiteManager()
   {
      return getSensorSuiteManager(null);
   }

   @Override
   public AtlasSensorSuiteManager getSensorSuiteManager(ROS2NodeInterface ros2Node)
   {
      if (sensorSuiteManager == null)
      {
         sensorSuiteManager = new AtlasSensorSuiteManager(getSimpleRobotName(),
                                                          this,
                                                          getCollisionBoxProvider(),
                                                          getROSClockCalculator(),
                                                          sensorInformation,
                                                          getJointMap(),
                                                          getPhysicalProperties(),
                                                          target,
                                                          ros2Node);
      }
      return sensorSuiteManager;
   }

   @Override
   public UIParameters getUIParameters()
   {
      return new AtlasUIParameters(selectedVersion, atlasPhysicalProperties);
   }

   @Override
   public SimulatedRobotiqHandsControlThread createSimulatedHandController(RealtimeROS2Node realtimeROS2Node)
   {
      switch (selectedVersion.getHandModel())
      {
         case ROBOTIQ:
            return new SimulatedRobotiqHandsControlThread(createFullRobotModel(),
                                                          realtimeROS2Node,
                                                          ROS2Tools.getControllerOutputTopic(getSimpleRobotName()),
                                                          ROS2Tools.getControllerInputTopic(getSimpleRobotName()));

         default:
            return null;
      }
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new DefaultLogModelProvider<>(SDFModelLoader.class,
                                           jointMap.getModelName(),
                                           selectedVersion.getSdfFileAsStream(),
                                           selectedVersion.getResourceDirectories());
   }

   @Override
   public DataServerSettings getLogSettings()
   {

      switch (target)
      {
         case REAL_ROBOT:
            return new DataServerSettings(true, "AtlasGUI");
         case GAZEBO:
         case SCS:
         default:
            return new DataServerSettings(false, "SimulationGUI");
      }
   }

   @Override
   public String getSimpleRobotName()
   {
      return simpleRobotName;
   }

   public void setSimpleRobotName(String simpleRobotName)
   {
      this.simpleRobotName = simpleRobotName;
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return new AtlasCollisionBoxProvider(getRobotDefinitionWithSDFCollision(), getJointMap());
   }

   @Override
   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return new AtlasFootstepPlannerParameters();
   }

   @Override
   public FootstepPlannerParametersBasics getFootstepPlannerParameters(String fileNameSuffix)
   {
      return new AtlasFootstepPlannerParameters(fileNameSuffix);
   }

   @Override
   public VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
   {
      return new AtlasVisibilityGraphParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return new AtlasSwingPlannerParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters(String fileNameSuffix)
   {
      return new AtlasSwingPlannerParameters(fileNameSuffix);
   }

   @Override
   public DRCOutputProcessor getCustomSimulationOutputProcessor(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot)
   {
      return new DRCSimulationOutputWriterForControllerThread(humanoidFloatingRootJointRobot);
   }

   @Override
   public JointDesiredOutputWriter getCustomSimulationOutputWriter(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot,
                                                                   HumanoidRobotContextData contextData)
   {
      return null;
   }

   public static String getParameterResourceName()
   {
      return "/us/ihmc/atlas/parameters/controller.xml";
   }

   @Override
   public String getParameterFileName()
   {
      return getParameterResourceName();
   }

   @Override
   public InputStream getParameterOverwrites()
   {
      if (target == RobotTarget.REAL_ROBOT)
      {
         return getClass().getResourceAsStream("/us/ihmc/atlas/parameters/real_robot.xml");
      }
      return null;
   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      return getClass().getResourceAsStream(getParameterResourceName());
   }

   @Override
   public RobotCollisionModel getHumanoidRobotKinematicsCollisionModel()
   {
      return new AtlasKinematicsCollisionModel(jointMap);
   }

   @Override
   public RobotCollisionModel getSimulationRobotCollisionModel(CollidableHelper helper, String robotCollisionMask, String... environmentCollisionMasks)
   {
      AtlasSimulationCollisionModel collisionModel = new AtlasSimulationCollisionModel(jointMap, selectedVersion);
      collisionModel.setCollidableHelper(helper, robotCollisionMask, environmentCollisionMasks);
      return collisionModel;
   }

   @Override
   public DiagnosticParameters getDiagnoticParameters()
   {
      return new AtlasDiagnosticParameters(getJointMap(), getSensorInformation(), target == RobotTarget.REAL_ROBOT);
   }

   @Override
   public RobotLowLevelMessenger newRobotLowLevelMessenger(ROS2NodeInterface ros2Node)
   {
      return new AtlasDirectRobotInterface(ros2Node, this);
   }
}
