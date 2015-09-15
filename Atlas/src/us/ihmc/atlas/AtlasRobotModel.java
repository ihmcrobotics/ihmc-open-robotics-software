package us.ihmc.atlas;

import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.AtlasArmControllerParameters;
import us.ihmc.atlas.parameters.AtlasCapturePointPlannerParameters;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasDefaultArmConfigurations;
import us.ihmc.atlas.parameters.AtlasDrivingControllerParameters;
import us.ihmc.atlas.parameters.AtlasFootstepPlanningParameterization;
import us.ihmc.atlas.parameters.AtlasHeightCalculatorParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasRobotMultiContactControllerParameters;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.parameters.AtlasStateEstimatorParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.atlas.physics.AtlasPhysicsEngineConfiguration;
import us.ihmc.atlas.ros.AtlasPPSTimestampOffsetProvider;
import us.ihmc.atlas.sensors.AtlasCollisionBoxProvider;
import us.ihmc.atlas.sensors.AtlasSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.HeightCalculatorParameters;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.pathGeneration.footstepPlanner.FootstepPlanningParameterization;
import us.ihmc.pathGeneration.footstepSnapper.AtlasFootstepSnappingParameters;
import us.ihmc.pathGeneration.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.robotiq.control.RobotiqHandCommandManager;
import us.ihmc.robotiq.model.RobotiqHandModel;
import us.ihmc.robotiq.simulatedHand.SimulatedRobotiqHandsController;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.physics.ScsCollisionConfigure;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

import com.jme3.math.Transform;

public class AtlasRobotModel implements DRCRobotModel
{
   private final double HARDSTOP_RESTRICTION_ANGLE = Math.toRadians(5.0);
   
   private final AtlasRobotVersion selectedVersion;
   private final DRCRobotModel.RobotTarget target;

   private static final long ESTIMATOR_DT_IN_NS = 1000000;
   private static final double ESTIMATOR_DT = TimeTools.nanoSecondstoSeconds(ESTIMATOR_DT_IN_NS);
   private static final double CONTROL_DT = 0.004;    // 0.006;

   private static final double ATLAS_ONBOARD_SAMPLINGFREQ = 1000.0;
   public static final double ATLAS_ONBOARD_DT = 1.0 / ATLAS_ONBOARD_SAMPLINGFREQ;
   private static final boolean USE_WHOLE_BODY_IK = true;

   private final JaxbSDFLoader loader;

   private final AtlasJointMap jointMap;
   private final AtlasSensorInformation sensorInformation;
   private final AtlasArmControllerParameters armControllerParameters;
   private final AtlasCapturePointPlannerParameters capturePointPlannerParameters;
   private final AtlasWalkingControllerParameters walkingControllerParameters;
   private final AtlasStateEstimatorParameters stateEstimatorParameters;
   private final AtlasRobotMultiContactControllerParameters multiContactControllerParameters;
   private final AtlasDrivingControllerParameters drivingControllerParameters;
   private final AtlasDefaultArmConfigurations defaultArmConfigurations;
   private final AtlasHeightCalculatorParameters heightCalculatorParameters;
   private final AtlasFootstepSnappingParameters snappingParameters;

   private boolean enableJointDamping = true;

   private final AtlasRobotModelMutator modelMutator = new AtlasRobotModelMutator();

   @Override
   public WholeBodyIkSolver createWholeBodyIkSolver()
   {
      if(USE_WHOLE_BODY_IK)
      {
         try
         {
            return new AtlasWholeBodyIK(this);
         }
         catch(UnsatisfiedLinkError e)
         {
            System.err.println("There was an error linking to the Whole Body IK C library, "
                  + "please check that it has been compiles and is located in the correct place, "
                  + "no wholebody components will function");
            System.err.println(e.getMessage());
         }
      }
      return null;
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, DRCRobotModel.RobotTarget target, boolean headless)
   {
      selectedVersion = atlasVersion;
      jointMap = new AtlasJointMap(selectedVersion);
      this.target = target;

      if (!headless)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(selectedVersion.getResourceDirectories(), selectedVersion.getSdfFileAsStream(), false, modelMutator);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[]
         {
         }, selectedVersion.getSdfFileAsStream(), headless, modelMutator);
      }

      for (String forceSensorNames : AtlasSensorInformation.forceSensorNames)
      {
         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, new RigidBodyTransform());
      }

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;
      capturePointPlannerParameters = new AtlasCapturePointPlannerParameters(runningOnRealRobot);
      sensorInformation = new AtlasSensorInformation(target);
      armControllerParameters = new AtlasArmControllerParameters(runningOnRealRobot, selectedVersion.getDistanceAttachmentPlateHand());
      walkingControllerParameters = new AtlasWalkingControllerParameters(target, jointMap);
      stateEstimatorParameters = new AtlasStateEstimatorParameters(jointMap, sensorInformation, runningOnRealRobot, getEstimatorDT());
      multiContactControllerParameters = new AtlasRobotMultiContactControllerParameters(jointMap);
      drivingControllerParameters = new AtlasDrivingControllerParameters(jointMap);
      defaultArmConfigurations = new AtlasDefaultArmConfigurations();
      heightCalculatorParameters = new AtlasHeightCalculatorParameters();
      snappingParameters = new AtlasFootstepSnappingParameters();
   }

   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return armControllerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParameters;
   }

   @Override
   public AtlasPhysicalProperties getPhysicalProperties()
   {
      return new AtlasPhysicalProperties();
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
   public Transform getJmeTransformWristToHand(RobotSide side)
   {
      return selectedVersion.getOffsetFromAttachmentPlate(side);
   }

   @Override
   public RigidBodyTransform getTransform3dWristToHand(RobotSide side)
   {
      return JMEGeometryUtils.transformFromJMECoordinatesToZup(getJmeTransformWristToHand(side));
   }

   @Override
   public String toString()
   {
      return selectedVersion.toString();
   }

   @Override
   public DRCRobotInitialSetup<SDFHumanoidRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new AtlasSimInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return multiContactControllerParameters;
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure(SDFRobot sdfRobot)
   {
      return new AtlasPhysicsEngineConfiguration(getJointMap(), sdfRobot);
   }

   @Override
   public AtlasContactPointParameters getContactPointParameters()
   {
      return jointMap.getContactPointParameters();
   }

   public void createHandContactPoints(boolean useHighResolutionPointGrid)
   {
      jointMap.getContactPointParameters().createHandContactPoints(useHighResolutionPointGrid);
   }

   public void addMoreFootContactPointsSimOnly()
   {
      jointMap.getContactPointParameters().addMoreFootContactPointsSimOnly();
   }

   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      AtlasDampingParameters.setDampingParameters(simulatedRobot, getDRCHandType(), getJointMap());
   }

   @Override
   public void setEnableJointDamping(boolean enableJointDamping)
   {
      this.enableJointDamping = enableJointDamping;
   }

   @Override
   public boolean getEnableJointDamping()
   {
      return enableJointDamping;
   }

   @Override
   public HandModel getHandModel()
   {
      if (selectedVersion.hasRobotiqHands())
         return new RobotiqHandModel();

      return null;
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return drivingControllerParameters;
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public SDFFullHumanoidRobotModel createFullRobotModel()
   {
      SDFFullHumanoidRobotModel fullRobotModel = loader.createFullRobotModel(getJointMap(), sensorInformation.getSensorFramesToTrack());
      for (RobotSide robotSide : RobotSide.values())
      {
         ArmJointName[] armJointNames = new ArmJointName[] {ArmJointName.FIRST_WRIST_PITCH, ArmJointName.WRIST_ROLL, ArmJointName.SECOND_WRIST_PITCH};

         for (ArmJointName armJointName : armJointNames)
         {
            double lowerLimit = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitLower();
            double upperLimit = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitUpper();

            double range = upperLimit - lowerLimit;

            if (range > 2.0 * HARDSTOP_RESTRICTION_ANGLE)
            {
               double safeLowerBound = lowerLimit + HARDSTOP_RESTRICTION_ANGLE;
               double safeUpperBound = upperLimit - HARDSTOP_RESTRICTION_ANGLE;


               fullRobotModel.getArmJoint(robotSide, armJointName).setJointLimitLower(safeLowerBound);
               fullRobotModel.getArmJoint(robotSide, armJointName).setJointLimitUpper(safeUpperBound);
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
   public SDFHumanoidRobot createSdfRobot(boolean createCollisionMeshes)
   {
      boolean useCollisionMeshes = false;
      boolean enableTorqueVelocityLimits = false;
      AtlasJointMap jointMap = getJointMap();
      boolean enableJointDamping = getEnableJointDamping();

      return loader.createRobot(jointMap.getModelName(), jointMap, useCollisionMeshes, enableTorqueVelocityLimits, enableJointDamping);
   }

   @Override
   public double getSimulateDT()
   {
      return 0.0001;
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
   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      if (target == DRCRobotModel.RobotTarget.REAL_ROBOT)
      {
         return AtlasPPSTimestampOffsetProvider.getInstance(sensorInformation);
      }

      if ((target == DRCRobotModel.RobotTarget.SCS) && DRCConfigParameters.SEND_ROBOT_DATA_TO_ROS)
      {
         return new SimulationRosClockPPSTimestampOffsetProvider();
      }

      return new AlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return new AtlasSensorSuiteManager(this, getCollisionBoxProvider(), getPPSTimestampOffsetProvider(), sensorInformation, getJointMap(), getPhysicalProperties(),
                                         getFootstepParameters(), target);
   }

   @Override
   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public SideDependentList<HandCommandManager> createHandCommandManager()
   {
      if (target == DRCRobotModel.RobotTarget.REAL_ROBOT)
      {
         SideDependentList<HandCommandManager> handCommandManagers = new SideDependentList<HandCommandManager>();
         switch (selectedVersion)
         {
            case ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI :
               handCommandManagers.set(RobotSide.LEFT, new RobotiqHandCommandManager(RobotSide.LEFT));
               return handCommandManagers;

            case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ :
               handCommandManagers.set(RobotSide.LEFT, new RobotiqHandCommandManager(RobotSide.LEFT));
               handCommandManagers.set(RobotSide.RIGHT, new RobotiqHandCommandManager(RobotSide.RIGHT));
               return handCommandManagers;

            case ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS :
               break;

            case ATLAS_UNPLUGGED_V5_NO_HANDS :
               break;

            default :
               break;
         }
      }

      return null;
   }

   @Override
   public DRCHandType getDRCHandType()
   {
      return selectedVersion.getHandModel();
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(SDFRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
           GlobalDataProducer globalDataProducer, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      switch (getDRCHandType())
      {
         case ROBOTIQ :
            return new SimulatedRobotiqHandsController(simulatedRobot, this, threadDataSynchronizer, globalDataProducer, closeableAndDisposableRegistry);

         default :
            return null;
      }
   }

   @Override
   public FootstepPlanningParameterization getFootstepParameters()
   {
      // TODO Auto-generated method stub
      return new AtlasFootstepPlanningParameterization();
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), selectedVersion.getSdfFileAsStream(), selectedVersion.getResourceDirectories());
   }

   @Override
   public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      return null;
   }

   @Override
   public LogSettings getLogSettings()
   {
      return getLogSettings(true);
   }

   public LogSettings getLogSettings(boolean useCameras)
   {
      switch (target)
      {
      case REAL_ROBOT :
         if(useCameras)
         {
            return LogSettings.ATLAS_IAN;
         }
         else
         {
            return LogSettings.ATLAS_NO_CAMERAS;
         }

      case GAZEBO :
      case SCS:
      default :
         return LogSettings.SIMULATION;
      }
   }

   @Override
   public DefaultArmConfigurations getDefaultArmConfigurations()
   {
      return defaultArmConfigurations;
   }

   @Override
   public ImmutablePair<Class<?>, String[]> getOperatorInterfaceStarter()
   {
      String[] args = {"-m " + getAtlasVersion().name()};

      return new ImmutablePair<Class<?>, String[]>(AtlasOperatorUserInterface.class, args);
   }

   @Override
   public Class<?> getSpectatorInterfaceClass()
   {
      return AtlasSpectatorInterface.class;
   }

   @Override
   public HeightCalculatorParameters getHeightCalculatorParameters()
   {
      return heightCalculatorParameters;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return new AtlasCollisionBoxProvider(loader, getJointMap());
   }

   @Override
   public FootstepSnappingParameters getSnappingParameters()
   {
      return snappingParameters;
   }

   @Override
   public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return walkingControllerParameters.getSliderBoardControlledNeckJointsWithLimits();
   }
   
   @Override
   public SideDependentList<LinkedHashMap<String,ImmutablePair<Double,Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return walkingControllerParameters.getSliderBoardControlledFingerJointsWithLimits();
   }

   @Override
   public double getStandPrepAngle(String jointName)
   {
      System.err.println("Need to add access to stand prep joint angles.");
      return 0;
   }
}
