package us.ihmc.wanderer.parameters;

import java.io.InputStream;

import com.jme3.math.Transform;

import us.ihmc.acsell.network.AcsellSensorSuiteManager;
import us.ihmc.avatar.DRCSimulationOutputWriterForControllerThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wanderer.controlParameters.WandererCapturePointPlannerParameters;
import us.ihmc.wanderer.controlParameters.WandererStateEstimatorParameters;
import us.ihmc.wanderer.controlParameters.WandererUIParameters;
import us.ihmc.wanderer.controlParameters.WandererWalkingControllerParameters;
import us.ihmc.wanderer.initialSetup.WandererInitialSetup;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.UIParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

public class WandererRobotModel implements DRCRobotModel
{
   private static final double SIMULATE_DT = 0.0001;
   private static final double CONTROLLER_DT = 0.004;
   private static final double ESTIMATOR_DT = 0.001;

   private final String[] resourceDirectories = new String[] { "models/wanderer/" };

   private final boolean runningOnRealRobot;
   private final JaxbSDFLoader loader;
   private final WandererJointMap jointMap = new WandererJointMap();
   private final WandererContactPointParameters contactPointParameters = new WandererContactPointParameters(jointMap);
   private final DRCRobotSensorInformation sensorInformation;
   private final WandererCapturePointPlannerParameters capturePointPlannerParameters;
   private final WandererWalkingControllerParameters walkingControllerParameters;
   private final WandererHighLevelControllerParameters highLevelControllerParameters;

   private final RobotDescription robotDescription;

   public WandererRobotModel(boolean runningOnRealRobot, boolean headless)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      sensorInformation = new WandererSensorInformation();

      this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), null);

      for (String forceSensorNames : getSensorInformation().getForceSensorNames())
      {
         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, new RigidBodyTransform());
      }

      capturePointPlannerParameters = new WandererCapturePointPlannerParameters(runningOnRealRobot);
      walkingControllerParameters = new WandererWalkingControllerParameters(jointMap, runningOnRealRobot);
      highLevelControllerParameters = new WandererHighLevelControllerParameters(jointMap);
      robotDescription = createRobotDescription();
   }

   private RobotDescription createRobotDescription()
   {
      boolean useCollisionMeshes = false;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      RobotDescription robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, contactPointParameters, useCollisionMeshes);
      return robotDescription;
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      return highLevelControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      WandererStateEstimatorParameters stateEstimatorParameters = new WandererStateEstimatorParameters(runningOnRealRobot, getEstimatorDT());
      return stateEstimatorParameters;
   }

   @Override
   public WandererJointMap getJointMap()
   {
      return jointMap;
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide side)
   {
      return new Transform();
   }

   private String getSdfFile()
   {
      return "models/wanderer/wanderer.sdf";
   }

   private String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   private InputStream getSdfFileAsStream()
   {
      return getClass().getClassLoader().getResourceAsStream(getSdfFile());
   }

   @Override
   public String toString()
   {
      return "Wanderer";
   }

   @Override
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new WandererInitialSetup(groundHeight, initialYaw);
   }

   // XXX: fix this
   @Override
   public RobotContactPointParameters getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public HandModel getHandModel()
   {
      return null;
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return new FullHumanoidRobotModelFromDescription(robotDescription, jointMap, sensorInformation.getSensorFramesToTrack());
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;
      HumanoidJointNameMap jointMap = getJointMap();
      return new HumanoidFloatingRootJointRobot(robotDescription, jointMap, enableJointDamping, enableTorqueVelocityLimits);
   }

   @Override
   public double getSimulateDT()
   {
      return SIMULATE_DT;
   }

   @Override
   public double getEstimatorDT()
   {
      return ESTIMATOR_DT;
   }

   @Override
   public double getControllerDT()
   {
      return CONTROLLER_DT;
   }

   private GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      return new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return new AcsellSensorSuiteManager(createFullRobotModel(), runningOnRealRobot);
   }

   @Override
   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public UIParameters getUIParameters()
   {
      return new WandererUIParameters();
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
         HumanoidGlobalDataProducer globalDataProducersw, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      return null;
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
   }

   @Override
   public LogSettings getLogSettings()
   {
      if(runningOnRealRobot)
      {
         return LogSettings.STEPPR_IHMC;
      }
      else
      {
         return LogSettings.SIMULATION;
      }
   }

   @Override public String getSimpleRobotName()
   {
      return "WANDERER";
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return null;
   }

   @Override
   public double getStandPrepAngle(String jointName)
   {
      System.err.println("Need to add access to stand prep joint angles.");
      return 0;
   }


   @Override
   public DRCOutputProcessor getCustomSimulationOutputProcessor(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot)
   {
      return new DRCSimulationOutputWriterForControllerThread(humanoidFloatingRootJointRobot);
   }

   @Override
   public JointDesiredOutputWriter getCustomSimulationOutputWriter(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot)
   {
      return null;
   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      return null;
   }
}
