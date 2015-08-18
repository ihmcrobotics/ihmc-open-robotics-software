package us.ihmc.wanderer.parameters;

import java.io.InputStream;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.acsell.network.AcsellSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.HeightCalculatorParameters;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.pathGeneration.footstepPlanner.FootstepPlanningParameterization;
import us.ihmc.pathGeneration.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.physics.ScsCollisionConfigure;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.humanoidRobotics.model.BaseFullRobotModel;
import us.ihmc.humanoidRobotics.partNames.NeckJointName;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.wanderer.controlParameters.WandererArmControlParameters;
import us.ihmc.wanderer.controlParameters.WandererCapturePointPlannerParameters;
import us.ihmc.wanderer.controlParameters.WandererStateEstimatorParameters;
import us.ihmc.wanderer.controlParameters.WandererWalkingControllerParameters;
import us.ihmc.wanderer.hardware.controllers.WandererOutputProcessor;
import us.ihmc.wanderer.initialSetup.WandererInitialSetup;
import us.ihmc.wanderer.operatorInterface.WandererOperatorInterface;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

import com.jme3.math.Transform;

public class WandererRobotModel implements DRCRobotModel
{
   private static final double SIMULATE_DT = 0.0001;
   private static final double CONTROLLER_DT = 0.004;
   private static final double ESTIMATOR_DT = 0.001;

   private final String[] resourceDirectories = new String[] { "models/wanderer/" };

   private final boolean runningOnRealRobot;
   private final JaxbSDFLoader loader;
   private final WandererJointMap jointMap = new WandererJointMap();
   private final DRCRobotSensorInformation sensorInformation;
   private final WandererArmControlParameters armControlParameters;
   private final WandererCapturePointPlannerParameters capturePointPlannerParameters;
   private final WandererWalkingControllerParameters walkingControllerParameters;
   private final WandererWalkingControllerParameters multiContactControllerParameters;
   
   private boolean enableJointDamping = true;

   @Override
   public WholeBodyIkSolver createWholeBodyIkSolver()  
   {
      return null;
   }

   public WandererRobotModel(boolean runningOnRealRobot, boolean headless)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      sensorInformation = new WandererSensorInformation();

      if (headless)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[] {}, getSdfFileAsStream(), true);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), false);
      }

      for (String forceSensorNames : getSensorInformation().getForceSensorNames())
      {
         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, new RigidBodyTransform());
      }

      capturePointPlannerParameters = new WandererCapturePointPlannerParameters(runningOnRealRobot);
      armControlParameters = new WandererArmControlParameters(runningOnRealRobot);
      walkingControllerParameters = new WandererWalkingControllerParameters(jointMap, runningOnRealRobot);
      multiContactControllerParameters = new WandererWalkingControllerParameters(jointMap, runningOnRealRobot);
   }

   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return armControlParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return multiContactControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      WandererStateEstimatorParameters stateEstimatorParameters = new WandererStateEstimatorParameters(runningOnRealRobot, getEstimatorDT());
      return stateEstimatorParameters;
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return new WandererPhysicalProperties();
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
   
   @Override
   public RigidBodyTransform getTransform3dWristToHand(RobotSide side)
   {
      return JMEGeometryUtils.transformFromJMECoordinatesToZup( getJmeTransformWristToHand(side));
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
   public DRCRobotInitialSetup<SDFHumanoidRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new WandererInitialSetup(groundHeight, initialYaw);
   }

   // XXX: fix this
   @Override
   public DRCRobotContactPointParameters getContactPointParameters()
   {
      return jointMap.getContactPointParameters();
   }

   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      System.err.println("Joint Damping not setup for Wanderer. WandererRobotModel setJointDamping!");
   }
   
   @Override
   public void setEnableJointDamping(boolean enableJointDamping)
   {
      this.enableJointDamping  = enableJointDamping;
   }

   @Override
   public boolean getEnableJointDamping()
   {
      return enableJointDamping;
   }

   @Override
   public HandModel getHandModel()
   {
      return null;
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure(SDFRobot robotModel)
   {
      return null;
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return null;
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public SDFFullRobotModel createFullRobotModel()
   {
      return loader.createFullRobotModel(getJointMap());
   }

   @Override
   public SDFHumanoidRobot createSdfRobot(boolean createCollisionMeshes)
   { 
      boolean useCollisionMeshes = false;
      boolean enableTorqueVelocityLimits = false;
      SDFJointNameMap jointMap = getJointMap();
      boolean enableJointDamping = getEnableJointDamping();
      return loader.createRobot(jointMap.getModelName(), jointMap, useCollisionMeshes, enableTorqueVelocityLimits, enableJointDamping);
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

   @Override
   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public PPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      return new AlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return new AcsellSensorSuiteManager(createFullRobotModel(), runningOnRealRobot);
   }

   @Override
   public SideDependentList<HandCommandManager> createHandCommandManager()
   {
      return null;
   }

   @Override
   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public DRCHandType getDRCHandType()
   {
      return DRCHandType.NONE;
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(SDFRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
         GlobalDataProducer globalDataProducersw)
   {
      return null;
   }

   @Override
   public FootstepPlanningParameterization getFootstepParameters()
   {
      return null;
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
   }

   @Override
   public OutputProcessor getOutputProcessor(BaseFullRobotModel controllerFullRobotModel)
   {
      return new WandererOutputProcessor(controllerFullRobotModel);
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

   @Override
   public DefaultArmConfigurations getDefaultArmConfigurations()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ImmutablePair<Class<?>, String[]> getOperatorInterfaceStarter()
   {
      String[] args = runningOnRealRobot ? new String[]{"--realRobot"} : null;
      return new ImmutablePair<Class<?>, String[]>(WandererOperatorInterface.class, args);
   }

   @Override
   public Class<?> getSpectatorInterfaceClass()
   {
      
      
      return null;
   }

   @Override
   public HeightCalculatorParameters getHeightCalculatorParameters()
   {
      return null;
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
   public FootstepSnappingParameters getSnappingParameters()
   {
      return null;
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
