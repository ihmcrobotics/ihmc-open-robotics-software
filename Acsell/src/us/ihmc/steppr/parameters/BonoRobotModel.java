package us.ihmc.steppr.parameters;

import java.io.InputStream;
import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import com.jme3.math.Transform;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.acsell.initialSetup.BonoInitialSetup;
import us.ihmc.acsell.network.AcsellSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.HeightCalculatorParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepPlanningParameterization;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.physics.ScsCollisionConfigure;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.steppr.controlParameters.BonoArmControlParameters;
import us.ihmc.steppr.controlParameters.BonoCapturePointPlannerParameters;
import us.ihmc.steppr.controlParameters.BonoStateEstimatorParameters;
import us.ihmc.steppr.controlParameters.BonoWalkingControllerParameters;
import us.ihmc.steppr.hardware.controllers.StepprOutputProcessor;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public class BonoRobotModel implements DRCRobotModel
{
   private static final double SIMULATE_DT = 0.0001;
   private static final double CONTROLLER_DT = 0.004;
   private static final double ESTIMATOR_DT = 0.001;

   private final String[] resourceDirectories = new String[] { "models/axl/", "models/axl/axl_description/", "models/axl/axl_description/bono/", };

   private final boolean runningOnRealRobot;
   private final JaxbSDFLoader loader;
   private final BonoJointMap jointMap = new BonoJointMap();
   private final DRCRobotSensorInformation sensorInformation;
   private final BonoArmControlParameters armControlParameters;
   private final BonoCapturePointPlannerParameters capturePointPlannerParameters;
   private final BonoWalkingControllerParameters walkingControllerParameters;
   private final BonoWalkingControllerParameters multiContactControllerParameters;
   
   private boolean enableJointDamping = true;

   @Override
   public WholeBodyIkSolver createWholeBodyIkSolver()  
   {
      return null;
   }

   public BonoRobotModel(boolean runningOnRealRobot, boolean headless)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      sensorInformation = new BonoSensorInformation();

      if (headless)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[] {}, getSdfFileAsStream(), true, null);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), false, null);
      }

      for (String forceSensorNames : getSensorInformation().getForceSensorNames())
      {
         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, new RigidBodyTransform());
      }

      capturePointPlannerParameters = new BonoCapturePointPlannerParameters(runningOnRealRobot);
      armControlParameters = new BonoArmControlParameters(runningOnRealRobot);
      walkingControllerParameters = new BonoWalkingControllerParameters(jointMap, runningOnRealRobot);
      multiContactControllerParameters = new BonoWalkingControllerParameters(jointMap, runningOnRealRobot);
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
      BonoStateEstimatorParameters stateEstimatorParameters = new BonoStateEstimatorParameters(runningOnRealRobot, getEstimatorDT());
      return stateEstimatorParameters;
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return new BonoPhysicalProperties();
   }

   @Override
   public BonoJointMap getJointMap()
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
      return "models/axl/axl_description/bono/robots/bono.sdf";
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
      return "BONO";
   }

   @Override
   public DRCRobotInitialSetup<SDFHumanoidRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new BonoInitialSetup(groundHeight, initialYaw);
   }

   // XXX: fix this
   @Override
   public RobotContactPointParameters getContactPointParameters()
   {
      return jointMap.getContactPointParameters();
   }

   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      System.err.println("Joint Damping not setup for Bono. BonoRobotModel setJointDamping!");
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
   public SDFFullHumanoidRobotModel createFullRobotModel()
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
         HumanoidGlobalDataProducer globalDataProducers, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
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
   public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      return new StepprOutputProcessor(controllerFullRobotModel);
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
   public HeightCalculatorParameters getHeightCalculatorParameters()
   {
      return null;
   }

   @Override public String getSimpleRobotName()
   {
      return "STEPPR";
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
