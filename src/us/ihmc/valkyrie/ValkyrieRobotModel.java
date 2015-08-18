package us.ihmc.valkyrie;

import java.io.InputStream;
import java.util.LinkedHashMap;
import java.util.Map;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
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
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrie.fingers.ValkyrieHandModel;
import us.ihmc.valkyrie.parameters.ValkyrieArmControllerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieCapturePointPlannerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrie.parameters.ValkyrieStateEstimatorParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;
import us.ihmc.valkyrie.roboNet.schedule.YamlWithIncludesLoader;
import us.ihmc.valkyrie.sensors.ValkyrieSensorSuiteManager;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotContactPointParameters;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public class ValkyrieRobotModel implements DRCRobotModel
{
   private static final boolean PRINT_MODEL = false;
   
   private final CapturePointPlannerParameters capturePointPlannerParameters;
   private final ArmControllerParameters armControllerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final StateEstimatorParameters stateEstimatorParamaters;
   private final DRCRobotPhysicalProperties physicalProperties;
   private final ValkyrieSensorInformation sensorInformation;
   private final ValkyrieJointMap jointMap;
   private final DRCHandType drcHandType = DRCHandType.VALKYRIE;
   private final String robotName = "VALKYRIE";
   private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>();
   private final boolean runningOnRealRobot;
   private final Map<String, Double> standPrepAngles = (Map<String, Double>) YamlWithIncludesLoader.load(ValkyrieConfigurationRoot.class, "standPrep", "setpoints.yaml");
   
   @Override
   public WholeBodyIkSolver createWholeBodyIkSolver()  
   {
      return null;
   }

   private final String[] resourceDirectories;
   {
         resourceDirectories = new String[]{
               "models/",
               "models/gazebo/",
               "models/val_description/",
               "models/val_description/sdf/",
            };
   }

   private final JaxbSDFLoader loader;

   private boolean enableJointDamping = true;

   public ValkyrieRobotModel(boolean runningOnRealRobot, boolean headless)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      jointMap = new ValkyrieJointMap();
      physicalProperties = new ValkyriePhysicalProperties();
      sensorInformation = new ValkyrieSensorInformation(runningOnRealRobot);

      if (headless)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[] {}, getSdfFileAsStream(), true);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), false);
      }

      for (String forceSensorNames : ValkyrieSensorInformation.forceSensorNames)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         if (forceSensorNames.equals("LeftAnkleRoll"))
         {
            transform.set(ValkyrieSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.LEFT));
         }
         else if (forceSensorNames.equals("RightAnkleRoll"))
         {
            transform.set(ValkyrieSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.RIGHT));
         }

         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, transform);
      }
      
      for(RobotSide side : RobotSide.values())
      {
         for(String parentJointName : ValkyrieSensorInformation.contactSensors.get(side).keySet())
         {
            for(String sensorName : ValkyrieSensorInformation.contactSensors.get(side).get(parentJointName).keySet())
            {
               loader.addContactSensor(jointMap,sensorName, parentJointName, ValkyrieSensorInformation.contactSensors.get(side).get(parentJointName).get(sensorName));
            }
         }
      }

      capturePointPlannerParameters = new ValkyrieCapturePointPlannerParameters(runningOnRealRobot);
      armControllerParameters = new ValkyrieArmControllerParameters(runningOnRealRobot);
      walkingControllerParameters = new ValkyrieWalkingControllerParameters(jointMap, runningOnRealRobot);
      stateEstimatorParamaters = new ValkyrieStateEstimatorParameters(runningOnRealRobot, getEstimatorDT(), sensorInformation, jointMap);
   }
   
   @Override
   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
	   return capturePointPlannerParameters;
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
      return stateEstimatorParamaters;
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return physicalProperties;
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return jointMap;
   }
   
   public double getStandPrepAngle(String jointName)
   {
      return standPrepAngles.get(jointName);
   }
   
   @Override
   public LinkedHashMap<NeckJointName,ImmutablePair<Double,Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return walkingControllerParameters.getSliderBoardControlledNeckJointsWithLimits();
   }
   
   @Override
   public SideDependentList<LinkedHashMap<String,ImmutablePair<Double,Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return walkingControllerParameters.getSliderBoardControlledFingerJointsWithLimits();
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide side)
   {
      //      if (offsetHandFromWrist.get(side) == null)
      //      {
      createTransforms();
      //      }

      return offsetHandFromWrist.get(side);
   }
   
   @Override
   public RigidBodyTransform getTransform3dWristToHand(RobotSide side)
   {
      return JMEGeometryUtils.transformFromJMECoordinatesToZup( getJmeTransformWristToHand(side));
   }

   private void createTransforms()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         Vector3f centerOfHandToWristTranslation = new Vector3f();
         float[] angles = new float[3];

         centerOfHandToWristTranslation = new Vector3f(0f, (float) robotSide.negateIfLeftSide(0.015f), -0.06f);
         angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
         angles[1] = 0.0f;
         angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
         //       
         Quaternion centerOfHandToWristRotation = new Quaternion(angles);
         offsetHandFromWrist.set(robotSide, new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation));
      }
   }

   private String getSdfFile()
   {
      return ValkyrieConfigurationRoot.SDF_FILE;
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
      return robotName;
   }

   @Override
   public DRCRobotInitialSetup<SDFHumanoidRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new ValkyrieInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure(SDFRobot robotModel)
   {
      return null;
   }

   @Override
   public DRCRobotContactPointParameters getContactPointParameters()
   {
      return jointMap.getContactPointParameters();
   }

   //For Sim Only
   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      System.err.println("Joint Damping not setup for Valkyrie. ValkyrieRobotModel setJointDamping!");
   }
   
   @Override
   public void setEnableJointDamping(boolean enableJointDamping)
   {
      this.enableJointDamping   = enableJointDamping;
   }

   @Override
   public boolean getEnableJointDamping()
   {
      return enableJointDamping;
   }

   @Override
   public HandModel getHandModel()
   {
      return new ValkyrieHandModel();
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return getWalkingControllerParameters();
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public SDFFullRobotModel createFullRobotModel()
   {
      return loader.createFullRobotModel(getJointMap(), sensorInformation.getSensorFramesToTrack());
   }

   @Override
   public SDFHumanoidRobot createSdfRobot(boolean createCollisionMeshes)
   { 
      boolean useCollisionMeshes = false;
      boolean enableTorqueVelocityLimits = false;
      SDFJointNameMap jointMap = getJointMap();
      boolean enableJointDamping = getEnableJointDamping();

      SDFHumanoidRobot sdfRobot =  loader.createRobot(jointMap.getModelName(), jointMap, useCollisionMeshes, enableTorqueVelocityLimits, enableJointDamping);

      if (PRINT_MODEL)
      {
         System.out.println("\nValkyrieRobotModel Link Masses:");

         StringBuffer stringBuffer = new StringBuffer();
         sdfRobot.printRobotJointsAndMasses(stringBuffer);
         System.out.println(stringBuffer);
         System.out.println();

         System.out.println("ValkyrieRobotModel: \n" + sdfRobot);
      }

      return sdfRobot;
   }

   @Override
   public double getSimulateDT()
   {
      return 0.00003875;
   }

   @Override
   public double getEstimatorDT()
   {
      return 0.002;
   }

   @Override
   public double getControllerDT()
   {
      return 0.004;
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
      return new ValkyrieSensorSuiteManager(this, getPPSTimestampOffsetProvider(), sensorInformation, jointMap, runningOnRealRobot);
   }

   @Override
   public SideDependentList<HandCommandManager> createHandCommandManager()
   {
	   return null;
   }

   @Override
   public DRCHandType getDRCHandType()
   {
      return drcHandType;
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(SDFRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer, GlobalDataProducer globalDataProducer)
   {
	   return null;
      //return new ValkyrieFingerController(this, simulatedRobot, threadDataSynchronizer, globalDataProducer, null);
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
      return null;
   }
   
   @Override
   public LogSettings getLogSettings()
   {
      if(runningOnRealRobot)
      {
         return LogSettings.VALKYRIE_IHMC;
      }
      else
      {
         return LogSettings.SIMULATION;
      }
   }

   @Override
   public DefaultArmConfigurations getDefaultArmConfigurations()
   {
      return null;
   }

   @Override
   public ImmutablePair<Class<?>, String[]> getOperatorInterfaceStarter()
   {
      String[] args = runningOnRealRobot ? new String[]{"--realRobot"} : null;
      return new ImmutablePair<Class<?>, String[]>(ValkyrieOperatorUserInterface.class, args);
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
      return "Valkyrie";
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
}
