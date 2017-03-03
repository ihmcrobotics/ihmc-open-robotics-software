package us.ihmc.valkyrie;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.LinkedHashMap;
import java.util.Map;

import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.tuple.ImmutablePair;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.avatar.handControl.HandCommandManager;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepPlanningParameterization;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrie.configuration.YamlWithIncludesLoader;
import us.ihmc.valkyrie.fingers.ValkyrieHandModel;
import us.ihmc.valkyrie.parameters.ValkyrieArmControllerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieCapturePointPlannerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrie.parameters.ValkyrieStateEstimatorParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;
import us.ihmc.valkyrie.sensors.ValkyrieSensorSuiteManager;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public class ValkyrieRobotModel implements DRCRobotModel, SDFDescriptionMutator
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
   private final Map<String, Double> standPrepAngles = (Map<String, Double>) YamlWithIncludesLoader.load("standPrep", "setpoints.yaml");
   private final DRCRobotModel.RobotTarget target;

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
   private final RobotDescription robotDescription;

   private boolean enableJointDamping = true;

   public ValkyrieRobotModel(DRCRobotModel.RobotTarget target, boolean headless, FootContactPoints simulationContactPoints)
   {
      this(target,headless, "DEFAULT", simulationContactPoints);
   }

   public ValkyrieRobotModel(DRCRobotModel.RobotTarget target, boolean headless)
   {
      this(target,headless, "DEFAULT", null);
   }

   public ValkyrieRobotModel(DRCRobotModel.RobotTarget target, boolean headless, String model)
   {
      this(target, headless, model, null);
   }

   public ValkyrieRobotModel(DRCRobotModel.RobotTarget target, boolean headless, String model, FootContactPoints simulationContactPoints)
   {
      this.target = target;
      jointMap = new ValkyrieJointMap(simulationContactPoints);
      physicalProperties = new ValkyriePhysicalProperties();
      sensorInformation = new ValkyrieSensorInformation(target);
      InputStream sdf = null;

      if(model.equalsIgnoreCase("DEFAULT"))
      {
         System.out.println("Loading robot model from: '"+getSdfFile()+"'");
         sdf=getSdfFileAsStream();
      }
      else
      {
         System.out.println("Loading robot model from: '"+model+"'");
         sdf=getClass().getClassLoader().getResourceAsStream(model);
         if(sdf==null)
         {
            try
            {
               sdf=new FileInputStream(model);
            }
            catch (FileNotFoundException e)
            {
               System.err.println("failed to load sdf file - file not found");
            }
         }

      }

      this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), sdf, this);

      for (String forceSensorNames : ValkyrieSensorInformation.forceSensorNames)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         if (forceSensorNames.equals("leftAnkleRoll") && target != RobotTarget.GAZEBO)
         {
            transform.set(ValkyrieSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.LEFT));
         }
         else if (forceSensorNames.equals("rightAnkleRoll") && target != RobotTarget.GAZEBO)
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

      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;
      capturePointPlannerParameters = new ValkyrieCapturePointPlannerParameters(runningOnRealRobot);
      armControllerParameters = new ValkyrieArmControllerParameters(jointMap, target);
      walkingControllerParameters = new ValkyrieWalkingControllerParameters(jointMap, target);
      stateEstimatorParamaters = new ValkyrieStateEstimatorParameters(runningOnRealRobot, getEstimatorDT(), sensorInformation, jointMap);
      robotDescription = createRobotDescription();
   }

   private RobotDescription createRobotDescription()
   {
      boolean useCollisionMeshes = false;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      RobotDescription robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, useCollisionMeshes);
      return robotDescription;
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }


   @Override
   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
	   return capturePointPlannerParameters;
   }

   @Override
   public ICPOptimizationParameters getICPOptimizationParameters()
   {
      return null;
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
      return JMEGeometryUtils.transformFromJMECoordinatesToZup(getJmeTransformWristToHand(side));
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
      if(this.target == RobotTarget.REAL_ROBOT)
         return ValkyrieConfigurationRoot.REAL_ROBOT_SDF_FILE;
      else
         return ValkyrieConfigurationRoot.SIM_SDF_FILE;
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
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new ValkyrieInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public RobotContactPointParameters getContactPointParameters()
   {
      return jointMap.getContactPointParameters();
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
   public ValkyrieSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return new FullHumanoidRobotModelFromDescription(robotDescription, jointMap, sensorInformation.getSensorFramesToTrack());
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
   {
      boolean enableTorqueVelocityLimits = false;
      boolean enableJointDamping = getEnableJointDamping();

      HumanoidFloatingRootJointRobot sdfRobot = new HumanoidFloatingRootJointRobot(robotDescription, jointMap, enableJointDamping, enableTorqueVelocityLimits);

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
      return 0.0001; //0.00003875;
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
      return new ValkyrieSensorSuiteManager(this, getPPSTimestampOffsetProvider(), sensorInformation, jointMap, target);
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
   public MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer, HumanoidGlobalDataProducer globalDataProducer, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
	   return null;
      //return new ValkyrieFingerController(this, simulatedRobot, threadDataSynchronizer, globalDataProducer, null);
   }

   @Override
   public FootstepPlanningParameterization getFootstepParameters()
   {
      return new FootstepPlanningParameterization();
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
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
      if(target == RobotTarget.REAL_ROBOT)
      {
         if(useCameras)
            return LogSettings.VALKYRIE_JSC;
         else
            return LogSettings.VALKYRIE_NO_CAMERAS;
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
   public String getSimpleRobotName()
   {
      return "Valkyrie";
   }

   public String getFullRobotName()
   {
      String fullRobotName = getSdfFile();
      fullRobotName = fullRobotName.substring(fullRobotName.lastIndexOf("/") + 1, fullRobotName.length());
      fullRobotName = StringUtils.capitalize(fullRobotName);
      fullRobotName = StringUtils.remove(fullRobotName, "_hw");
      fullRobotName = StringUtils.remove(fullRobotName, "_sim");
      fullRobotName = StringUtils.remove(fullRobotName, ".sdf");

      return fullRobotName;
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
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      if(this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {
      if(this.jointMap.getModelName().equals(model.getName()))
      {
         switch(linkHolder.getName())
         {
            case "hokuyo_link":
               modifyHokuyoInertia(linkHolder);
               break;
            default:
               break;
         }

      }
   }

   @Override
   public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor)
   {
      if(this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {
      if(this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {
      if(this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {
      if(this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   private void modifyHokuyoInertia(SDFLinkHolder linkHolder)
   {
      linkHolder.getInertia().setM00(0.000401606); // i_xx
      linkHolder.getInertia().setM01(4.9927e-08); // i_xy
      linkHolder.getInertia().setM02(1.0997e-05); // i_xz
      linkHolder.getInertia().setM11(0.00208115); // i_yy
      linkHolder.getInertia().setM12(-9.8165e-09); // i_yz
      linkHolder.getInertia().setM22(0.00178402); // i_zz
   }
}
