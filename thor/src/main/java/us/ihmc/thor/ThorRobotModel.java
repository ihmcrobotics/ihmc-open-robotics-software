package us.ihmc.thor;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;

import org.apache.commons.lang3.StringUtils;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
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
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.thor.configuration.ThorConfigurationRoot;
import us.ihmc.thor.parameters.ThorCapturePointPlannerParameters;
import us.ihmc.thor.parameters.ThorContactPointParameters;
import us.ihmc.thor.parameters.ThorHighLevelControllerParameters;
import us.ihmc.thor.parameters.ThorJointMap;
import us.ihmc.thor.parameters.ThorSensorInformation;
import us.ihmc.thor.parameters.ThorStateEstimatorParameters;
import us.ihmc.thor.parameters.ThorUIParameters;
import us.ihmc.thor.parameters.ThorWalkingControllerParameters;
import us.ihmc.thor.sensors.ThorSensorSuiteManager;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.UIParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

public class ThorRobotModel implements DRCRobotModel, SDFDescriptionMutator
{
   private static final boolean PRINT_MODEL = false;

   private final ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final StateEstimatorParameters stateEstimatorParamaters;
   private final HighLevelControllerParameters highLevelControllerParameters;
   private final ThorSensorInformation sensorInformation;
   private final ThorJointMap jointMap;
   private final ThorContactPointParameters contactPointParameters;
   private final DRCHandType drcHandType = DRCHandType.NONE;
   private final String robotName = "THOR";
   private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>();
   private final RobotTarget target;

   private final String[] resourceDirectories;
   {
         resourceDirectories = new String[]{
               "models/",
               "models/gazebo/",
               "models/thor_description/",
               "models/thor_description/sdf/",
            };
   }

   private final JaxbSDFLoader loader;
   private final RobotDescription robotDescription;

   public ThorRobotModel(RobotTarget target, boolean headless)
   {
      this(target,headless, "DEFAULT");
   }

   public ThorRobotModel(RobotTarget target, boolean headless, String model)
   {
      this.target = target;
      jointMap = new ThorJointMap();
      contactPointParameters = new ThorContactPointParameters(jointMap);
      sensorInformation = new ThorSensorInformation(target);
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

      for (String forceSensorNames : ThorSensorInformation.forceSensorNames)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         if (forceSensorNames.equals("l_ankle_roll") && target != RobotTarget.GAZEBO)
         {
            transform.set(ThorSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.LEFT));
         }
         else if (forceSensorNames.equals("r_ankle_roll") && target != RobotTarget.GAZEBO)
         {
            transform.set(ThorSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.RIGHT));
         }

         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, transform);
      }

      for(RobotSide side : RobotSide.values())
      {
         for(String parentJointName : ThorSensorInformation.contactSensors.get(side).keySet())
         {
            for(String sensorName : ThorSensorInformation.contactSensors.get(side).get(parentJointName).keySet())
            {
               loader.addContactSensor(jointMap,sensorName, parentJointName, ThorSensorInformation.contactSensors.get(side).get(parentJointName).get(sensorName));
            }
         }
      }

      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;
      capturePointPlannerParameters = new ThorCapturePointPlannerParameters(runningOnRealRobot);
      walkingControllerParameters = new ThorWalkingControllerParameters(target, jointMap);
      stateEstimatorParamaters = new ThorStateEstimatorParameters(runningOnRealRobot, getEstimatorDT(), sensorInformation, jointMap);
      highLevelControllerParameters = new ThorHighLevelControllerParameters(jointMap);
      robotDescription = createRobotDescription();
   }

   private RobotDescription createRobotDescription()
   {
      boolean useCollisionMeshes = false;
      boolean enableTorqueVelocityLimits = true;
      boolean enableJointDamping = true;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      RobotDescription robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, contactPointParameters,
            useCollisionMeshes);
      return robotDescription;
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }


   @Override
   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
   {
	   return capturePointPlannerParameters;
   }

   @Override
   public UIParameters getUIParameters()
   {
      return new ThorUIParameters();
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
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParamaters;
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return jointMap;
   }

   @Override
   public double getStandPrepAngle(String jointName)
   {
      System.err.println("Need to add access to stand prep joint angles.");
      return 0;
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

   private void createTransforms()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         Vector3f centerOfHandToWristTranslation = new Vector3f();
         float[] angles = new float[3];

         centerOfHandToWristTranslation = new Vector3f(0f, robotSide.negateIfLeftSide(0.015f), -0.06f);
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
         return ThorConfigurationRoot.REAL_ROBOT_SDF_FILE;
      else
         return ThorConfigurationRoot.SIM_SDF_FILE;
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
      return new ThorInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public RobotContactPointParameters getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public HandModel getHandModel()
   {
      return null; // // FIXME: 11/19/16
   }

   @Override
   public ThorSensorInformation getSensorInformation()
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
      HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot = new HumanoidFloatingRootJointRobot(robotDescription, jointMap, enableJointDamping, enableTorqueVelocityLimits);
      return humanoidFloatingRootJointRobot;
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
      return new ThorSensorSuiteManager(this, getPPSTimestampOffsetProvider(), sensorInformation, contactPointParameters, target);
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer, HumanoidGlobalDataProducer globalDataProducer, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
	   return null;
      //return new ThorFingerController(this, simulatedRobot, threadDataSynchronizer, globalDataProducer, null);
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
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
         return null;
      }
      else
      {
         return LogSettings.SIMULATION;
      }
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Thor";
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

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      return null;
   }
}
