package us.ihmc.atlas;

import java.util.LinkedHashMap;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import com.jme3.math.Transform;

import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.AtlasArmControllerParameters;
import us.ihmc.atlas.parameters.AtlasCapturePointPlannerParameters;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasDefaultArmConfigurations;
import us.ihmc.atlas.parameters.AtlasFootstepPlanningParameterization;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.parameters.AtlasStateEstimatorParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.atlas.ros.AtlasPPSTimestampOffsetProvider;
import us.ihmc.atlas.sensors.AtlasCollisionBoxProvider;
import us.ihmc.atlas.sensors.AtlasSensorSuiteManager;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.handControl.HandCommandManager;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.avatar.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepPlanningParameterization;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.AtlasFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotiq.control.RobotiqHandCommandManager;
import us.ihmc.robotiq.model.RobotiqHandModel;
import us.ihmc.robotiq.simulatedHand.SimulatedRobotiqHandsController;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public class AtlasRobotModel implements DRCRobotModel, SDFDescriptionMutator
{
   public final static boolean SCALE_ATLAS = false;
   private final static double DESIRED_ATLAS_HEIGHT = 0.66;
   private final static double DESIRED_ATLAS_WEIGHT = 15;


   private final double HARDSTOP_RESTRICTION_ANGLE = Math.toRadians(5.0);

   private final AtlasRobotVersion selectedVersion;
   private final DRCRobotModel.RobotTarget target;

   private static final long ESTIMATOR_DT_IN_NS = 1000000;
   private static final double ESTIMATOR_DT = Conversions.nanoSecondstoSeconds(ESTIMATOR_DT_IN_NS);
   private static final double CONTROL_DT = 0.004;    // 0.006;

   private static final double ATLAS_ONBOARD_SAMPLINGFREQ = 1000.0;
   public static final double ATLAS_ONBOARD_DT = 1.0 / ATLAS_ONBOARD_SAMPLINGFREQ;

   public static final boolean BATTERY_MASS_SIMULATOR_IN_ROBOT = false;

   private final JaxbSDFLoader loader;

   private final AtlasPhysicalProperties atlasPhysicalProperties;
   private final AtlasJointMap jointMap;
   private final AtlasSensorInformation sensorInformation;
   private final AtlasArmControllerParameters armControllerParameters;
   private final AtlasCapturePointPlannerParameters capturePointPlannerParameters;
   private final AtlasICPOptimizationParameters icpOptimizationParameters;
   private final AtlasWalkingControllerParameters walkingControllerParameters;
   private final AtlasStateEstimatorParameters stateEstimatorParameters;
   private final AtlasDefaultArmConfigurations defaultArmConfigurations;
   private final AtlasFootstepSnappingParameters snappingParameters;

   private boolean enableJointDamping = true;

   private final RobotDescription robotDescription;

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, DRCRobotModel.RobotTarget target, boolean headless)
   {
      this(atlasVersion, target, headless, null);
   }

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, DRCRobotModel.RobotTarget target, boolean headless, FootContactPoints simulationContactPoints)
   {
      if(SCALE_ATLAS)
      {
         atlasPhysicalProperties  = new AtlasPhysicalProperties(DESIRED_ATLAS_HEIGHT, DESIRED_ATLAS_WEIGHT);
      }
      else
      {
         atlasPhysicalProperties = new AtlasPhysicalProperties();
      }

      selectedVersion = atlasVersion;
      jointMap = new AtlasJointMap(selectedVersion, atlasPhysicalProperties, simulationContactPoints);
      this.target = target;

      this.loader = DRCRobotSDFLoader.loadDRCRobot(selectedVersion.getResourceDirectories(), selectedVersion.getSdfFileAsStream(), this);

      sensorInformation = new AtlasSensorInformation(atlasVersion, target);

      for (String forceSensorNames : sensorInformation.getForceSensorNames())
      {
         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, new RigidBodyTransform());
      }

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;
      capturePointPlannerParameters = new AtlasCapturePointPlannerParameters(atlasPhysicalProperties);
      icpOptimizationParameters = new AtlasICPOptimizationParameters(runningOnRealRobot);
      armControllerParameters = new AtlasArmControllerParameters(runningOnRealRobot, jointMap);
      walkingControllerParameters = new AtlasWalkingControllerParameters(target, jointMap);
      stateEstimatorParameters = new AtlasStateEstimatorParameters(jointMap, sensorInformation, runningOnRealRobot, getEstimatorDT());
      defaultArmConfigurations = new AtlasDefaultArmConfigurations();
      snappingParameters = new AtlasFootstepSnappingParameters();

      robotDescription = createRobotDescription();
   }

   private RobotDescription createRobotDescription()
   {
      boolean useCollisionMeshes = true;

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
      return atlasPhysicalProperties;
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
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new AtlasSimInitialSetup(groundHeight, initialYaw);
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

//   public void addMoreFootContactPointsSimOnly(int nContactPointsX, int nContactPointsY, boolean edgePointsOnly)
//   {
//      jointMap.getContactPointParameters().addMoreFootContactPointsSimOnly(nContactPointsX, nContactPointsY, edgePointsOnly);
//   }

   public void setJointDamping(FloatingRootJointRobot simulatedRobot)
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
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      RobotDescription robotDescription = loader.createRobotDescription(getJointMap());
      FullHumanoidRobotModel fullRobotModel = new FullHumanoidRobotModelFromDescription(robotDescription, getJointMap(), sensorInformation.getSensorFramesToTrack());
      for (RobotSide robotSide : RobotSide.values())
      {
         ArmJointName[] armJointNames = new ArmJointName[] {ArmJointName.FIRST_WRIST_PITCH, ArmJointName.WRIST_ROLL, ArmJointName.SECOND_WRIST_PITCH};

         for (ArmJointName armJointName : armJointNames)
         {
            OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
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
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
   {
      boolean enableTorqueVelocityLimits = false;
      HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot = new HumanoidFloatingRootJointRobot(robotDescription, jointMap, getEnableJointDamping(), enableTorqueVelocityLimits);
      return humanoidFloatingRootJointRobot;
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

   private GeneralizedSDFRobotModel getGeneralizedRobotModel()
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

      if ((target == DRCRobotModel.RobotTarget.SCS) && AtlasSensorInformation.SEND_ROBOT_DATA_TO_ROS)
      {
         return new SimulationRosClockPPSTimestampOffsetProvider();
      }

      return new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return new AtlasSensorSuiteManager(this, getCollisionBoxProvider(), getPPSTimestampOffsetProvider(), sensorInformation, getJointMap(), getPhysicalProperties(),
                                         target);
   }

   @Override
   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public ICPOptimizationParameters getICPOptimizationParameters()
   {
      return icpOptimizationParameters;
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
   public MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
           HumanoidGlobalDataProducer globalDataProducer, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
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
   public String getSimpleRobotName()
   {
      if (selectedVersion != AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
         return "Atlas";
      else
         return "AtlasNoForearms"; // This String is notably used in the UI to figure out if Atlas has the forearms on.
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
         List<SDFVisual> visuals = linkHolder.getVisuals();
         if(visuals != null)
         {
            for (SDFVisual sdfVisual : visuals)
            {
               SDFGeometry geometry = sdfVisual.getGeometry();
               if(geometry != null)
               {
                  SDFGeometry.Mesh mesh = geometry.getMesh();
                  if(mesh != null)
                  {
                     String meshUri = mesh.getUri();
                     if(meshUri.contains("meshes_unplugged"))
                     {
                        String replacedURI = meshUri.replace(".dae", ".obj");
                        mesh.setUri(replacedURI);
                     }
                  }
               }

            }
         }

         switch(linkHolder.getName())
         {
         case "pelvis":
            addAdditionalPelvisImuInImuFrame(linkHolder);
            break;
         case "utorso":
            addCustomCrashProtectionVisual(linkHolder);

            if (BATTERY_MASS_SIMULATOR_IN_ROBOT)
            {
               modifyLinkInertialPose(linkHolder, "-0.043 0.00229456 0.316809 0 -0 0");
               modifyLinkMass(linkHolder, 84.609);
            }
            else
            {
               modifyLinkInertialPose(linkHolder, "0.017261 0.0032352 0.3483 0 0 0");
               modifyLinkMass(linkHolder, 60.009);
               double ixx = 1.5;
               double ixy = 0.0;
               double ixz = 0.1;
               double iyy = 1.5;
               double iyz = 0.0;
               double izz = 0.5;
               modifyLinkInertia(linkHolder, new Matrix3D(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz));
            }

            addChestIMU(linkHolder);
            break;
         case "l_lfarm":
         case "r_lfarm":
            modifyLinkMass(linkHolder, 1.6);
            break;
         case "l_hand":
            modifyLeftHand(linkHolder);
            break;
         case "r_hand":
            modifyRightHand(linkHolder);
            break;
         case "l_finger_1_link_0":
            modifyLinkPose(linkHolder, "0.0903097 1.15155 0.38309 1.5708 0.000796327 1.5708");
            break;
         case "l_finger_1_link_1":
            modifyLinkPose(linkHolder, "0.0903097 1.15155 0.38309 1.5708 0.000796327 1.5708");
            break;
         case "l_finger_1_link_2":
            modifyLinkPose(linkHolder, "0.0903092 1.20153 0.35505 1.5708 0.520796 1.57081");
            break;
         case "l_finger_1_link_3":
            modifyLinkPose(linkHolder, "0.0903088 1.23536 0.335645 1.5708 0.000796327 1.5708");
            break;
         case "l_finger_2_link_0":
            modifyLinkPose(linkHolder, "0.16231 1.15155 0.38309 1.5708 0.000796327 1.5708");
            break;
         case "l_finger_2_link_1":
            modifyLinkPose(linkHolder, "0.16231 1.15155 0.38309 1.5708 0.000796327 1.5708");
            break;
         case "l_finger_2_link_2":
            modifyLinkPose(linkHolder, "0.162309 1.20153 0.35505 1.5708 0.520796 1.57081");
            break;
         case "l_finger_2_link_3":
            modifyLinkPose(linkHolder, "0.162309 1.23536 0.335645 1.5708 0.000796327 1.5708");
            break;
         case "l_finger_middle_link_0":
            modifyLinkPose(linkHolder, "0.12631 1.15155 0.47409 -1.57079 -0.000796327 1.5708");
            break;
         case "l_finger_middle_link_1":
            modifyLinkPose(linkHolder, "0.12631 1.15155 0.47409 -1.57079 -0.000796327 1.5708");
            break;
         case "l_finger_middle_link_2":
            modifyLinkPose(linkHolder, "0.12631 1.20153 0.50213 -1.57079 -0.520796 1.57079");
            break;
         case "l_finger_middle_link_3":
            modifyLinkPose(linkHolder, "0.126311 1.23536 0.521535 -1.57079 -0.000796327 1.5708");
            break;
         case "r_finger_1_link_0":
            modifyLinkPose(linkHolder, "0.16231 -1.15155 0.38309 1.57079 0.000796327 -1.57079");
            break;
         case "r_finger_1_link_1":
            modifyLinkPose(linkHolder, "0.16231 -1.15155 0.38309 1.57079 0.000796327 -1.57079");
            break;
         case "r_finger_1_link_2":
            modifyLinkPose(linkHolder, "0.16231 -1.20153 0.35505 1.57079 0.520796 -1.57079");
            break;
         case "r_finger_1_link_3":
            modifyLinkPose(linkHolder, "0.16231 -1.23536 0.335645 1.57079 0.000796327 -1.57079");
            break;
         case "r_finger_2_link_0":
            modifyLinkPose(linkHolder, "0.0903097 -1.15155 0.38309 1.57079 0.000796327 -1.57079");
            break;
         case "r_finger_2_link_1":
            modifyLinkPose(linkHolder, "0.0903097 -1.15155 0.38309 1.57079 0.000796327 -1.57079");
            break;
         case "r_finger_2_link_2":
            modifyLinkPose(linkHolder, "0.0903099 -1.20153 0.35505 1.57079 0.520796 -1.57079");
            break;
         case "r_finger_2_link_3":
            modifyLinkPose(linkHolder, "0.09031 -1.23536 0.335645 1.57079 0.000796327 -1.57079");
            break;
         case "r_finger_middle_link_0":
            modifyLinkPose(linkHolder, "0.12631 -1.15155 0.47409 -1.5708 -0.000796327 -1.5708");
            break;
         case "r_finger_middle_link_1":
            modifyLinkPose(linkHolder, "0.12631 -1.15155 0.47409 -1.5708 -0.000796327 -1.5708");
            break;
         case "r_finger_middle_link_2":
            modifyLinkPose(linkHolder, "0.12631 -1.20153 0.50213 -1.5708 -0.520796 -1.57079");
            break;
         case "r_finger_middle_link_3":
            modifyLinkPose(linkHolder, "0.126311 -1.23536 0.521535 -1.5708 -0.000796327 -1.5708");
            break;
         case "hokuyo_link":
            modifyHokuyoInertia(linkHolder);
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
         if(sensor.getType().equals("imu") && sensor.getName().equals("imu_sensor"))
         {
            sensor.setName("imu_sensor_at_pelvis_frame");
         }

         if(sensor.getType().equals("gpu_ray") && sensor.getName().equals("head_hokuyo_sensor"))
         {
            sensor.getRay().getScan().getHorizontal().setSamples("720");
            sensor.getRay().getScan().getHorizontal().setMinAngle("-1.5708");
            sensor.getRay().getScan().getHorizontal().setMaxAngle("1.5708");
         }
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
      if (this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   private void addAdditionalPelvisImuInImuFrame(SDFLinkHolder pelvis)
   {
      SDFSensor sdfImu = new SDFSensor();
      sdfImu.setName("imu_sensor_at_imu_frame");
      sdfImu.setType("imu");
      sdfImu.setPose("-0.0905 -0.000004 -0.0125 0.0 3.14159265359 -0.78539816339");

      SDFSensor.IMU imu = new SDFSensor.IMU();

      SDFSensor.IMU.IMUNoise imuNoise = new SDFSensor.IMU.IMUNoise();
      imuNoise.setType("gaussian");

      SDFSensor.IMU.IMUNoise.NoiseParameters rateNoise = new SDFSensor.IMU.IMUNoise.NoiseParameters();
      rateNoise.setMean("0");
      rateNoise.setStddev("0.0002");
      rateNoise.setBias_mean("7.5e-06");
      rateNoise.setBias_stddev("8e-07");

      SDFSensor.IMU.IMUNoise.NoiseParameters accelNoise = new SDFSensor.IMU.IMUNoise.NoiseParameters();
      accelNoise.setMean("0");
      accelNoise.setStddev("0.017");
      accelNoise.setBias_mean("0.1");
      accelNoise.setBias_stddev("0.001");

      imuNoise.setRate(rateNoise);
      imuNoise.setAccel(accelNoise);

      imu.setNoise(imuNoise);
      sdfImu.setImu(imu);

      pelvis.getSensors().add(sdfImu);
   }

   private void addChestIMU(SDFLinkHolder chestLink)
   {
      SDFSensor chestIMU = new SDFSensor();
      chestIMU.setName("imu_sensor_chest");
      chestIMU.setType("imu");

      // Position only approximate. If we start using the acceleration measurements this will have to be fixed.
      String piHalf = String.valueOf(Math.PI/2.0);
      String negativePiHalf = String.valueOf(-Math.PI/2.0);
      chestIMU.setPose("-0.15 0.0 0.3 " + piHalf + " 0.0 " + negativePiHalf);

      SDFSensor.IMU imu = new SDFSensor.IMU();
      chestIMU.setImu(imu);

      chestLink.getSensors().add(chestIMU);
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

   private void addCustomCrashProtectionVisual(SDFLinkHolder linkHolder)
   {
      List<SDFVisual> visuals = linkHolder.getVisuals();

      SDFVisual crashProtectionVisual = new SDFVisual();
      crashProtectionVisual.setName("utorso_crash_visual");

      SDFVisual.SDFMaterial crashProtectionVisualMaterial = new SDFVisual.SDFMaterial();
      crashProtectionVisualMaterial.setLighting("1");
      crashProtectionVisualMaterial.setAmbient("0.75686275 0 0.75686275 1");
      crashProtectionVisualMaterial.setDiffuse(".7 0 .7 1");
      crashProtectionVisualMaterial.setSpecular("1 0 1 1");
      crashProtectionVisualMaterial.setEmissive("0 0 1 1");

      SDFGeometry crashProtectionGeometry = new SDFGeometry();
      SDFGeometry.Mesh crashProtectionGeometryMesh = new SDFGeometry.Mesh();
      crashProtectionGeometryMesh.setScale("1 1 1");
      crashProtectionGeometryMesh.setUri("model://atlas_description/meshes_unplugged/ATLAS_UNPLUGGED_UPPER_BODY_CRASH_PROTECTION_NO_SHOULDER.stl");

      crashProtectionGeometry.setMesh(crashProtectionGeometryMesh);

      crashProtectionVisual.setMaterial(crashProtectionVisualMaterial);
      crashProtectionVisual.setPose("0 0 0 0 -0 0");
      crashProtectionVisual.setGeometry(crashProtectionGeometry);

      visuals.add(crashProtectionVisual);
   }

   private void modifyLeftHand(SDFLinkHolder linkHolder)
   {
      modifyLinkMass(linkHolder, 2.7);
      modifyLinkInertialPose(linkHolder, "-0.0 0.04 0.0 0 -0 0");
      List<SDFVisual> visuals = linkHolder.getVisuals();
      if(visuals != null)
      {
         for (SDFVisual sdfVisual : visuals)
         {
            if(sdfVisual.getName().equals("l_hand_visual"))
            {
               sdfVisual.setPose("-0.00179 0.126 0 0 -1.57079 0");
            }
         }
      }
   }

   private void modifyRightHand(SDFLinkHolder linkHolder)
   {
      modifyLinkMass(linkHolder, 2.7);
      modifyLinkInertialPose(linkHolder, "-0.0 -0.04 0.0 0 -0 0");
      List<SDFVisual> visuals = linkHolder.getVisuals();

      if(visuals != null)
      {
         for (SDFVisual sdfVisual : visuals)
         {
            if(sdfVisual.getName().equals("r_hand_visual"))
            {
               sdfVisual.setPose("-0.00179 -0.126 0 3.14159 -1.57079 0");

               sdfVisual.getGeometry().getMesh().setUri("model://robotiq_hand_description/meshes/s-model_articulated/visual/palmRight.STL");
            }
         }

         addCheckerboardToRightHand(visuals);
      }

   }

   private void addCheckerboardToRightHand(List<SDFVisual> visuals)
   {
      SDFVisual chessboardVisual = new SDFVisual();
      chessboardVisual.setName("r_hand_chessboard");
      chessboardVisual.setPose("0.065 -0.198 0.04 0 1.57 0");

      SDFGeometry chessboardVisualGeometry = new SDFGeometry();
      SDFGeometry.Mesh chessboardVisualGeometryMesh = new SDFGeometry.Mesh();

      chessboardVisualGeometryMesh.setScale(".75 .75 .01");
      chessboardVisualGeometryMesh.setUri("model://ihmc/calibration_cube.dae");

      chessboardVisualGeometry.setMesh(chessboardVisualGeometryMesh);

      chessboardVisual.setGeometry(chessboardVisualGeometry);

      visuals.add(chessboardVisual);
   }

   private void modifyLinkMass(SDFLinkHolder linkHolder, double mass)
   {
      linkHolder.setMass(mass);
   }

   private void modifyLinkPose(SDFLinkHolder linkHolder, String pose)
   {
      linkHolder.getTransformFromModelReferenceFrame().set(ModelFileLoaderConversionsHelper.poseToTransform(pose));
   }

   private void modifyLinkInertialPose(SDFLinkHolder linkHolder, String pose)
   {
      linkHolder.setInertialFrameWithRespectToLinkFrame(ModelFileLoaderConversionsHelper.poseToTransform(pose));
   }

   private void modifyLinkInertia(SDFLinkHolder linkHolder, Matrix3D inertia)
   {
      linkHolder.setInertia(inertia);
   }

}
