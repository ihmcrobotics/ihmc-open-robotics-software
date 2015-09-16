package us.ihmc.atlas;

import com.jme3.math.Transform;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.SdfLoader.*;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.*;
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

import java.util.LinkedHashMap;
import java.util.List;

public class AtlasRobotModel implements DRCRobotModel, SDFDescriptionMutator
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
         this.loader = DRCRobotSDFLoader.loadDRCRobot(selectedVersion.getResourceDirectories(), selectedVersion.getSdfFileAsStream(), false, this);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[]
         {
         }, selectedVersion.getSdfFileAsStream(), headless, this);
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
         case "utorso":
            modifyLinkInertialPose(linkHolder, "-0.043 0.00229456 0.316809 0 -0 0");
            addCustomCrashProtectionVisual(linkHolder);
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

   private void modifyHokuyoInertia(SDFLinkHolder linkHolder)
   {
      linkHolder.getInertia().m00 = 0.000401606; // i_xx
      linkHolder.getInertia().m01 = 4.9927e-08; // i_xy
      linkHolder.getInertia().m02 = 1.0997e-05; // i_xz
      linkHolder.getInertia().m11 = 0.00208115; // i_yy
      linkHolder.getInertia().m12 = -9.8165e-09; // i_yz
      linkHolder.getInertia().m22 = 0.00178402; // i_zz
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
      linkHolder.getTransformFromModelReferenceFrame().set(SDFConversionsHelper.poseToTransform(pose));
   }

   private void modifyLinkInertialPose(SDFLinkHolder linkHolder, String pose)
   {
      linkHolder.setInertialFrameWithRespectToLinkFrame(SDFConversionsHelper.poseToTransform(pose));
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
      if(this.jointMap.getModelName().equals(model.getName()))
      {
         addAdditionalImuInImuFrame(model);
      }
   }

   private void addAdditionalImuInImuFrame(GeneralizedSDFRobotModel model)
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

      for (SDFLinkHolder sdfLinkHolder : model.getRootLinks())
      {
         if(sdfLinkHolder.getName().equals("pelvis"))
         {
            sdfLinkHolder.getSensors().add(sdfImu);
         }
      }
   }
}
