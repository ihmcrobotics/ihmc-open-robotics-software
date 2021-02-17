package us.ihmc.atlas;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import java.util.ArrayList;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class AtlasFlatGroundWalkingTrackSCS2
{
   private static boolean createYoVariableServer = System.getProperty("create.yovariable.server") != null
         && Boolean.parseBoolean(System.getProperty("create.yovariable.server"));

   private final RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS,
                                                                                      "flat_ground_walking_track_simulation");

   public AtlasFlatGroundWalkingTrackSCS2()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      FlatGroundEnvironment environment = new FlatGroundEnvironment();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(environment, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(robotModel.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      HeightMap heightMapForFootstepZ = null;
      if (cheatWithGroundHeightAtForFootstep)
      {
         heightMapForFootstepZ = scsInitialSetup.getHeightMap();
      }
      HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();
      HighLevelControllerStateFactory customControllerStateFactory = null;

      HighLevelHumanoidControllerFactory controllerFactory = createControllerFactory(robotModel,
                                                                                     realtimeROS2Node,
                                                                                     useVelocityAndHeadingScript,
                                                                                     heightMapForFootstepZ,
                                                                                     walkingScriptParameters,
                                                                                     customControllerStateFactory);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, initialYaw);

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setHighLevelHumanoidControllerFactory(controllerFactory);
      avatarSimulationFactory.setCommonAvatarEnvrionmentInterface(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSCSInitialSetup(scsInitialSetup);
      avatarSimulationFactory.setGuiInitialSetup(guiInitialSetup);
      avatarSimulationFactory.setRealtimeROS2Node(realtimeROS2Node);
      avatarSimulationFactory.setCreateYoVariableServer(createYoVariableServer);

      SCS2AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      avatarSimulation.start();
   }

   private static HighLevelHumanoidControllerFactory createControllerFactory(DRCRobotModel robotModel, RealtimeROS2Node realtimeROS2Node,
                                                                             boolean useVelocityAndHeadingScript, HeightMap heightMapForFootstepZ,
                                                                             HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters,
                                                                             HighLevelControllerStateFactory customControllerStateFactory)
   {
      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      CoPTrajectoryParameters copTrajectoryParameters = robotModel.getCoPTrajectoryParameters();
      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionalContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                                                                    feetForceSensorNames,
                                                                                                    feetContactSensorNames,
                                                                                                    wristForceSensorNames,
                                                                                                    highLevelControllerParameters,
                                                                                                    walkingControllerParameters,
                                                                                                    copTrajectoryParameters);
      setupHighLevelStates(controllerFactory, feetForceSensorNames, highLevelControllerParameters.getFallbackControllerState());
      controllerFactory.setHeadingAndVelocityEvaluationScriptParameters(walkingScriptParameters);
      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeROS2Node);
      if (customControllerStateFactory != null)
         controllerFactory.addCustomControlState(customControllerStateFactory);

      controllerFactory.createComponentBasedFootstepDataMessageGenerator(useVelocityAndHeadingScript, heightMapForFootstepZ);

      return controllerFactory;
   }

   public static void setupHighLevelStates(HighLevelHumanoidControllerFactory controllerFactory, SideDependentList<String> feetForceSensorNames,
                                           HighLevelControllerName fallbackControllerState)
   {
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();

      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);

      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, fallbackControllerState);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);

      controllerFactory.setInitialState(HighLevelControllerName.WALKING);
   }

   public static void main(String[] args)
   {
      new AtlasFlatGroundWalkingTrackSCS2();
   }
}
