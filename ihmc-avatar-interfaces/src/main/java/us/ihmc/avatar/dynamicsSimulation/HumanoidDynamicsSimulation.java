package us.ihmc.avatar.dynamicsSimulation;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class HumanoidDynamicsSimulation
{
   private final RealtimeROS2Node realtimeROS2Node;
   private final SimulationConstructionSet2 simulationConstructionSet;
   private final SCS2AvatarSimulation avatarSimulation;

   public static HumanoidDynamicsSimulation createForManualTest(DRCRobotModel robotModel,
                                                                CommonAvatarEnvironmentInterface environment,
                                                                int recordTicksPerControllerTick,
                                                                int dataBufferSize)
   {
      return create(robotModel, environment, PubSubImplementation.FAST_RTPS, recordTicksPerControllerTick, dataBufferSize, false);
   }

   public static HumanoidDynamicsSimulation createForAutomatedTest(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment)
   {
      return create(robotModel, environment, PubSubImplementation.INTRAPROCESS, 1, 1024, false);
   }

   public static HumanoidDynamicsSimulation create(DRCRobotModel robotModel,
                                                   CommonAvatarEnvironmentInterface environment,
                                                   PubSubImplementation pubSubImplementation,
                                                   int recordTicksPerControllerTick,
                                                   int dataBufferSize,
                                                   boolean logToFile)
   {
      return create(robotModel, environment, 0.0, 0.0, 0.0, 0.0, pubSubImplementation, recordTicksPerControllerTick, dataBufferSize, logToFile);
   }

   public static HumanoidDynamicsSimulation create(DRCRobotModel robotModel,
                                                   CommonAvatarEnvironmentInterface environment,
                                                   double groundHeight,
                                                   double startingX,
                                                   double startingY,
                                                   double startingYaw,
                                                   PubSubImplementation pubSubImplementation,
                                                   int recordTicksPerControllerTick,
                                                   int dataBufferSize,
                                                   boolean logToFile)
   {
      SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(environment, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setTimePerRecordTick(robotModel.getControllerDT() * recordTicksPerControllerTick);
      scsInitialSetup.setUsePerfectSensors(true);
      scsInitialSetup.setSimulationDataBufferSize(dataBufferSize);
      scsInitialSetup.setUseExperimentalPhysicsEngine(false);

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      CoPTrajectoryParameters copTrajectoryParameters = robotModel.getCoPTrajectoryParameters();
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
      {
         contactableBodiesFactory.addAdditionalContactPoint(contactPointParameters.getAdditionalContactRigidBodyNames().get(i),
                                                            contactPointParameters.getAdditionalContactNames().get(i),
                                                            contactPointParameters.getAdditionalContactTransforms().get(i));
      }

      RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, "humanoid_simulation_controller");

      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                                                                    robotModel.getSensorInformation().getFeetForceSensorNames(),
                                                                                                    robotModel.getSensorInformation()
                                                                                                              .getWristForceSensorNames(),
                                                                                                    robotModel.getHighLevelControllerParameters(),
                                                                                                    robotModel.getWalkingControllerParameters(),
                                                                                                    robotModel.getPushRecoveryControllerParameters(),
                                                                                                    copTrajectoryParameters,
                                                                                                    robotModel.getSplitFractionCalculatorParameters());
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();
      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);
      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, DO_NOTHING_BEHAVIOR);
      controllerFactory.addControllerFailureTransition(WALKING, DO_NOTHING_BEHAVIOR);
      controllerFactory.setInitialState(HighLevelControllerName.WALKING);
      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeROS2Node);

      SCS2AvatarSimulationFactory avatarSimulationFactory = new SCS2AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setHighLevelHumanoidControllerFactory(controllerFactory);
      avatarSimulationFactory.setCommonAvatarEnvrionmentInterface(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotModel.getDefaultRobotInitialSetup(groundHeight, startingYaw, startingX, startingY));
      avatarSimulationFactory.setInitializeEstimatorToActual(true);
      avatarSimulationFactory.setSimulationDataRecordTimePeriod(robotModel.getControllerDT() * recordTicksPerControllerTick);
      avatarSimulationFactory.setSimulationDataBufferSize(dataBufferSize);
      avatarSimulationFactory.setUseImpulseBasedPhysicsEngine(false);
      avatarSimulationFactory.setShowGUI(simulationTestingParameters.getCreateGUI());
      avatarSimulationFactory.setUsePerfectSensors(simulationTestingParameters.getUsePefectSensors());
      avatarSimulationFactory.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      avatarSimulationFactory.setRealtimeROS2Node(realtimeROS2Node);
      avatarSimulationFactory.setCreateYoVariableServer(false);
      avatarSimulationFactory.setLogToFile(logToFile);

      SCS2AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();
      avatarSimulation.setSystemExitOnDestroy(false);
      SimulationConstructionSet2 scs = avatarSimulation.getSimulationConstructionSet();

      avatarSimulation.start();

      // TODO set up some useful graphs

      return new HumanoidDynamicsSimulation(realtimeROS2Node, avatarSimulation);
   }

   private HumanoidDynamicsSimulation(RealtimeROS2Node realtimeROS2Node, SCS2AvatarSimulation avatarSimulation)
   {
      this.realtimeROS2Node = realtimeROS2Node;
      this.avatarSimulation = avatarSimulation;
      this.simulationConstructionSet = avatarSimulation.getSimulationConstructionSet();
   }

   public void destroy()
   {
      LogTools.info("Shutting down");
      avatarSimulation.destroy();
      realtimeROS2Node.destroy();
   }

   public RealtimeROS2Node getRealtimeROS2Node()
   {
      return realtimeROS2Node;
   }

   public SimulationConstructionSet2 getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

   public SCS2AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }

   public void simulate()
   {
      simulationConstructionSet.simulate();
   }
}
