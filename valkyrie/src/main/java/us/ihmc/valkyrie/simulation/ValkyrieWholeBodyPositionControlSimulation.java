package us.ihmc.valkyrie.simulation;

import java.util.EnumMap;

import controller_msgs.msg.dds.GroundPlaneMessage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointspacePositionControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.valkyrie.ValkyrieInitialSetupFactories;
import us.ihmc.valkyrie.ValkyrieMutableInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieWholeBodyPositionControlSimulation
{
   public static void main(String[] args)
   {
      new ValkyrieWholeBodyPositionControlSimulation();
   }

   private static final boolean REMOVE_JOINT_LIMITS = true;

   private final double dt = 8.0e-4;
   private final ValkyrieRobotModel robotModel;

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "ground_plane_node");
   private final IHMCROS2Publisher<GroundPlaneMessage> groundPlanePublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                        ROS2Tools.IHMC_ROOT.withTypeName(GroundPlaneMessage.class));
   private final GroundPlaneMessage groundPlaneMessage = new GroundPlaneMessage();

   public ValkyrieWholeBodyPositionControlSimulation()
   {
      groundPlaneMessage.getRegionNormal().set(Axis3D.Z);

      robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);
      if (REMOVE_JOINT_LIMITS)
      {
         robotModel.setRobotDefinitionMutator(robotModel.getRobotDefinitionMutator().andThen(RobotDefinitionTools.jointLimitRemover()));
      }

      ValkyrieJointMap jointMap = robotModel.getJointMap();
      robotModel.setHighLevelControllerParameters(new ValkyrieSimulationPositionControlParameters(robotModel.getHighLevelControllerParameters(),
                                                                                                  jointMap,
                                                                                                  HighLevelControllerName.CUSTOM1));
      ValkyrieSimulationLowLevelControllerFactory simulationLowLevelControllerFactory = new ValkyrieSimulationLowLevelControllerFactory(jointMap, dt);
      robotModel.setSimulationLowLevelControllerFactory(simulationLowLevelControllerFactory);

      robotModel.setSimulateDT(dt);
      robotModel.setControllerDT(dt);
      robotModel.setEstimatorDT(dt);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new DefaultCommonAvatarEnvironment());
      simulationStarter.setUsePerfectSensors(true);
      ValkyrieMutableInitialSetup initialSetup = ValkyrieInitialSetupFactories.newAllFoursBellyDown(jointMap);
//      initialSetup.rootJointOrientation.prependYawRotation(1.35);
//      initialSetup.rootJointPosition.add(0.2, 0.1, 0);
      simulationStarter.setRobotInitialSetup(initialSetup);
      simulationStarter.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      simulationStarter.getSCSInitialSetup().setRecordFrequency(10);
      simulationStarter.registerHighLevelControllerState(new HighLevelControllerStateFactory()
      {
         @Override
         public HighLevelControllerName getStateEnum()
         {
            return HighLevelControllerName.CUSTOM1;
         }

         @Override
         public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
         {
            CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();
            StatusMessageOutputManager statusOutputManager = controllerFactoryHelper.getStatusMessageOutputManager();
            OneDoFJointBasics[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();
            HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();
            HighLevelControllerParameters highLevelControllerParameters = controllerFactoryHelper.getHighLevelControllerParameters();
            JointDesiredOutputListReadOnly highLevelControllerOutput = controllerFactoryHelper.getLowLevelControllerOutput();

            controllerToolbox.addUpdatable(time ->
            {
               Point3DBasics rootJointPosition = controllerToolbox.getFullRobotModel().getRootJoint().getJointPose().getPosition();
               groundPlaneMessage.getRegionOrigin().set(new Point2D(rootJointPosition));
               groundPlanePublisher.publish(groundPlaneMessage);
            });

            return new JointspacePositionControllerState(getStateEnum(),
                                                         commandInputManager,
                                                         statusOutputManager,
                                                         controlledJoints,
                                                         controllerToolbox.getYoTime(),
                                                         highLevelControllerParameters,
                                                         highLevelControllerOutput);
         }
      });

      simulationStarter.registerControllerStateTransition(new ControllerStateTransitionFactory<HighLevelControllerName>()
      {
         @Override
         public HighLevelControllerName getStateToAttachEnum()
         {
            return HighLevelControllerName.WALKING;
         }

         @Override
         public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> stateMap,
                                                                                    HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                                    YoRegistry parentRegistry)
         {
            return new StateTransition<>(HighLevelControllerName.CUSTOM1, t -> true);
         }
      });

      HumanoidNetworkProcessorParameters networkProcessorParameters = new HumanoidNetworkProcessorParameters();
      simulationStarter.createSimulation(networkProcessorParameters, true, false);
      simulationStarter.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getRequestedControlStateEnum().set(null);
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().setFastSimulate(true, 10);

//      KinematicsToolboxModule kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false, 10, false, PubSubImplementation.FAST_RTPS);
//      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().addYoRegistry(kinematicsToolboxModule.getRegistry());
   }
}
