package us.ihmc.valkyrie.simulation;

import controller_msgs.msg.dds.GroundPlaneMessage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.MultiContactScriptReader;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointspacePositionControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.valkyrie.ValkyrieInitialSetupFactories;
import us.ihmc.valkyrie.ValkyrieMutableInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.EnumMap;

public class ValkyriePlanarRegionPositionControlSimulation
{
   private final PlanarRegionsList regions = createWallAndTable();
   private final RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, getClass().getSimpleName());
   private final GroundPlaneMessage groundPlaneMessage = new GroundPlaneMessage();
   private final IHMCRealtimeROS2Publisher<GroundPlaneMessage> groundPlanePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.IHMC_ROOT.withTypeName(GroundPlaneMessage.class));
   private final ValkyrieRobotModel robotModel;

   public ValkyriePlanarRegionPositionControlSimulation(boolean headless)
   {
      PlanarRegionsList[] regionsAsArray = new PlanarRegionsList[regions.getNumberOfPlanarRegions()];
      for (int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         regionsAsArray[i] = new PlanarRegionsList(regions.getPlanarRegion(i));
      }

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("NadiaMultiContactEnvironment",
                                                                                                regionsAsArray,
                                                                                                new AppearanceDefinition[] {YoAppearance.Gray(), YoAppearance.BurlyWood(), YoAppearance.LightGrey()},
                                                                                                0.01,
                                                                                                false);
      groundPlaneMessage.getRegionNormal().set(Axis3D.Z);
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);

      SCS2AvatarSimulationFactory simulationFactory = new SCS2AvatarSimulationFactory();
      simulationFactory.setRobotModel(robotModel);
      simulationFactory.setUsePerfectSensors(true);
      simulationFactory.setRunMultiThreaded(true);
      simulationFactory.setInitializeEstimatorToActual(true);
      simulationFactory.setUseImpulseBasedPhysicsEngine(true);
      simulationFactory.setSimulationDataRecordTickPeriod(20);
      //      simulationFactory.setSimulationDataBufferSize(16000);
      simulationFactory.setRealtimeROS2Node(ros2Node);
      simulationFactory.setCommonAvatarEnvrionmentInterface(environment);

      ValkyrieMutableInitialSetup initialSetup = ValkyrieInitialSetupFactories.newStand(robotModel.getRobotDefinition(), robotModel.getJointMap());
      initialSetup.getRootJointPosition().subX(0.1);
      initialSetup.getRootJointPosition().subZ(0.01);
      simulationFactory.setRobotInitialSetup(initialSetup);

      HighLevelControllerName positionControlState = HighLevelControllerName.CUSTOM1;
      robotModel.setHighLevelControllerParameters(new ValkyrieSimulationPositionControlParameters(robotModel.getHighLevelControllerParameters(), robotModel.getJointMap(), positionControlState));

      HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory = simulationFactory.setDefaultHighLevelHumanoidControllerFactory();
      highLevelHumanoidControllerFactory.addCustomControlState(new HighLevelControllerStateFactory()
      {
         @Override
         public HighLevelControllerName getStateEnum()
         {
            return positionControlState;
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
                                                         controllerToolbox,
                                                         highLevelControllerParameters,
                                                         highLevelControllerOutput);
         }
      });

      highLevelHumanoidControllerFactory.addCustomStateTransition(new ControllerStateTransitionFactory<HighLevelControllerName>()
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

      SCS2AvatarSimulation avatarSimulation = simulationFactory.createAvatarSimulation();
//      KinematicsToolboxModule kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false, 20, false, DomainFactory.PubSubImplementation.FAST_RTPS);

      ContactParameters contactParameters = ContactParameters.defaultIneslasticContactParameters(true);
      contactParameters.setCoefficientOfFriction(1.0);
      contactParameters.setCoulombMomentFrictionRatio(0.8);
      ((ImpulseBasedPhysicsEngine) avatarSimulation.getSimulationConstructionSet().getPhysicsEngine()).setGlobalContactParameters(contactParameters);
      avatarSimulation.start();

   }

   public static void main(String[] args)
   {
      new ValkyriePlanarRegionPositionControlSimulation(false);
   }

   public static PlanarRegionsList createWallAndTable()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      // ground
      generator.addRectangle(5.0, 5.0);

      // wall
      generator.translate(0.5, -0.3, 0.6);
      generator.rotate(0.5 * Math.PI, Axis3D.Y);
      generator.addRectangle(1.2, 0.7);

      // table
      generator.identity();
      generator.translate(0.85, 0.3, 0.75);
      generator.addRectangle(0.35, 0.35);

      return generator.getPlanarRegionsList();
   }
}
