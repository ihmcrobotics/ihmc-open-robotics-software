package us.ihmc.valkyrie.simulation;

import java.io.File;
import java.nio.file.Path;
import java.util.EnumMap;

import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;

import controller_msgs.msg.dds.GroundPlaneMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.MultiContactScriptReader;
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
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.valkyrie.ValkyrieInitialSetupFactories;
import us.ihmc.valkyrie.ValkyrieMutableInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyriePlanarRegionPositionControlSimulation
{
   public static void main(String[] args)
   {
      new ValkyriePlanarRegionPositionControlSimulation(args.length > 0 && args[0].equals("headless"));
   }

   private static final boolean REMOVE_JOINT_LIMITS = true;

   private final double dt = 8.0e-4;
   private final ValkyrieRobotModel robotModel;

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ground_plane_node");
   private final IHMCROS2Publisher<GroundPlaneMessage> groundPlanePublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                        ROS2Tools.IHMC_ROOT.withTypeName(GroundPlaneMessage.class));
   private final GroundPlaneMessage groundPlaneMessage = new GroundPlaneMessage();

   public enum InitialPose
   {
      STANDING, DOWN_ON_ALL_FOURS, FROM_SCRIPT
   }

   public static InitialPose initialPose = InitialPose.FROM_SCRIPT;

   public ValkyriePlanarRegionPositionControlSimulation(boolean headless)
   {
      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.createFlatGround();
      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.01, false);

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

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setUsePerfectSensors(true);

      if (headless)
      {
         simulationStarter.getGuiInitialSetup().setGraphics3DAdapter(new NullGraphics3DAdapter());
         simulationStarter.getGuiInitialSetup().setShowWindow(false);
      }

      if (initialPose == InitialPose.FROM_SCRIPT)
      {
         MultiContactScriptReader scriptReader = new MultiContactScriptReader();
         Path currentDirectory = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software")
                                                   .resolve("valkyrie/src/main/resources/multiContact/scripts").toAbsolutePath().normalize();
         System.out.println(currentDirectory);
         JFileChooser fileChooser = new JFileChooser(currentDirectory.toFile());
         fileChooser.setFileFilter(new FileNameExtensionFilter("JSON log", "json"));

         int chooserState = fileChooser.showOpenDialog(null);
         if (chooserState != JFileChooser.APPROVE_OPTION)
            return;

         File selectedFile = fileChooser.getSelectedFile();
         if (!scriptReader.loadScript(selectedFile))
            return;

         scriptReader.loadScript(selectedFile);
         KinematicsToolboxSnapshotDescription initialSnapshot = scriptReader.getAllItems().get(0);
         RobotConfigurationData robotConfigurationData = initialSnapshot.getControllerConfiguration();

         ValkyrieMutableInitialSetup initialSetup = new ValkyrieMutableInitialSetup(jointMap);

         // set root joint
         initialSetup.getRootJointPosition().set(robotConfigurationData.getRootTranslation());
         initialSetup.getRootJointOrientation().set(robotConfigurationData.getRootOrientation());
         initialSetup.getRootJointOrientation().appendPitchRotation(Math.toRadians(1.0));

         // set joint positions
         OneDoFJointBasics[] oneDoFJoints = robotModel.createFullRobotModel().getControllableOneDoFJoints();
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            if (!oneDoFJoints[i].getName().contains("hokuyo"))
               initialSetup.setJoint(oneDoFJoints[i].getName(), (double) robotConfigurationData.getJointAngles().get(i));
         }

         simulationStarter.setRobotInitialSetup(initialSetup);
      }
      else if (initialPose == InitialPose.DOWN_ON_ALL_FOURS)
      {
         ValkyrieMutableInitialSetup initialSetup = ValkyrieInitialSetupFactories.newAllFoursBellyDown(jointMap);
         simulationStarter.setRobotInitialSetup(initialSetup);
      }
      simulationStarter.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      simulationStarter.getSCSInitialSetup().setRecordFrequency(50);
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
                                                         controllerToolbox,
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
      simulationStarter.createSimulation(networkProcessorParameters, true, true);
      simulationStarter.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getRequestedControlStateEnum().set(null);
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().setFastSimulate(true, 10);
      simulationStarter.getSimulationConstructionSet().skipLoadingDefaultConfiguration();

      // adds a new row each time. the method above looks like it's supposed to prevent that but it doesn't seem to work, at least on windows.
      //      simulationStarter.getSimulationConstructionSet().setupGraph("t");

      KinematicsToolboxModule kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false, 10, false, DomainFactory.PubSubImplementation.FAST_RTPS);
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().addYoRegistry(kinematicsToolboxModule.getRegistry());
   }
}
