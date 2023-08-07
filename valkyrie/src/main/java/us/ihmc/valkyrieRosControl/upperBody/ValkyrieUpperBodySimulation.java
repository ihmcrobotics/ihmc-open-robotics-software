package us.ihmc.valkyrieRosControl.upperBody;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.sensorProcessors.FloatingJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.concurrent.atomic.AtomicBoolean;

public class ValkyrieUpperBodySimulation
{
   private static final int bufferSize = 16000;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.UPPER_BODY);
   private final Robot simulationRobot;
   private final SimControllerInput controllerInput;

   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics[] controlledOneDoFJoints;

   private final SimulationConstructionSet2 simulation;
   private final SimulationSessionControls sessionControls;

   private final AtomicBoolean initializeRequestAtomic = new AtomicBoolean(true);
   private final YoBoolean initializeRequestYoVariable = new YoBoolean("requestInitialize", registry);
   private RealtimeROS2Node ros2Node;
   private RobotConfigurationDataPublisher robotConfigurationDataPublisher;

   private final YoBoolean submitChestCommand = new YoBoolean("submitChestCommand", registry);
   private final YoDouble chestPitch = new YoDouble("chestPitch", registry);
   private final YoDouble chestRoll = new YoDouble("chestRoll", registry);
   private final YoDouble chestYaw = new YoDouble("chestYaw", registry);

   private final int simulationTicksPerControlTicks;
   private final YoLong doControlCounter = new YoLong("doControlCounter", registry);

   private final ValkyrieUpperBodyManipulationState manipulationState;
   private final CommandInputManager commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
   private final StatusMessageOutputManager statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());
   private final YoDouble initializationTime = new YoDouble("initializationTime", registry);

   public ValkyrieUpperBodySimulation()
   {
      Pair<RigidBodyBasics, OneDoFJointBasics[]> upperBodySystem = ValkyrieUpperBodyController.createUpperBodySystem(robotModel);
      rootBody = upperBodySystem.getLeft();
      controlledOneDoFJoints = upperBodySystem.getRight();

      simulation = new SimulationConstructionSet2(robotModel.getSimpleRobotName() + "MultiContactSimulation");
      simulation.changeBufferSize(bufferSize);
      simulation.addRegistry(registry);

      simulation.addRobot(robotModel.getRobotDefinition());
      simulationRobot = simulation.getRobots().get(0);
      sessionControls = simulation.getSimulationSession().getSimulationSessionControls();


      simulationTicksPerControlTicks = (int) (robotModel.getControllerDT() / robotModel.getSimulateDT());
      simulation.setDT(robotModel.getSimulateDT());
      simulation.setBufferRecordTickPeriod(10);

      manipulationState = new ValkyrieUpperBodyManipulationState(commandInputManager,
                                                                 robotModel.getControllerDT(),
                                                                 robotModel.getJointMap(),
                                                                 robotModel.getHighLevelControllerParameters(),
                                                                 robotModel.getWalkingControllerParameters(),
                                                                 rootBody,
                                                                 controlledOneDoFJoints,
                                                                 simulation.getTime(),
                                                                 new YoGraphicsListRegistry());

      controllerInput = simulationRobot.getControllerManager().getControllerInput();
      simulationRobot.getRegistry().addChild(manipulationState.getYoRegistry());

      simulationRobot.addController(new Controller()
      {
         @Override
         public void doControl()
         {
            if (submitChestCommand.getValue())
            {
               submitChestCommand.set(false);
               ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
               SO3TrajectoryControllerCommand chestTrajectory = chestTrajectoryCommand.getSO3Trajectory();
               chestTrajectory.setTrajectoryFrame(ReferenceFrame.getWorldFrame());
               chestTrajectory.addTrajectoryPoint(3.0, new Quaternion(chestYaw.getDoubleValue(), chestPitch.getDoubleValue(), chestRoll.getDoubleValue()), new Vector3D());
               commandInputManager.submitCommand(chestTrajectoryCommand);
            }

            // read robot state
            for (int i = 0; i < controlledOneDoFJoints.length; i++)
            {
               controlledOneDoFJoints[i].setQ(((SimOneDoFJointBasics) controllerInput.getInput().findJoint(controlledOneDoFJoints[i].getName())).getQ());
               controlledOneDoFJoints[i].setQd(((SimOneDoFJointBasics) controllerInput.getInput().findJoint(controlledOneDoFJoints[i].getName())).getQd());
            }
            rootBody.updateFramesRecursively();

            boolean initializeRequested = initializeRequestAtomic.getAndSet(false) || initializeRequestYoVariable.getValue();
            initializeRequestYoVariable.set(false);

            if (initializeRequested)
            {
               manipulationState.onEntry();
               initializationTime.set(simulation.getTime().getValue());
            }

            boolean runController = doControlCounter.getValue() % simulationTicksPerControlTicks == 0;
            if (runController)
            {
               manipulationState.doAction(simulation.getTime().getDoubleValue() - initializationTime.getValue());
            }

            for (int i = 0; i < controlledOneDoFJoints.length; i++)
            {
               SimOneDoFJointBasics simJoint = simulationRobot.getOneDoFJoint(controlledOneDoFJoints[i].getName());
               simJoint.setTau(manipulationState.getOutputForLowLevelController().getDesiredJointTorque(controlledOneDoFJoints[i]));
            }

            if (runController)
            {
               if (robotConfigurationDataPublisher != null)
               {
                  robotConfigurationDataPublisher.write();
               }
            }

            doControlCounter.increment();
         }
      });

      new DefaultParameterReader().readParametersInRegistry(manipulationState.getYoRegistry());

      for (RobotSide robotSide : RobotSide.values)
      {
         simulationRobot.getOneDoFJoint(robotSide.getLowerCaseName() + "ShoulderRoll").setQ(robotSide.negateIfRightSide(-1.2));
         simulationRobot.getOneDoFJoint(robotSide.getLowerCaseName() + "ShoulderPitch").setQ(-0.2);
         simulationRobot.getOneDoFJoint(robotSide.getLowerCaseName() + "ElbowPitch").setQ(robotSide.negateIfRightSide(-1.5));
      }

      simulation.setCameraPosition(0.5, -9.0, 0.7);
      simulation.setCameraFocusPosition(0.5, 0.0, 0.7);
      simulation.start(true, false, false);
   }

//   abstract void initialize();
//   abstract void doControl();

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   public RobotDefinition getRobotDefinition()
   {
      return robotModel.getRobotDefinition();
   }

   public SimulationConstructionSet2 getSimulation()
   {
      return simulation;
   }

   public SimulationSessionControls getSessionControls()
   {
      return sessionControls;
   }

   public static void main(String[] args)
   {
      new ValkyrieUpperBodySimulation();
   }
}
