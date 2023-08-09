package us.ihmc.valkyrieRosControl.upperBody;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

public class ValkyrieUpperBodySimulation
{
   private static final int bufferSize = 16000;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Random random = new Random(32890);
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.UPPER_BODY);
   private final Robot simulationRobot;
   private final SimControllerInput controllerInput;

   private final RigidBodyBasics rootBody;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final HumanoidJointNameMap jointNameMap = robotModel.getJointMap();
   private final Map<String, OneDoFJointBasics> jointMap = new HashMap<>();

   private final SimulationConstructionSet2 simulation;
   private final SimulationSessionControls sessionControls;

   private final AtomicBoolean initializeRequestAtomic = new AtomicBoolean(true);
   private final YoBoolean initializeRequestYoVariable = new YoBoolean("requestInitialize", registry);
   private RealtimeROS2Node ros2Node;
   private RobotConfigurationDataPublisher robotConfigurationDataPublisher;

   private final YoBoolean submitRandomConfig = new YoBoolean("submitRandomConfig", registry);
   private final YoBoolean submitChestCommand = new YoBoolean("submitChestCommand", registry);
   private final YoDouble chestPitch = new YoDouble("chestPitch", registry);
   private final YoDouble chestRoll = new YoDouble("chestRoll", registry);
   private final YoDouble chestYaw = new YoDouble("chestYaw", registry);

   private final YoBoolean submitNeckCommand = new YoBoolean("submitNeckCommand", registry);
   private final YoDouble neckPitch1 = new YoDouble("neckPitch1", registry);
   private final YoDouble neckYaw = new YoDouble("neckYaw", registry);
   private final YoDouble neckPitch2 = new YoDouble("neckPitch2", registry);

   private final SideDependentList<YoBoolean> submitArmCommand = new SideDependentList<>();
   private final SideDependentList<YoDouble> shoulderPitch = new SideDependentList<>();
   private final SideDependentList<YoDouble> shoulderRoll = new SideDependentList<>();
   private final SideDependentList<YoDouble> shoulderYaw = new SideDependentList<>();
   private final SideDependentList<YoDouble> elbowPitch = new SideDependentList<>();
   private final SideDependentList<YoDouble> forearmYaw = new SideDependentList<>();
   private final SideDependentList<YoDouble> wristRoll = new SideDependentList<>();
   private final SideDependentList<YoDouble> wristPitch = new SideDependentList<>();

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

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         jointMap.put(controlledOneDoFJoints[i].getName(), controlledOneDoFJoints[i]);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         submitArmCommand.put(robotSide, new YoBoolean("submit" + robotSide.getCamelCaseNameForMiddleOfExpression() + "ArmCommand", registry));
         shoulderPitch.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "ShoulderPitch", registry));
         shoulderRoll.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "ShoulderRoll", registry));
         shoulderYaw.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "ShoulderYaw", registry));
         elbowPitch.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "ElbowPitch", registry));
         forearmYaw.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "ForearmYaw", registry));
         wristRoll.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "WristRoll", registry));
         wristPitch.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "WristPitch", registry));

         shoulderPitch.get(robotSide).set(0.1);
         shoulderRoll.get(robotSide).set(robotSide.negateIfRightSide(-1.0));
         shoulderYaw.get(robotSide).set(0.1);
         elbowPitch.get(robotSide).set(robotSide.negateIfRightSide(-1.3));
      }

      simulation = new SimulationConstructionSet2(robotModel.getSimpleRobotName() + "ValkyrieUpperBody");
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
               ChestTrajectoryMessage chestTrajectory = HumanoidMessageTools.createChestTrajectoryMessage(3.0, new Quaternion(chestYaw.getDoubleValue(), chestPitch.getDoubleValue(), chestRoll.getDoubleValue()), ReferenceFrame.getWorldFrame());
               commandInputManager.submitMessage(chestTrajectory);
            }

            if (submitNeckCommand.getValue())
            {
               submitNeckCommand.set(false);
               NeckTrajectoryMessage neckTrajectory = HumanoidMessageTools.createNeckTrajectoryMessage(3.0, new double[]{neckPitch1.getDoubleValue(), neckYaw.getDoubleValue(), neckPitch2.getDoubleValue()});
               commandInputManager.submitMessage(neckTrajectory);
            }

            for (RobotSide robotSide : RobotSide.values)
            {
               if (submitArmCommand.get(robotSide).getValue())
               {
                  submitArmCommand.get(robotSide).set(false);

                  double[] desiredJointPositions = {shoulderPitch.get(robotSide).getValue(),
                                                    shoulderRoll.get(robotSide).getValue(),
                                                    shoulderYaw.get(robotSide).getValue(),
                                                    elbowPitch.get(robotSide).getValue(),
                                                    forearmYaw.get(robotSide).getValue(),
                                                    wristRoll.get(robotSide).getValue(),
                                                    wristPitch.get(robotSide).getValue()};
                  ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, 3.0, desiredJointPositions);
                  commandInputManager.submitMessage(armTrajectoryMessage);
               }
            }

            if (submitRandomConfig.getValue())
            {
               submitRandomConfig.set(false);

               double trajectoryTime = 3.0;

               List<String> neckJointNames = jointNameMap.getNeckJointNamesAsStrings();
               commandInputManager.submitMessage(HumanoidMessageTools.createNeckTrajectoryMessage(trajectoryTime, createRandomJointAngles(neckJointNames)));

               List<String> spineJointNames = jointNameMap.getSpineJointNamesAsStrings();
               commandInputManager.submitMessage(HumanoidMessageTools.createSpineTrajectoryMessage(trajectoryTime, createRandomJointAngles(spineJointNames)));

               for (RobotSide robotSide : RobotSide.values)
               {
                  List<String> armJointNames = jointNameMap.getArmJointNamesAsStrings(robotSide);
                  commandInputManager.submitMessage(HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, createRandomJointAngles(armJointNames)));
               }
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
      setupROSComms();

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

   private double[] createRandomJointAngles(List<String> jointNames)
   {
      double[] jointAngles = new double[jointNames.size()];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = getRandomJointAngle(jointNames.get(i));
      }
      return jointAngles;
   }

   private double getRandomJointAngle(String jointName)
   {
      OneDoFJointBasics joint = jointMap.get(jointName);
      double jointLimitLower = joint.getJointLimitLower();
      double jointLimitUpper = joint.getJointLimitUpper();
      double reductionFactor = 0.3;

      double jointLimitReducedLower = joint.getJointLimitLower() + reductionFactor * 0.5 * (jointLimitUpper - jointLimitLower);
      double jointLimitReducedUpper = joint.getJointLimitUpper() - reductionFactor * 0.5 * (jointLimitUpper - jointLimitLower);
      return EuclidCoreRandomTools.nextDouble(random, jointLimitReducedLower, jointLimitReducedUpper);
   }

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

   private void setupROSComms()
   {
      ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "valkyrie_upper_body_simulation");
      robotConfigurationDataPublisher = ValkyrieUpperBodyController.createRobotConfigurationDataPublisher(robotModel,
                                                                                                          commandInputManager,
                                                                                                          statusMessageOutputManager,
                                                                                                          ros2Node,
                                                                                                          controlledOneDoFJoints,
                                                                                                          System::currentTimeMillis,
                                                                                                          System::currentTimeMillis);

      ros2Node.spin();
   }

   public static void main(String[] args)
   {
      new ValkyrieUpperBodySimulation();
   }
}
