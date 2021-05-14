package us.ihmc.valkyrie.externalForceEstimation;

import java.util.concurrent.atomic.AtomicReference;

import javax.swing.JButton;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieExternalForceEstimationSimulation
{
   private static final double simDT = 2e-4; // normally 6.6e-4. (controlDT=4e-3)
   private static final Vector3D initialForce = new Vector3D(0.0, 0.0, 0.0);

   public ValkyrieExternalForceEstimationSimulation()
   {
      ValkyrieRobotVersion version = ValkyrieRobotVersion.FINGERLESS;
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, version);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new FlatGroundProfile());
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.createSimulation(null, false, false);
      simulationStarter.getSCSInitialSetup().setUsePerfectSensors(true);

      double controllerDT = robotModel.getControllerDT();
      RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS, "valkyrie_wrench_estimation_sim");

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoRegistry scsRootRegistry = simulationStarter.getAvatarSimulation().getSimulationConstructionSet().getRootRegistry();

      HumanoidFloatingRootJointRobot scsRobot = simulationStarter.getSDFRobot();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      // ----- Root Joint ----- //
      //      Joint scsEndEffector = scsRobot.getRootJoint();
      //      RigidBodyBasics endEffector = fullRobotModel.getRootBody();

      // ----- 1DOF Joints ----- //
//            String endEffectorName = "torsoRoll"; // Chest
//            String endEffectorName = "rightShoulderRoll"; // Shoulder
      String endEffectorName = "rightElbowPitch"; // Elbow
      //      String endEffectorName = "rightForearmYaw"; // Forearm
      //      String endEffectorName = "rightWristRoll"; // Wrist roll
      //      String endEffectorName = "rightWristPitch"; // Wrist pitch
      Joint scsEndEffector = scsRobot.getJoint(endEffectorName);
      RigidBodyBasics endEffector = fullRobotModel.getOneDoFJointByName(endEffectorName).getSuccessor();

      Vector3D externalForcePointOffset = new Vector3D(0.0, -0.32, 0.0);
//      Vector3D externalForcePointOffset = new Vector3D(0.3, 0.0, 0.5);
//      Vector3D externalForcePointOffset = new Vector3D(0.0, -0.32, 0.5);

      ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp", externalForcePointOffset, scsRobot);
      scsEndEffector.addExternalForcePoint(externalForcePoint);
      externalForcePoint.setForce(initialForce);

      double forceGraphicScale = 0.05;
      AppearanceDefinition simulatedForceColor = YoAppearance.Red();
      AppearanceDefinition estimatedForceColor = YoAppearance.Green();
      YoFrameVector3D estimatedForce = new YoFrameVector3D("estimatedForce", ReferenceFrame.getWorldFrame(), scsRootRegistry);

      YoGraphicVector simulatedForceVector = new YoGraphicVector("simulatedForceVector",
                                                                 externalForcePoint.getYoPosition(),
                                                                 externalForcePoint.getYoForce(),
                                                                 forceGraphicScale,
                                                                 simulatedForceColor);

      YoGraphicVector estimatedForceVector = new YoGraphicVector("estimatedForceVector",
                                                                 externalForcePoint.getYoPosition(),
                                                                 estimatedForce,
                                                                 forceGraphicScale,
                                                                 estimatedForceColor);

      YoGraphicPosition simulatedForcePoint = new YoGraphicPosition("simulatedForcePoint", externalForcePoint.getYoPosition(), 0.025, simulatedForceColor);

      YoGraphicsList externalForcePointViz = new YoGraphicsList("simulatedExternalForce");
      externalForcePointViz.add(simulatedForceVector);
      externalForcePointViz.add(estimatedForceVector);
      externalForcePointViz.add(simulatedForcePoint);
      graphicsListRegistry.registerYoGraphicsList(externalForcePointViz);

      simulationStarter.getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().setDT(simDT, (int) (controllerDT / simDT));

      IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                                ToolboxStateMessage.class,
                                                                                                                ExternalForceEstimationToolboxModule.getInputTopic(
                                                                                                             robotModel.getSimpleRobotName()));

      IHMCRealtimeROS2Publisher<ExternalForceEstimationConfigurationMessage> configurationMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                                                                ExternalForceEstimationConfigurationMessage.class,
                                                                                                                                                ExternalForceEstimationToolboxModule
                                                                                                                                             .getInputTopic(
                                                                                                                                                   robotModel.getSimpleRobotName()));

      JButton wakeupButton = new JButton("Start estimation");
      wakeupButton.addActionListener(e ->
                                     {
                                        ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
                                        configurationMessage.setEstimatorGain(0.5);
                                        configurationMessage.getRigidBodyHashCodes().add(endEffector.hashCode());
                                        configurationMessage.getContactPointPositions().add().set(externalForcePointOffset);
                                        configurationMessage.setCalculateRootJointWrench(false);
                                        configurationMessagePublisher.publish(configurationMessage);

                                        ThreadTools.sleep(1);

                                        ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
                                        toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
                                        toolboxStateMessage.setRequestLogging(true);
                                        toolboxStatePublisher.publish(toolboxStateMessage);
                                     });
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().addButton(wakeupButton);

      JButton sleepButton = new JButton("Stop estimation");
      sleepButton.addActionListener(e ->
                                    {
                                       ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
                                       toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.SLEEP);
                                       toolboxStatePublisher.publish(toolboxStateMessage);
                                    });
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().addButton(sleepButton);

      new ExternalForceEstimationToolboxModule(robotModel, true, PubSubImplementation.FAST_RTPS);

      AtomicReference<ExternalForceEstimationOutputStatus> toolboxOutputStatus = new AtomicReference<>();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    ExternalForceEstimationOutputStatus.class,
                                                    ExternalForceEstimationToolboxModule.getOutputTopic(robotModel.getSimpleRobotName()),
                                           s -> toolboxOutputStatus.set(s.takeNextData()));

      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().addScript(t ->
                                                                                       {
                                                                                          if (toolboxOutputStatus.get() != null)
                                                                                             estimatedForce.set(toolboxOutputStatus.get()
                                                                                                                                   .getEstimatedExternalForces()
                                                                                                                                   .get(0));
                                                                                       });
      simulationStarter.getAvatarSimulation().start();
      simulationStarter.getAvatarSimulation().simulate();
      ros2Node.spin();
   }

   public static void main(String[] args)
   {
      new ValkyrieExternalForceEstimationSimulation();
   }
}
