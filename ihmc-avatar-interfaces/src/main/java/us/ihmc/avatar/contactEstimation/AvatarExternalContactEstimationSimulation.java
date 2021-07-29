package us.ihmc.avatar.contactEstimation;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.concurrent.atomic.AtomicReference;

public abstract class AvatarExternalContactEstimationSimulation
{
   private enum Mode
   {
      ESTIMATE_FORCE,
      LOCALIZE_CONTACT
   }

   private static final Mode mode = Mode.LOCALIZE_CONTACT;

   public AvatarExternalContactEstimationSimulation(String jointName, Point3D offset)
   {
      DRCRobotModel robotModel = getRobotModel();
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new FlatGroundEnvironment());
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.getSCSInitialSetup().setUseExperimentalPhysicsEngine(false);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.createSimulation(null, false, false);
      simulationStarter.getSCSInitialSetup().setUsePerfectSensors(true);

      double controllerDT = robotModel.getControllerDT();
      double simDT = robotModel.getSimulateDT();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "valkyrie_wrench_estimation_sim");
      int rigidBodyHashCode = robotModel.createFullRobotModel().getOneDoFJointByName(jointName).getSuccessor().hashCode();

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      HumanoidFloatingRootJointRobot scsRobot = simulationStarter.getSDFRobot();
      ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp0", offset, scsRobot);
      Joint scsJoint = scsRobot.getJoint(jointName);
      scsJoint.addExternalForcePoint(externalForcePoint);
      SimulationConstructionSet scs = simulationStarter.getSimulationConstructionSet();

      double forceGraphicScale = 0.05;
      AppearanceDefinition simulatedForceColor = YoAppearance.Red();
      AppearanceDefinition estimatedForceColor = YoAppearance.Green();

      YoFrameVector3D estimatedForce = new YoFrameVector3D("estimatedForce", ReferenceFrame.getWorldFrame(), scs.getRootRegistry());
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
      externalForcePointViz.add(simulatedForcePoint);
      externalForcePointViz.add(estimatedForceVector);
      graphicsListRegistry.registerYoGraphicsList(externalForcePointViz);

      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setDT(simDT, (int) (controllerDT / simDT));
      setupSingleSupportButton(scs, ros2Node, robotModel.getSimpleRobotName());

      IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ToolboxStateMessage.class, ExternalForceEstimationToolboxModule.getInputTopic(robotModel.getSimpleRobotName()));
      IHMCROS2Publisher<ExternalForceEstimationConfigurationMessage> configurationMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ExternalForceEstimationConfigurationMessage.class, ExternalForceEstimationToolboxModule.getInputTopic(robotModel.getSimpleRobotName()));

      if (mode == Mode.ESTIMATE_FORCE)
      {
         AtomicReference<ExternalForceEstimationOutputStatus> toolboxOutputStatus = new AtomicReference<>();
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, ExternalForceEstimationOutputStatus.class, ExternalForceEstimationToolboxModule.getOutputTopic(robotModel.getSimpleRobotName()), s -> toolboxOutputStatus.set(s.takeNextData()));
         simulationStarter.getAvatarSimulation().getSimulationConstructionSet().addScript(t ->
                                                                                          {
                                                                                             if (toolboxOutputStatus.get() != null)
                                                                                                estimatedForce.set(toolboxOutputStatus.get()
                                                                                                                                      .getEstimatedExternalForces()
                                                                                                                                      .get(0));
                                                                                          });
      }

      JButton wakeupButton = new JButton("Start estimation");
      wakeupButton.addActionListener(e ->
                                     {
                                        ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
                                        configurationMessage.setEstimatorGain(0.7);

                                        if (mode == Mode.ESTIMATE_FORCE)
                                        {
                                           configurationMessage.setCalculateRootJointWrench(true);
                                           configurationMessage.setEstimateContactLocation(false);
                                           configurationMessage.getContactPointPositions().add().set(offset);
                                           configurationMessage.getRigidBodyHashCodes().add(rigidBodyHashCode);
                                        }
                                        else
                                        {
                                           configurationMessage.setCalculateRootJointWrench(true);
                                           configurationMessage.setEstimateContactLocation(true);
                                        }

                                        configurationMessagePublisher.publish(configurationMessage);

                                        ThreadTools.sleep(2);

                                        ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
                                        toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
                                        toolboxStateMessage.setRequestLogging(false);
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

      new ExternalForceEstimationToolboxModule(robotModel, getCollisionModel(), true, DomainFactory.PubSubImplementation.FAST_RTPS);

      simulationStarter.getAvatarSimulation().start();
      simulationStarter.getAvatarSimulation().simulate();
   }

   private static void setupSingleSupportButton(SimulationConstructionSet scs, ROS2Node ros2Node, String robotName)
   {
      JButton button = new JButton("Single support");

      IHMCROS2Publisher<FootTrajectoryMessage> publisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootTrajectoryMessage.class, ROS2Tools.getControllerInputTopic(robotName));
      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(RobotSide.LEFT, 1.0, new Point3D(0.015, 0.14, 0.25), new Quaternion());
      button.addActionListener(event -> publisher.publish(footTrajectoryMessage));

      scs.addButton(button);
   }

   protected abstract DRCRobotModel getRobotModel();

   protected abstract RobotCollisionModel getCollisionModel();
}
