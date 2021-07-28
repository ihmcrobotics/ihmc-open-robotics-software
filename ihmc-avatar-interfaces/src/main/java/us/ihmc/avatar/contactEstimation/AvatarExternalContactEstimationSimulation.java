package us.ihmc.avatar.contactEstimation;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;

import javax.swing.*;

public abstract class AvatarExternalContactEstimationSimulation
{
   public AvatarExternalContactEstimationSimulation(String jointName, Point3D offset, double simDT, boolean estimateContactLocation)
   {
      DRCRobotModel robotModel = getRobotModel();
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new FlatGroundEnvironment());
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.getSCSInitialSetup().setUseExperimentalPhysicsEngine(false);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.createSimulation(null, false, false);
      simulationStarter.getSCSInitialSetup().setUsePerfectSensors(true);

      double controllerDT = robotModel.getControllerDT();
      RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "valkyrie_wrench_estimation_sim");

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      HumanoidFloatingRootJointRobot scsRobot = simulationStarter.getSDFRobot();
      ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp0", offset, scsRobot);
      Joint scsJoint = scsRobot.getJoint(jointName);
      scsJoint.addExternalForcePoint(externalForcePoint);

      double forceGraphicScale = 0.05;
      AppearanceDefinition simulatedForceColor = YoAppearance.Red();

      YoGraphicVector simulatedForceVector = new YoGraphicVector("simulatedForceVector",
                                                                 externalForcePoint.getYoPosition(),
                                                                 externalForcePoint.getYoForce(),
                                                                 forceGraphicScale,
                                                                 simulatedForceColor);

      YoGraphicPosition simulatedForcePoint = new YoGraphicPosition("simulatedForcePoint", externalForcePoint.getYoPosition(), 0.025, simulatedForceColor);

      YoGraphicsList externalForcePointViz = new YoGraphicsList("simulatedExternalForce");
      externalForcePointViz.add(simulatedForceVector);
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
                                                                                                                                                            robotModel
                                                                                                                                                                  .getSimpleRobotName()));

      JButton wakeupButton = new JButton("Start estimation");
      wakeupButton.addActionListener(e ->
                                     {
                                        ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
                                        configurationMessage.setEstimatorGain(0.5);
                                        configurationMessage.setCalculateRootJointWrench(false);
                                        configurationMessage.setEstimateContactLocation(estimateContactLocation);
                                        configurationMessagePublisher.publish(configurationMessage);

                                        ThreadTools.sleep(1);

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

      new ExternalForceEstimationToolboxModule(robotModel, true, DomainFactory.PubSubImplementation.FAST_RTPS);

      simulationStarter.getAvatarSimulation().start();
      simulationStarter.getAvatarSimulation().simulate();
      ros2Node.spin();
   }

   protected abstract DRCRobotModel getRobotModel();
}
