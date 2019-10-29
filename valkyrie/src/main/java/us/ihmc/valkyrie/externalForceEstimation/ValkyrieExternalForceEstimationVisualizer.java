package us.ihmc.valkyrie.externalForceEstimation;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import javax.swing.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ValkyrieExternalForceEstimationVisualizer implements SCSVisualizerStateListener
{
   private final RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, "valkyrie_wrench_estimation_visualizer");
   private final MessageTopicNameGenerator subTopicNameGenerator;
   private final MessageTopicNameGenerator pubTopicNameGenerator;
   private final int endEffectorHashCode;
   private final Vector3D externalForcePointOffset = new Vector3D();
   private String endEffectorName;

   public ValkyrieExternalForceEstimationVisualizer()
   {
      SCSVisualizer scsVisualizer = new SCSVisualizer(16384);
      scsVisualizer.setVariableUpdateRate(8);
      scsVisualizer.addSCSVisualizerStateListener(this);
      scsVisualizer.setShowOverheadView(true);

      // ----- Toolbox Output Display -----//
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      subTopicNameGenerator = ExternalForceEstimationToolboxModule.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName());
      pubTopicNameGenerator = ExternalForceEstimationToolboxModule.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName());

      // ----- Root Joint ----- //
      //      RigidBodyBasics endEffector = fullRobotModel.getRootBody();

      // ----- 1DOF Joints ----- //
      //      endEffectorName = "torsoRoll"; // Chest
      //      endEffectorName = "rightShoulderRoll"; // Shoulder
      endEffectorName = "rightElbowPitch"; // Elbow
      //      endEffectorName = "rightForearmYaw"; // Forearm
      //      endEffectorName = "rightWristRoll"; // Wrist roll
      //      endEffectorName = "rightWristPitch"; // Wrist pitch
      RigidBodyBasics endEffector = fullRobotModel.getOneDoFJointByName(endEffectorName).getSuccessor();

      endEffectorHashCode = endEffector.hashCode();
      externalForcePointOffset.set(0.0, -0.35, -0.03);

      // ----- Client -----//
      YoVariableClient client = new YoVariableClient(scsVisualizer);
      client.startWithHostSelector();
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      // ----- Toolbox Output Display ----- //
      //      Joint scsEndEffector = robot.getRootJoints().get(0);
      Joint scsEndEffector = robot.getJoint(endEffectorName);
      KinematicPoint externalForcePoint = new KinematicPoint("efp", robot);
      externalForcePoint.setOffsetJoint(externalForcePointOffset);
      scsEndEffector.addKinematicPoint(externalForcePoint);

      double forceGraphicScale = 0.05;
      AppearanceDefinition estimatedForceColor = YoAppearance.Green();
      YoFrameVector3D estimatedForce = new YoFrameVector3D("estimatedForce", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicVector estimatedForceVector = new YoGraphicVector("estimatedForceVector",
                                                                 externalForcePoint.getYoPosition(),
                                                                 estimatedForce,
                                                                 forceGraphicScale, estimatedForceColor);

      scs.addYoGraphic(estimatedForceVector);

      AtomicReference<ExternalForceEstimationOutputStatus> toolboxOutputStatus = new AtomicReference<>();
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ExternalForceEstimationOutputStatus.class,
                                           pubTopicNameGenerator,
                                           s -> toolboxOutputStatus.set(s.takeNextData()));

      AtomicBoolean tare = new AtomicBoolean();
      AtomicBoolean reset = new AtomicBoolean();
      Vector3D tareOffset = new Vector3D();

      new Thread(() ->
                 {
                    while(true)
                    {
                       if(reset.getAndSet(false))
                       {
                          tareOffset.setToZero();
                       }

                       if(toolboxOutputStatus.get() != null)
                       {
                          if(tare.getAndSet(false))
                          {
                             tareOffset.set(toolboxOutputStatus.get().getEstimatedExternalForce());
                          }

                          estimatedForce.set(toolboxOutputStatus.get().getEstimatedExternalForce());
                          estimatedForce.sub(tareOffset);
                       }

                       ThreadTools.sleep(100);
                    }
                 }).start();

      // ----- Toolbox Control ----- //
      IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                       ToolboxStateMessage.class,
                                                                                                       subTopicNameGenerator);
      IHMCRealtimeROS2Publisher<ExternalForceEstimationConfigurationMessage> configurationMessagePublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                                                       ExternalForceEstimationConfigurationMessage.class,
                                                                                                                                       subTopicNameGenerator);
      JButton wakeupButton = new JButton("Start");
      wakeupButton.addActionListener(e ->
                                     {
                                        ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
                                        configurationMessage.setEstimatorGain(0.5);
                                        configurationMessage.setEndEffectorHashCode(endEffectorHashCode);
                                        configurationMessage.getExternalForcePosition().set(externalForcePointOffset);
                                        configurationMessagePublisher.publish(configurationMessage);

                                        ThreadTools.sleep(1);

                                        ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
                                        toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
                                        toolboxStateMessage.setRequestLogging(true);
                                        toolboxStatePublisher.publish(toolboxStateMessage);
                                     });
      scs.addButton(wakeupButton);

      JButton sleepButton = new JButton("Stop");
      sleepButton.addActionListener(e ->
                                    {
                                       ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
                                       toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.SLEEP);
                                       toolboxStatePublisher.publish(toolboxStateMessage);
                                    });
      scs.addButton(sleepButton);

      JButton tareButton = new JButton("Tare");
      tareButton.addActionListener(e -> tare.set(true));
      scs.addButton(tareButton);

      JButton resetButton = new JButton("Reset");
      resetButton.addActionListener(e -> reset.set(true));
      scs.addButton(resetButton);

      ros2Node.spin();
   }

   public static void main(String[] args)
   {
      new ValkyrieExternalForceEstimationVisualizer();
   }
}
