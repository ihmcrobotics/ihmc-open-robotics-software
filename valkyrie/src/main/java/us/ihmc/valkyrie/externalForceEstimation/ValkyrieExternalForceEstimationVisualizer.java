package us.ihmc.valkyrie.externalForceEstimation;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.swing.JButton;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizerStateListener;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieExternalForceEstimationVisualizer implements SCSVisualizerStateListener
{
   private final RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, "valkyrie_wrench_estimation_visualizer");
   private final ROS2Topic inputTopic;
   private final ROS2Topic outputTopic;
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
      ValkyrieRobotVersion version = ValkyrieExternalForceEstimationModule.version;
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, version);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      inputTopic = ExternalForceEstimationToolboxModule.getInputTopic(robotModel.getSimpleRobotName());
      outputTopic = ExternalForceEstimationToolboxModule.getOutputTopic(robotModel.getSimpleRobotName());

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
   public void starting(SimulationConstructionSet scs, Robot robot, YoRegistry registry)
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
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    ExternalForceEstimationOutputStatus.class,
                                                    outputTopic,
                                           s -> toolboxOutputStatus.set(s.takeNextData()));

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

                       ThreadTools.sleep(100);
                    }
                 }).start();

      // ----- Toolbox Control ----- //
      IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                                ToolboxStateMessage.class,
                                                                                                                inputTopic);
      IHMCRealtimeROS2Publisher<ExternalForceEstimationConfigurationMessage> configurationMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                                                                ExternalForceEstimationConfigurationMessage.class,
                                                                                                                                                inputTopic);
      JButton wakeupButton = new JButton("Start");
      wakeupButton.addActionListener(e ->
                                     {
                                        ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
                                        configurationMessage.setEstimatorGain(0.5);
                                        configurationMessage.getRigidBodyHashCodes().add(endEffectorHashCode);
                                        configurationMessage.getContactPointPositions().add().set(externalForcePointOffset);
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
