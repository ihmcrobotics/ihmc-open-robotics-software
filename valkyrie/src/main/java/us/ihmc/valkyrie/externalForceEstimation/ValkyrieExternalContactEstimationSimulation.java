package us.ihmc.valkyrie.externalForceEstimation;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalForceEstimationToolboxModule;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
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
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import javax.swing.*;

public class ValkyrieExternalContactEstimationSimulation
{
   private boolean useExperimentalPhysicsEngine = true;

   private static final double simDT = 2e-4; // normally 6.6e-4. (controlDT=4e-3)
   private static final Vector3D initialForce = new Vector3D(0.0, 0.0, 0.0);

   //      String jointName = "torsoRoll"; Point3D offset = new Point3D(0.156, 0.093, 0.197);
   //      String jointName = "pelvis";  Point3D offset =new Point3D(0.131, 0.000, -0.044);
   //      String jointName = "torsoRoll"; Point3D offset = new Point3D(0.113, -0.050, 0.090);
   String jointName = "torsoRoll"; Point3D offset = new Point3D(0.137, 0.050, 0.329);
   //      String jointName = "torsoRoll"; Point3D offset = new Point3D(0.124, -0.176, 0.259);
   //      String jointName = "leftForearmYaw"; Point3D offset = new Point3D(0.081, 0.020, 0.026);
   //      String jointName = "leftForearmYaw"; Point3D offset = new Point3D(0.076, 0.120, -0.036);
   //            String jointName = "leftForearmYaw"; Point3D offset = new Point3D(-0.068, 0.170, -0.033);
   //      String jointName = "leftForearmYaw"; Point3D offset = new Point3D(0.071, 0.259, 0.009);
   //      String jointName = "rightForearmYaw"; Point3D offset = new Point3D(0.081, -0.020, 0.026);
   //      String jointName = "rightForearmYaw"; Point3D offset = new Point3D(0.076, -0.120, -0.036);
   //      String jointName = "rightForearmYaw"; Point3D offset = new Point3D(-0.068, -0.170, -0.033);
   //      String jointName = "rightForearmYaw"; Point3D offset = new Point3D(0.071, -0.245, -0.034);
   //      String jointName = "pelvis";  Point3D offset =new Point3D(0.140, 0.000, -0.100);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(0.100, 0.122, 0.045);
   //      String jointName = "pelvis";  Point3D offset =new Point3D(-0.130, 0.052, -0.200);
   //      String jointName = "pelvis";  Point3D offset =new Point3D(0.133, 0.000, -0.261);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(0.019, 0.187, -0.072);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(0.057, 0.192, -0.131);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(-0.030, 0.192, -0.235);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(0.144, 0.132, -0.208);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(0.121, 0.172, -0.242);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(0.048, 0.199, -0.289);
   //      String jointName = "rightHipPitch"; Point3D offset = new Point3D(0.019, -0.187, -0.072);
   //      String jointName = "rightHipPitch"; Point3D offset = new Point3D(0.057, -0.192, -0.131);
   //      String jointName = "rightHipPitch"; Point3D offset = new Point3D(-0.030, -0.192, -0.235);
   //      String jointName = "rightHipPitch"; Point3D offset = new Point3D(0.144, -0.132, -0.208);
   //      String jointName = "rightHipPitch"; Point3D offset = new Point3D(0.121, -0.172, -0.242);
   //      String jointName = "rightHipPitch"; Point3D offset = new Point3D(0.048, -0.199, -0.289);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(0.109, 0.083, -0.422);
   //      String jointName = "leftHipPitch"; Point3D offset = new Point3D(0.043, 0.161, -0.444);
   //      String jointName = "leftKneePitch"; Point3D offset = new Point3D(-0.044, 0.104, -0.127);
   //      String jointName = "leftKneePitch"; Point3D offset = new Point3D(0.084, 0.058, -0.160);
   //      String jointName = "leftKneePitch"; Point3D offset = new Point3D(0.079, -0.014, -0.348);
   //      String jointName = "rightHipPitch"; Point3D offset = new Point3D(0.109, -0.083, -0.422);
   //      String jointName = "rightHipPitch"; Point3D offset = new Point3D(0.043, -0.161, -0.444);
   //      String jointName = "rightKneePitch"; Point3D offset = new Point3D(-0.044, -0.104, -0.127);
   //      String jointName = "rightKneePitch"; Point3D offset = new Point3D(0.084, -0.058, -0.160);
   //      String jointName = "rightKneePitch"; Point3D offset = new Point3D(0.079, 0.014, -0.348);

   public ValkyrieExternalContactEstimationSimulation()
   {
      ValkyrieRobotVersion version = ValkyrieRobotVersion.FINGERLESS;
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, version);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new FlatGroundEnvironment());
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.getSCSInitialSetup().setUseExperimentalPhysicsEngine(useExperimentalPhysicsEngine);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.createSimulation(null, false, false);
      simulationStarter.getSCSInitialSetup().setUsePerfectSensors(true);

      if (useExperimentalPhysicsEngine)
      {
         addExternalForcePoints(simulationStarter, Pair.of(jointName, offset));
      }

      double controllerDT = robotModel.getControllerDT();
      RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS, "valkyrie_wrench_estimation_sim");

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoRegistry scsRootRegistry = simulationStarter.getAvatarSimulation().getSimulationConstructionSet().getRootRegistry();

      HumanoidFloatingRootJointRobot scsRobot = simulationStarter.getSDFRobot();

      if (!useExperimentalPhysicsEngine)
      {
         ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp0", offset, scsRobot);
         Joint scsJoint = scsRobot.getJoint(jointName);
         scsJoint.addExternalForcePoint(externalForcePoint);
         externalForcePoint.setForce(initialForce);

         FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
         RigidBodyBasics endEffector = fullRobotModel.getOneDoFJointByName(jointName).getSuccessor();

         double forceGraphicScale = 0.05;
         AppearanceDefinition simulatedForceColor = YoAppearance.Red();
         YoFrameVector3D estimatedForce = new YoFrameVector3D("estimatedForce", ReferenceFrame.getWorldFrame(), scsRootRegistry);

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
      }

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
                                        configurationMessage.setCalculateRootJointWrench(false);
                                        configurationMessage.setEstimateContactLocation(true);
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

      simulationStarter.getAvatarSimulation().start();
      simulationStarter.getAvatarSimulation().simulate();
      ros2Node.spin();
   }

   /**
    * When using experimental physics engine
    */
   private static void addExternalForcePoints(DRCSimulationStarter simulationStarter, Pair<String, Tuple3DReadOnly>... externalForcePointsToAdd)
   {
      HumanoidFloatingRootJointRobot sdfRobot = simulationStarter.getSDFRobot();
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicsList efpGraphicsList = new YoGraphicsList("simulatedExternalForce");

      for (int i = 0; i < externalForcePointsToAdd.length; i++)
      {
         String efpName = "efp" + i;
         String jointName = externalForcePointsToAdd[i].getLeft();
         Tuple3DReadOnly offset = externalForcePointsToAdd[i].getRight();

         ExternalForcePoint externalForcePoint = new ExternalForcePoint(efpName, offset, sdfRobot);
         sdfRobot.getJoint(jointName).addExternalForcePoint(externalForcePoint);

         YoGraphicVector simulatedForceVector = new YoGraphicVector("force" + i,
                                                                    externalForcePoint.getYoPosition(),
                                                                    externalForcePoint.getYoForce(),
                                                                    0.05,
                                                                    YoAppearance.Red());
         YoGraphicPosition simulatedForcePoint = new YoGraphicPosition("forcePoint" + i, externalForcePoint.getYoPosition(), 0.025, YoAppearance.Red());
         efpGraphicsList.add(simulatedForceVector);
         efpGraphicsList.add(simulatedForcePoint);
      }

      graphicsListRegistry.registerYoGraphicsList(efpGraphicsList);
      simulationStarter.getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);
   }

   public static void main(String[] args)
   {
      new ValkyrieExternalContactEstimationSimulation();
   }
}

