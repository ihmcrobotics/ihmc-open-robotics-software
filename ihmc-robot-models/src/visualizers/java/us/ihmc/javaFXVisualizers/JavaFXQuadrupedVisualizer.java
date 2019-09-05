package us.ihmc.javaFXVisualizers;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.zip.CRC32;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class JavaFXQuadrupedVisualizer
{
   private GraphicsRobot graphicsRobot;
   private JavaFXGraphics3DNode robotRootNode;
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);

   private final FullQuadrupedRobotModel fullRobotModel;

   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;

   private Messager messager = null;
   private Topic<FullQuadrupedRobotModel> robotModelTopic = null;
   private final AtomicReference<RobotConfigurationData> robotConfigurationDataReference = new AtomicReference<>();

   private boolean isRobotLoaded = false;
   private final Group rootNode = new Group();

   public JavaFXQuadrupedVisualizer(FullQuadrupedRobotModelFactory fullRobotModelFactory)
   {
      this(fullRobotModelFactory, graphics -> {});
   }

   public JavaFXQuadrupedVisualizer(FullQuadrupedRobotModelFactory fullRobotModelFactory, Consumer<Graphics3DNode> graphicsMutator)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      allJoints = fullRobotModel.getOneDoFJoints();

      jointNameHash = calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());

      new Thread(() -> loadRobotModelAndGraphics(fullRobotModelFactory, graphicsMutator), "RobotVisualizerLoading").start();
   }

   public void attachMessager(Messager messager, Topic<FullQuadrupedRobotModel> robotModelTopic)
   {
      this.messager = messager;
      this.robotModelTopic = robotModelTopic;
   }

   private static int calculateJointNameHash(OneDoFJointBasics[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJointBasics joint : joints)
      {
         crc.update(joint.getName().getBytes());
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         crc.update(forceSensorDefinition.getSensorName().getBytes());
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         crc.update(imuDefinition.getName().getBytes());
      }

      return (int) crc.getValue();
   }

   public boolean isRobotLoaded()
   {
      return isRobotLoaded;
   }

   private void loadRobotModelAndGraphics(FullQuadrupedRobotModelFactory fullRobotModelFactory, Consumer<Graphics3DNode> graphicsMutator)
   {
      RobotDescription robotDescription = fullRobotModelFactory.getRobotDescription();
      graphicsRobot = new GraphicsIDRobot(robotDescription.getName(), fullRobotModel.getElevator(), robotDescription);
      robotRootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      robotRootNode.setMouseTransparent(true);
      addNodesRecursively(graphicsRobot.getRootNode(), robotRootNode, graphicsMutator);
      robotRootNode.update();

      isRobotLoaded = true;
   }

   public void handle(long now)
   {
      if (!isRobotLoaded)
         return;
      else if (rootNode.getChildren().isEmpty())
         rootNode.getChildren().add(robotRootNode);

      RobotConfigurationData robotConfigurationData = robotConfigurationDataReference.getAndSet(null);
      if (robotConfigurationData == null)
         return;

      if (robotConfigurationData.getJointNameHash() != jointNameHash)
         throw new RuntimeException("Joint names do not match for RobotConfigurationData");

      RigidBodyTransform newRootJointPose = new RigidBodyTransform(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation());
      fullRobotModel.getRootJoint().setJointConfiguration(newRootJointPose);

      float[] newJointConfiguration = robotConfigurationData.getJointAngles().toArray();
      for (int i = 0; i < allJoints.length; i++)
         allJoints[i].setQ(newJointConfiguration[i]);

      fullRobotModel.getElevator().updateFramesRecursively();
      graphicsRobot.update();
      robotRootNode.update();

      if (messager != null)
      {
         messager.submitMessage(robotModelTopic, fullRobotModel);
      }
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
   }

   private void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode, Consumer<Graphics3DNode> graphicsMutator)
   {
      graphicsMutator.accept(graphics3dNode);
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
      parentNode.addChild(node);
      graphics3dNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node, graphicsMutator));
   }

   public void submitNewConfiguration(RobotConfigurationData robotConfigurationData)
   {
      if (robotConfigurationData.getJointNameHash() != jointNameHash)
         throw new RuntimeException("Joint names do not match for RobotConfigurationData");

      robotConfigurationDataReference.set(robotConfigurationData);
   }

   public FullQuadrupedRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
