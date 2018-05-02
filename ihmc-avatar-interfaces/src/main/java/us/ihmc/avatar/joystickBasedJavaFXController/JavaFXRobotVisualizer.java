package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class JavaFXRobotVisualizer
{
   private final GraphicsRobot graphicsRobot;
   private final JavaFXGraphics3DNode rootNode;
   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJoint[] allJoints;
   private final int jointNameHash;
   private final AtomicReference<RigidBodyTransform> newRootJointPoseReference = new AtomicReference<>(null);
   private final AtomicReference<float[]> newJointConfigurationReference = new AtomicReference<>(null);

   private final AnimationTimer animationTimer;

   public JavaFXRobotVisualizer(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      RobotDescription robotDescription = fullRobotModelFactory.getRobotDescription();
      graphicsRobot = new GraphicsRobot(robotDescription.getName(), fullRobotModel.getElevator(), robotDescription);
      rootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      rootNode.setMouseTransparent(true);
      addNodesRecursively(graphicsRobot.getRootNode(), rootNode);
      rootNode.update();

      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            RigidBodyTransform newRootJointPose = newRootJointPoseReference.getAndSet(null);
            if (newRootJointPose != null)
               fullRobotModel.getRootJoint().setPositionAndRotation(newRootJointPose);

            float[] newJointConfiguration = newJointConfigurationReference.getAndSet(null);
            if (newJointConfiguration != null)
            {
               for (int i = 0; i < allJoints.length; i++)
                  allJoints[i].setQ(newJointConfiguration[i]);
            }
            fullRobotModel.getElevator().updateFramesRecursively();
            graphicsRobot.update();
            rootNode.update();
         }
      };
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
   }

   private void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode)
   {
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
      parentNode.addChild(node);
      graphics3dNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node));
   }

   public void submitNewConfiguration(RobotConfigurationData robotConfigurationData)
   {
      if (robotConfigurationData.getJointNameHash() != jointNameHash)
         throw new RuntimeException("Joint names do not match for RobotConfigurationData");

      newRootJointPoseReference.set(new RigidBodyTransform(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation()));
      newJointConfigurationReference.set(robotConfigurationData.getJointAngles().toArray());
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
