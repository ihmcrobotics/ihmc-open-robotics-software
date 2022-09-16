package us.ihmc.valkyrie.joystick;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotModels.description.RobotDefinitionConverter;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class ValkyrieJavaFXInverseKinematicsVisualizer
{
   private final GraphicsRobot graphicsRobot;
   private final JavaFXGraphics3DNode rootNode;
   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] allJoints;
   private final AtomicReference<RigidBodyTransform> newRootJointPoseReference = new AtomicReference<>(null);
   private final AtomicReference<float[]> newJointConfigurationReference = new AtomicReference<>(null);

   private final AtomicBoolean enable = new AtomicBoolean(false);
   private KinematicsToolboxOutputStatus packetInProgress = null;

   private final AnimationTimer animationTimer;

   public ValkyrieJavaFXInverseKinematicsVisualizer(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      RobotDefinition robotDefinition = fullRobotModelFactory.getRobotDefinition();
      graphicsRobot = new GraphicsIDRobot(robotDefinition.getName() + "aa",
                                          fullRobotModel.getElevator(),
                                          RobotDefinitionConverter.toGraphicsObjectsHolder(robotDefinition));
      rootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      rootNode.setMouseTransparent(true);
      addNodesRecursively(graphicsRobot.getRootNode(), rootNode);
      rootNode.update();

      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (!enable.get())
            {
               packetInProgress = null;
               rootNode.setVisible(false);
               return;
            }

            if (packetInProgress != null)
            {
               rootNode.setVisible(true);

               KinematicsToolboxOutputStatus toShowOutputStatus = packetInProgress;

               if (toShowOutputStatus == null)
               {
                  enable(false);
                  return;
               }

               visualizeFrame(toShowOutputStatus);
            }

            updateRobotConfiguration();
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

   public void enable(boolean value)
   {
      enable.set(value);
   }

   public void submitKinematicsToolboxOutputStatus(KinematicsToolboxOutputStatus outputStatus)
   {
      packetInProgress = outputStatus;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   private void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode)
   {
      AppearanceDefinition appearance = YoAppearance.YellowGreen();
      appearance.setTransparency(0.9);
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
      parentNode.addChild(node);
      graphics3dNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node));
   }

   private void visualizeFrame(KinematicsToolboxOutputStatus frame)
   {
      float[] joints = new float[allJoints.length];
      for (int i = 0; i < joints.length; i++)
      {
         joints[i] = frame.getDesiredJointAngles().get(i);
      }

      newRootJointPoseReference.set(new RigidBodyTransform(frame.getDesiredRootOrientation(), frame.getDesiredRootPosition()));
      newJointConfigurationReference.set(joints);
   }

   private void updateRobotConfiguration()
   {
      RigidBodyTransform newRootJointPose = newRootJointPoseReference.getAndSet(null);
      if (newRootJointPose != null)
         fullRobotModel.getRootJoint().setJointConfiguration(newRootJointPose);

      float[] newJointConfiguration = newJointConfigurationReference.getAndSet(null);
      if (newJointConfiguration != null)
      {
         for (int i = 0; i < allJoints.length; i++)
            allJoints[i].setQ(newJointConfiguration[i]);
      }
   }
}
