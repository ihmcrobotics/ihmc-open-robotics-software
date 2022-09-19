package us.ihmc.valkyrie.joystick;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotModels.description.RobotDefinitionConverter;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class ValkyrieJavaFXMotionPreviewVisualizer
{
   private final GraphicsRobot graphicsRobot;
   private final JavaFXGraphics3DNode rootNode;
   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] allJoints;
   private final AtomicReference<RigidBodyTransform> newRootJointPoseReference = new AtomicReference<>(null);
   private final AtomicReference<float[]> newJointConfigurationReference = new AtomicReference<>(null);

   private final AtomicBoolean enable = new AtomicBoolean(false);
   private double localTime = 0.0;
   private KinematicsPlanningToolboxOutputStatus packetInProgress = null;

   private final double onetickTime = 0.2;

   private final AnimationTimer animationTimer;

   public ValkyrieJavaFXMotionPreviewVisualizer(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      RobotDefinition robotDefinition = fullRobotModelFactory.getRobotDefinition();
      graphicsRobot = new GraphicsIDRobot(robotDefinition.getName(),
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
               localTime = 0;
               rootNode.setVisible(false);
               return;
            }

            if (packetInProgress != null)
            {
               rootNode.setVisible(true);
               double trajectoryTime = packetInProgress.getKeyFrameTimes().get(packetInProgress.getKeyFrameTimes().size() - 1);

               KinematicsToolboxOutputStatus toShowOutputStatus = findFrameFromTime(packetInProgress, localTime); //get close output status based on inverseKinematicsSolution.trajectoryTimes.

               if (toShowOutputStatus == null)
               {
                  LogTools.error("Could not find frame for t = " + localTime + ", stopping preview.", this);
                  enable(false);
                  return;
               }

               visualizeFrame(toShowOutputStatus);
               localTime += onetickTime;

               // for loop
               if (localTime >= trajectoryTime)
               {
                  localTime = 0.0;
               }
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

   public void submitKinematicsPlanningToolboxOutputStatus(KinematicsPlanningToolboxOutputStatus outputStatus)
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
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
      parentNode.addChild(node);
      graphics3dNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node));
   }

   private KinematicsToolboxOutputStatus findFrameFromTime(KinematicsPlanningToolboxOutputStatus outputStatus, double time)
   {
      if (time <= 0.0)
         return outputStatus.getRobotConfigurations().get(0);
      else if (time >= outputStatus.getKeyFrameTimes().get(outputStatus.getKeyFrameTimes().size() - 1))
         return outputStatus.getRobotConfigurations().get(outputStatus.getRobotConfigurations().size() - 1);
      else
      {
         double timeGap = 0.0;

         int indexOfFrame = 0;
         int numberOfTrajectoryTimes = outputStatus.getKeyFrameTimes().size();

         for (int i = 1; i < numberOfTrajectoryTimes; i++)
         {
            timeGap = time - outputStatus.getKeyFrameTimes().get(i);
            if (timeGap < 0)
            {
               indexOfFrame = i;
               break;
            }
         }

         KinematicsToolboxOutputStatus frameOne = outputStatus.getRobotConfigurations().get(indexOfFrame - 1);
         KinematicsToolboxOutputStatus frameTwo = outputStatus.getRobotConfigurations().get(indexOfFrame);

         double timeOne = outputStatus.getKeyFrameTimes().get(indexOfFrame - 1);
         double timeTwo = outputStatus.getKeyFrameTimes().get(indexOfFrame);

         double alpha = (time - timeOne) / (timeTwo - timeOne);

         return MessageTools.interpolateMessages(frameOne, frameTwo, alpha);
      }
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
