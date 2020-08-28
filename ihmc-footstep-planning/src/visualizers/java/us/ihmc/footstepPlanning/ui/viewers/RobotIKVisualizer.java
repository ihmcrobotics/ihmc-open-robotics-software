package us.ihmc.footstepPlanning.ui.viewers;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.communication.UserInterfaceIKMode;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.ToDoubleFunction;
import java.util.zip.CRC32;

public class RobotIKVisualizer
{
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;

   private final FullHumanoidRobotModel fullRobotModel;

   private final Map<UserInterfaceIKMode, OneDoFJointBasics[]> ikJointPaths = new HashMap<>();
   private final Map<UserInterfaceIKMode, GraphicsIDRobot> limbGraphics = new HashMap<>();
   private final Map<UserInterfaceIKMode, JavaFXGraphics3DNode> ikLimbGraphicsNodes = new HashMap<>();

   private final AtomicReference<RigidBodyTransform> newRootJointPoseReference = new AtomicReference<>(null);
   private final AtomicReference<float[]> newJointConfigurationReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> ikEnabled;
   private final AtomicReference<double[]> ikSolution;
   private final AtomicReference<UserInterfaceIKMode> ikMode;
   private final AtomicBoolean newModeFlag = new AtomicBoolean();

   private boolean isRobotLoaded = false;
   private final Group rootNode = new Group();

   private final AnimationTimer animationTimer;

   public RobotIKVisualizer(FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory, DRCRobotJointMap jointMap, Messager messager)
   {
      fullRobotModel = fullHumanoidRobotModelFactory.createFullRobotModel();
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(),
                                             fullRobotModel.getIMUDefinitions());

      ikEnabled = messager.createInput(FootstepPlannerMessagerAPI.IKEnabled, false);
      ikSolution = messager.createInput(FootstepPlannerMessagerAPI.IKSolution);
      ikMode = messager.createInput(FootstepPlannerMessagerAPI.SelectedIKMode);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.SelectedIKMode, b -> newModeFlag.set(true));

      new Thread(() -> loadRobotModelAndGraphics(fullHumanoidRobotModelFactory, jointMap), "RobotVisualizerLoading").start();

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            UserInterfaceIKMode selectedMode = ikMode.get();

            if (!isRobotLoaded)
            {
               return;
            }
            else if (ikEnabled.get() && rootNode.getChildren().isEmpty())
            {
               rootNode.getChildren().add(ikLimbGraphicsNodes.get(selectedMode));
            }
            else if (!ikEnabled.get())
            {
               if (!rootNode.getChildren().isEmpty())
               {
                  rootNode.getChildren().clear();
               }

               return;
            }

            RigidBodyTransform newRootJointPose = newRootJointPoseReference.getAndSet(null);
            if (newRootJointPose != null)
            {
               fullRobotModel.getRootJoint().setJointConfiguration(newRootJointPose);
            }

            float[] newJointConfiguration = newJointConfigurationReference.getAndSet(null);
            if (newJointConfiguration != null)
            {
               for (int i = 0; i < allJoints.length; i++)
               {
                  allJoints[i].setQ(newJointConfiguration[i]);
               }
            }

            double[] newIKSolution = ikSolution.getAndSet(null);
            GraphicsIDRobot selectedLimbGraphics = limbGraphics.get(selectedMode);
            OneDoFJointBasics[] jointPath = ikJointPaths.get(selectedMode);

            if (newIKSolution != null && newIKSolution.length == jointPath.length)
            {
               for (int i = 0; i < newIKSolution.length; i++)
               {
                  jointPath[i].setQ(newIKSolution[i]);
               }
            }

            fullRobotModel.getElevator().updateFramesRecursively();

            selectedLimbGraphics.update();
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.set(jointPath[0].getFrameAfterJoint().getTransformToWorldFrame());
            selectedLimbGraphics.getGraphicsNode(jointPath[0]).getTransform().set(transformToWorld);

            ikLimbGraphicsNodes.get(selectedMode).update();
         }
      };
   }

   public static int calculateJointNameHash(OneDoFJointBasics[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
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

   private void loadRobotModelAndGraphics(FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory, DRCRobotJointMap jointMap)
   {
      RobotDescription robotDescription = fullHumanoidRobotModelFactory.getRobotDescription();

      // Arm graphics
      for (RobotSide robotSide : RobotSide.values)
      {
         UserInterfaceIKMode ikMode = robotSide == RobotSide.LEFT ? UserInterfaceIKMode.LEFT_ARM : UserInterfaceIKMode.RIGHT_ARM;
         String firstArmJointName = jointMap.getArmJointNamesAsStrings(robotSide).get(0);
         JointBasics firstArmJoint = fullRobotModel.getOneDoFJointByName(firstArmJointName);
         setupGraphics(ikMode, firstArmJoint, fullRobotModel.getHand(robotSide), robotDescription);
      }

      // Neck graphics
      UserInterfaceIKMode neckMode = UserInterfaceIKMode.NECK;
      String firstNeckJointName = jointMap.getNeckJointNamesAsStrings().get(0);
      JointBasics firstNeckJoint = fullRobotModel.getOneDoFJointByName(firstNeckJointName);
      setupGraphics(neckMode, firstNeckJoint, fullRobotModel.getHead(), robotDescription);

      isRobotLoaded = true;
   }

   private void setupGraphics(UserInterfaceIKMode ikMode, JointBasics firstJoint, RigidBodyBasics terminalBody, RobotDescription robotDescription)
   {
      ikJointPaths.put(ikMode, MultiBodySystemTools.createOneDoFJointPath(firstJoint.getPredecessor(), terminalBody));
      limbGraphics.put(ikMode, new GraphicsIDRobot(ikMode.name(), Collections.singletonList(firstJoint), robotDescription, false));

      JavaFXGraphics3DNode jfxArmGraphicsNode = new JavaFXGraphics3DNode(limbGraphics.get(ikMode).getRootNode());
      jfxArmGraphicsNode.setMouseTransparent(true);
      addNodesRecursively(limbGraphics.get(ikMode).getRootNode(), jfxArmGraphicsNode);
      jfxArmGraphicsNode.update();
      ikLimbGraphicsNodes.put(ikMode, jfxArmGraphicsNode);
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

   public void submitNewConfiguration(RigidBodyTransform rootJointTransform, ToDoubleFunction<String> jointAngles)
   {
      newRootJointPoseReference.set(rootJointTransform);
      float[] jointAngleArray = new float[allJoints.length];
      for (int i = 0; i < allJoints.length; i++)
      {
         jointAngleArray[i] = (float) jointAngles.applyAsDouble(allJoints[i].getName());
      }
      newJointConfigurationReference.set(jointAngleArray);
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
