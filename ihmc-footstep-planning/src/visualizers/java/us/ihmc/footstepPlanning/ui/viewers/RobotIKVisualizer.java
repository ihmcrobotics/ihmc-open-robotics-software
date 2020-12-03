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
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.ToDoubleFunction;
import java.util.zip.CRC32;

public class RobotIKVisualizer
{
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;

   private final FullHumanoidRobotModel fullRobotModel;

   private final Map<UserInterfaceIKMode, List<OneDoFJointBasics>> ikJointPaths = new HashMap<>();
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

   public RobotIKVisualizer(FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory, HumanoidJointNameMap jointMap, Messager messager)
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
            if (newModeFlag.getAndSet(false))
            {
               rootNode.getChildren().clear();
            }

            UserInterfaceIKMode selectedMode = ikMode.get();

            if (!isRobotLoaded)
            {
               return;
            }
            else if (ikEnabled.get() && rootNode.getChildren().isEmpty())
            {
               rootNode.getChildren().add(ikLimbGraphicsNodes.get(selectedMode));
            }

            double[] newIKSolution = ikSolution.getAndSet(null);
            GraphicsIDRobot selectedLimbGraphics = limbGraphics.get(selectedMode);
            List<OneDoFJointBasics> ikJointPath = ikJointPaths.get(selectedMode);

            setRobotState(ikJointPath);

            if (newIKSolution != null && newIKSolution.length == ikJointPath.size())
            {
               for (int i = 0; i < newIKSolution.length; i++)
               {
                  ikJointPath.get(i).setQ(newIKSolution[i]);
               }
            }

            fullRobotModel.getElevator().updateFramesRecursively();

            selectedLimbGraphics.update();
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.set(ikJointPath.get(0).getFrameAfterJoint().getTransformToWorldFrame());
            selectedLimbGraphics.getGraphicsNode(ikJointPath.get(0)).getTransform().set(transformToWorld);

            ikLimbGraphicsNodes.get(selectedMode).update();
         }
      };


      messager.registerTopicListener(FootstepPlannerMessagerAPI.IKEnabled, enabled ->
      {
         if (enabled)
         {
            setRobotState(null);
            animationTimer.start();
         }
         else
         {
            animationTimer.stop();
            rootNode.getChildren().clear();
         }
      });
   }

   private void setRobotState(List<OneDoFJointBasics> jointsToExclude)
   {
      RigidBodyTransform newRootJointPose = newRootJointPoseReference.getAndSet(null);
      float[] newJointConfiguration = newJointConfigurationReference.getAndSet(null);

      if (newRootJointPose != null)
      {
         fullRobotModel.getRootJoint().setJointConfiguration(newRootJointPose);
      }

      if (newJointConfiguration != null)
      {
         for (int i = 0; i < allJoints.length; i++)
         {
            OneDoFJointBasics joint = allJoints[i];
            if (jointsToExclude == null || !jointsToExclude.contains(joint))
            {
               joint.setQ(newJointConfiguration[i]);
            }
         }
      }
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

   private void loadRobotModelAndGraphics(FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory, HumanoidJointNameMap jointMap)
   {
      RobotDescription robotDescription = fullHumanoidRobotModelFactory.getRobotDescription();

      for (RobotSide robotSide : RobotSide.values)
      {
         // Arm graphics
         UserInterfaceIKMode armIKMode = robotSide == RobotSide.LEFT ? UserInterfaceIKMode.LEFT_ARM : UserInterfaceIKMode.RIGHT_ARM;
         String firstArmJointName = jointMap.getArmJointNamesAsStrings(robotSide).get(0);
         JointBasics firstArmJoint = fullRobotModel.getOneDoFJointByName(firstArmJointName);
         setupGraphics(armIKMode, firstArmJoint, fullRobotModel.getHand(robotSide), robotDescription);

         // Leg graphics
         UserInterfaceIKMode legIKMode = robotSide == RobotSide.LEFT ? UserInterfaceIKMode.LEFT_LEG : UserInterfaceIKMode.RIGHT_LEG;
         String firstLegJointName = jointMap.getLegJointNamesAsStrings(robotSide).get(0);
         JointBasics firstLegJoint = fullRobotModel.getOneDoFJointByName(firstLegJointName);
         setupGraphics(legIKMode, firstLegJoint, fullRobotModel.getFoot(robotSide), robotDescription);
      }

      // Neck graphics
      UserInterfaceIKMode neckMode = UserInterfaceIKMode.NECK;
      String firstNeckJointName = jointMap.getNeckJointNamesAsStrings().get(0);
      JointBasics firstNeckJoint = fullRobotModel.getOneDoFJointByName(firstNeckJointName);
      setupGraphics(neckMode, firstNeckJoint, fullRobotModel.getHead(), robotDescription);

      // Chest graphics
      UserInterfaceIKMode chestMode = UserInterfaceIKMode.CHEST;
      String firstSpineJointName = jointMap.getSpineJointNamesAsStrings().get(0);
      JointBasics firstSpineJoint = fullRobotModel.getOneDoFJointByName(firstSpineJointName);
      setupGraphics(chestMode, firstSpineJoint, fullRobotModel.getChest(), robotDescription);

      isRobotLoaded = true;
   }

   private void setupGraphics(UserInterfaceIKMode ikMode, JointBasics firstJoint, RigidBodyBasics terminalBody, RobotDescription robotDescription)
   {
      OneDoFJointBasics[] jointPathArray = MultiBodySystemTools.createOneDoFJointPath(firstJoint.getPredecessor(), terminalBody);
      ikJointPaths.put(ikMode, Arrays.asList(jointPathArray));
      limbGraphics.put(ikMode,
                       new GraphicsIDRobot(ikMode.name(),
                                           Collections.singletonList(firstJoint),
                                           robotDescription,
                                           false,
                                           Collections.singletonList(terminalBody)));

      JavaFXGraphics3DNode jfxArmGraphicsNode = new JavaFXGraphics3DNode(limbGraphics.get(ikMode).getRootNode());
      jfxArmGraphicsNode.setMouseTransparent(true);
      addNodesRecursively(limbGraphics.get(ikMode).getRootNode(), jfxArmGraphicsNode);
      jfxArmGraphicsNode.update();
      ikLimbGraphicsNodes.put(ikMode, jfxArmGraphicsNode);
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
