package us.ihmc.footstepPlanning.ui.viewers;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

import java.util.Collections;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.ToDoubleFunction;
import java.util.zip.CRC32;

public class RobotIKVisualizer
{
   private JavaFXGraphics3DNode robotRootNode;
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;

   private final FullHumanoidRobotModel fullRobotModel;

   private final SideDependentList<OneDoFJointBasics[]> armJointPaths = new SideDependentList<>();
   private final SideDependentList<GraphicsIDRobot> armGraphics = new SideDependentList<>();
   private final SideDependentList<JavaFXGraphics3DNode> armGraphicsNodes = new SideDependentList<>();

   private final AtomicReference<RigidBodyTransform> newRootJointPoseReference = new AtomicReference<>(null);
   private final AtomicReference<float[]> newJointConfigurationReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> ikEnabled;
   private final AtomicReference<double[]> ikSolution;
   private final AtomicReference<RobotSide> ikSide;
   private final AtomicBoolean newSideFlag = new AtomicBoolean();

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
      ikSide = messager.createInput(FootstepPlannerMessagerAPI.SelectedIKSide);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.SelectedIKSide, b -> newSideFlag.set(true));

      new Thread(() -> loadRobotModelAndGraphics(fullHumanoidRobotModelFactory, jointMap), "RobotVisualizerLoading").start();

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            RobotSide selectedSide = ikSide.get();

            if (!isRobotLoaded)
            {
               return;
            }
            else if (ikEnabled.get() && rootNode.getChildren().isEmpty())
            {
               rootNode.getChildren().add(armGraphicsNodes.get(selectedSide));
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

            double[] newIKArmSolution = ikSolution.getAndSet(null);
            GraphicsIDRobot selectedArmGraphics = armGraphics.get(selectedSide);
            OneDoFJointBasics[] armJointPath = armJointPaths.get(selectedSide);

            if (newIKArmSolution != null && newIKArmSolution.length == armJointPath.length)
            {
               for (int i = 0; i < newIKArmSolution.length; i++)
               {
                  armJointPath[i].setQ(newIKArmSolution[i]);
               }
            }

            fullRobotModel.getElevator().updateFramesRecursively();

            selectedArmGraphics.update();
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.set(armJointPath[0].getFrameAfterJoint().getTransformToWorldFrame());
            selectedArmGraphics.getGraphicsNode(armJointPath[0]).getTransform().set(transformToWorld);

            armGraphicsNodes.get(selectedSide).update();
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

      for (RobotSide robotSide : RobotSide.values)
      {
         String firstArmJointName = jointMap.getArmJointNamesAsStrings(robotSide).get(0);
         JointBasics firstArmJoint = fullRobotModel.getOneDoFJointByName(firstArmJointName);
         armJointPaths.put(robotSide, MultiBodySystemTools.createOneDoFJointPath(firstArmJoint.getPredecessor(), fullRobotModel.getHand(robotSide)));
         armGraphics.put(robotSide, new GraphicsIDRobot(robotDescription.getName(), Collections.singletonList(firstArmJoint), robotDescription, false));

         JavaFXGraphics3DNode jfxArmGraphicsNode = new JavaFXGraphics3DNode(armGraphics.get(robotSide).getRootNode());
         jfxArmGraphicsNode.setMouseTransparent(true);
         addNodesRecursively(armGraphics.get(robotSide).getRootNode(), jfxArmGraphicsNode);
         jfxArmGraphicsNode.update();
         armGraphicsNodes.put(robotSide, jfxArmGraphicsNode);
      }

      isRobotLoaded = true;
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
