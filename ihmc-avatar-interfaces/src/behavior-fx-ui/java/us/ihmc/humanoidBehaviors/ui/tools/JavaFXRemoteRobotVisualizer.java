package us.ihmc.humanoidBehaviors.ui.tools;

import javafx.scene.Group;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.tools.thread.Activator;

public class JavaFXRemoteRobotVisualizer extends Group
{
   private final RemoteSyncedRobotModel remoteSyncedRobotModel;

   private GraphicsRobot graphicsRobot;
   private JavaFXGraphics3DNode robotRootNode;
   private Activator robotLoadedActivator = new Activator();

   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);

   public JavaFXRemoteRobotVisualizer(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      remoteSyncedRobotModel = new RemoteSyncedRobotModel(robotModel, ros2Node);

      new Thread(() -> loadRobotModelAndGraphics(robotModel.getRobotDescription()), "RobotVisualizerLoading").start();

      animationTimer.start();
   }

   private void handle(long now)
   {
      if (robotLoadedActivator.poll())
      {
         if (robotLoadedActivator.hasChanged())
         {
            getChildren().add(robotRootNode);
         }

         remoteSyncedRobotModel.pollFullRobotModel();

         graphicsRobot.update();
         robotRootNode.update();
      }
   }

   private void loadRobotModelAndGraphics(RobotDescription robotDescription)
   {
      graphicsRobot = new GraphicsIDRobot(robotDescription.getName(), remoteSyncedRobotModel.getFullRobotModel().getElevator(), robotDescription);
      robotRootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      robotRootNode.setMouseTransparent(true);
      addNodesRecursively(graphicsRobot.getRootNode(), robotRootNode);
      robotRootNode.update();

      robotLoadedActivator.activate();
   }

   private void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode)
   {
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
      parentNode.addChild(node);
      graphics3dNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node));
   }
}
