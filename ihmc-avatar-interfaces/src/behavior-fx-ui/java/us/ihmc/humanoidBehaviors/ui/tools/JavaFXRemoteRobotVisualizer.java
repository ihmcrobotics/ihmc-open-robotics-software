package us.ihmc.humanoidBehaviors.ui.tools;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.ui.references.Activator;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class JavaFXRemoteRobotVisualizer extends RemoteSyncedRobotModel
{
   private GraphicsRobot graphicsRobot;
   private JavaFXGraphics3DNode robotRootNode;

   private Activator robotLoadedActivator = new Activator();
   private final Group rootNode = new Group();

   private final AnimationTimer animationTimer;

   public JavaFXRemoteRobotVisualizer(DRCRobotModel robotModel, Ros2Node ros2Node)
   {
      super(robotModel, ros2Node);

      new Thread(() -> loadRobotModelAndGraphics(robotModel.getRobotDescription()), "RobotVisualizerLoading").start();

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (robotLoadedActivator.poll())
            {
               if (robotLoadedActivator.activationChanged())
               {
                  rootNode.getChildren().add(robotRootNode);
               }

               pollFullRobotModel();

               graphicsRobot.update();
               robotRootNode.update();
            }
         }
      };
   }

   private void loadRobotModelAndGraphics(RobotDescription robotDescription)
   {
      graphicsRobot = new GraphicsIDRobot(robotDescription.getName(), fullRobotModel.getElevator(), robotDescription);
      robotRootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      robotRootNode.setMouseTransparent(true);
      addNodesRecursively(graphicsRobot.getRootNode(), robotRootNode);
      robotRootNode.update();

      robotLoadedActivator.activate();
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

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
