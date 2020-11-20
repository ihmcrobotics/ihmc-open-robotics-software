package us.ihmc.humanoidBehaviors.ui;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.function.Consumer;

public abstract class BehaviorUIInterface
{
   private final SubScene sceneNode;
   private final Pane visualizationPane;
   private final ROS2NodeInterface ros2Node;
   private final Messager behaviorMessager;
   private final DRCRobotModel robotModel;

   private final Group group3D = new Group();
   private final Pane pane = JavaFXMissingTools.loadFromFXML(this);

   protected BehaviorUIInterface(SubScene sceneNode, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.sceneNode = sceneNode;
      this.visualizationPane = visualizationPane;
      this.ros2Node = ros2Node;
      this.behaviorMessager = behaviorMessager;
      this.robotModel = robotModel;
   }

   public abstract void setEnabled(boolean enabled);

   public abstract void destroy();

   public Group get3DGroup()
   {
      return group3D;
   }

   protected void enable3DGroup(boolean enabled, Node... nodes)
   {
      Consumer<Node> operation = enabled ? group3D.getChildren()::add : group3D.getChildren()::remove;

      for (Node node : nodes)
      {
         operation.accept(node);
      }
   }

   public Pane getPane()
   {
      return pane;
   }

   protected SubScene getSceneNode()
   {
      return sceneNode;
   }

   protected Pane getVisualizationPane()
   {
      return visualizationPane;
   }

   protected ROS2NodeInterface getRos2Node()
   {
      return ros2Node;
   }

   protected Messager getBehaviorMessager()
   {
      return behaviorMessager;
   }

   protected DRCRobotModel getRobotModel()
   {
      return robotModel;
   }
}
