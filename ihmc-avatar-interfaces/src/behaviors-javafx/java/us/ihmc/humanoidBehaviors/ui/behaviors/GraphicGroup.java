package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;

import java.util.ArrayList;

public class GraphicGroup
{
   private final Group parent;
   private final ArrayList<Node> graphicNodes = new ArrayList<>();
   private boolean enabled = false;

   public GraphicGroup(Group parent)
   {
      this.parent = parent;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
      if (enabled)
      {
         for (Node graphicNode : graphicNodes)
         {
            Platform.runLater(() -> parent.getChildren().add(graphicNode));
         }
      }
      else
      {
         for (Node graphicNode : graphicNodes)
         {
            Platform.runLater(() -> parent.getChildren().remove(graphicNode));
         }
      }
   }

   public void removeAll()
   {
      Platform.runLater(() -> parent.getChildren().removeAll());
      graphicNodes.clear();
   }

   public void add(Node graphicNode)
   {
      graphicNodes.add(graphicNode);
      if (enabled)
      {
         Platform.runLater(() -> parent.getChildren().add(graphicNode));
      }
   }
}
