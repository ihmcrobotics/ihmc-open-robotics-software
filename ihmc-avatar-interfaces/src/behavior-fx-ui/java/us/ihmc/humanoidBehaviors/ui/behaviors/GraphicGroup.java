package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;

import java.util.ArrayList;

public class GraphicGroup<T extends Node> extends Group
{
   private final ArrayList<T> graphicNodes = new ArrayList<>();

   public void setEnabled(boolean enabled)
   {
      if (!enabled)
      {
         for (T boundingBoxGraphic : graphicNodes)
         {
            Platform.runLater(() -> getChildren().remove(boundingBoxGraphic));
         }
      }
      else
      {
         for (T boundingBoxGraphic : graphicNodes)
         {
            Platform.runLater(() -> getChildren().add(boundingBoxGraphic));
         }
      }
   }

   public void removeAll()
   {
      Platform.runLater(() -> getChildren().removeAll());
      graphicNodes.clear();
   }

   public void add(T graphicNode)
   {
      graphicNodes.add(graphicNode);
      Platform.runLater(() -> getChildren().add(graphicNode));
   }
}
