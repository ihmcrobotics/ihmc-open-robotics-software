package us.ihmc.humanoidBehaviors.ui.model;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;

public class FXUIBehavior
{
   private final Group root = new Group();
   protected final ObservableList<Node> rootChildren = root.getChildren();

   protected void registerGraphic(FXUIGraphic graphic)
   {
      rootChildren.add(graphic.getRoot());
   }

   protected void removeGraphic(FXUIGraphic graphic)
   {
      rootChildren.remove(graphic.getRoot());
   }

   public Group getRoot()
   {
      return root;
   }
}
