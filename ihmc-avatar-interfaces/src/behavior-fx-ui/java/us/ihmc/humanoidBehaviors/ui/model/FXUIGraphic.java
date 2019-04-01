package us.ihmc.humanoidBehaviors.ui.model;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;

public abstract class FXUIGraphic
{
   private final Group root = new Group();
   protected final ObservableList<Node> rootChildren = root.getChildren();

   public Group getRoot()
   {
      return root;
   }

   public void setMouseTransparent(boolean transparent)
   {
      root.setMouseTransparent(transparent);
   }
}
