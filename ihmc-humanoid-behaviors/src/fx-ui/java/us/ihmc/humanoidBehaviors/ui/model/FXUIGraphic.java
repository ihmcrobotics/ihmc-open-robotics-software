package us.ihmc.humanoidBehaviors.ui.model;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;

public abstract class FXUIGraphic extends AnimationTimer
{
   private final Group root = new Group();
   protected final ObservableList<Node> rootChildren = root.getChildren();

   public Group getRoot()
   {
      return root;
   }
}
