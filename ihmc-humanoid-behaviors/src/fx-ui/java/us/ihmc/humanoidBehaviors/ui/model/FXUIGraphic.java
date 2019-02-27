package us.ihmc.humanoidBehaviors.ui.model;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.messager.Messager;

public abstract class FXUIGraphic extends AnimationTimer
{
   protected final Messager messager;

   private final Group root = new Group();
   protected final ObservableList<Node> rootChildren = root.getChildren();

   public FXUIGraphic(Messager messager)
   {
      this.messager = messager;
   }

   public Group getRoot()
   {
      return root;
   }
}
