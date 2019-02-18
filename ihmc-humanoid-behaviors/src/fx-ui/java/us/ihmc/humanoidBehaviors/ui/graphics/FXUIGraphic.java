package us.ihmc.humanoidBehaviors.ui.graphics;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.humanoidBehaviors.ui.editors.FXUIEditor;

public abstract class FXUIGraphic extends AnimationTimer
{
   private final Group root = new Group();
   protected final ObservableList<Node> rootChildren = root.getChildren();

   public Group getRoot()
   {
      return root;
   }

   public static final FXUIGraphic NONE = new FXUIGraphic() {
      @Override
      public void handle(long now)
      {

      }
   };
}
