package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.layout.AnchorPane;
import us.ihmc.humanoidBehaviors.ui.graphics.FXUIGraphic;

import java.util.ArrayList;

public class FXUIBehavior
{
   private final ArrayList<FXUIGraphic> graphics = new ArrayList<>();

   private final Group root = new Group();
   protected final ObservableList<Node> rootChildren = root.getChildren();

   protected void registerGraphic(FXUIGraphic graphic)
   {
      graphics.add(graphic);
      rootChildren.add(graphic.getRoot());
   }

   public Group getRoot()
   {
      return root;
   }

   public void start()
   {
      graphics.forEach(FXUIGraphic::start);
   }

   public void stop()
   {
      graphics.forEach(FXUIGraphic::stop);
   }
}
