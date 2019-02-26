package us.ihmc.humanoidBehaviors.ui.model;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.humanoidBehaviors.ui.model.FXUIGraphic;

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
      graphic.start();
   }

   protected void removeGraphic(FXUIGraphic graphic)
   {
      rootChildren.remove(graphic.getRoot());
      graphics.remove(graphic);
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
