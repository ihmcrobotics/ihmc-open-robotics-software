package us.ihmc.gdx.ui.behaviors;

import imgui.ImGui;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;

public class ExampleSimpleNodeInterface extends GDXBehaviorUIInterface
{
   public ExampleSimpleNodeInterface(String name) {
      setName(name);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {

   }

   @Override
   public void update()
   {

   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {

   }

   @Override
   public void renderRegularPanelImGuiWidgets()
   {
      ImGui.dummy(80.0f, 30.0f);
   }

   @Override
   public void destroy()
   {

   }

   @Override
   public Point2D getTreeNodeInitialPosition()
   {
      return new Point2D(0, 0);
   }
}
