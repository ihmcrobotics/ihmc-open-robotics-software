package us.ihmc.rdx.ui.behavior;

import us.ihmc.rdx.ui.GDXImGuiBasedUI;
import us.ihmc.rdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;

public class ImGuiGDXRootBehaviorUI extends ImGuiGDXBehaviorUIInterface
{
   private final Runnable renderTreeNodeImGuiWidgets;

   public ImGuiGDXRootBehaviorUI(Runnable renderTreeNodeImGuiWidgets)
   {
      this.renderTreeNodeImGuiWidgets = renderTreeNodeImGuiWidgets;
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {

   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      renderTreeNodeImGuiWidgets.run();
   }

   @Override
   public void update()
   {

   }

   @Override
   public void destroy()
   {

   }

   @Override
   public String getName()
   {
      return "Behavior Module";
   }
}
