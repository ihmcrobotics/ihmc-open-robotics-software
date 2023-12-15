package us.ihmc.rdx.ui.behavior;

import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;

public class RDXRootBehaviorUI extends RDXBehaviorUIInterface
{
   private final Runnable renderTreeNodeImGuiWidgets;

   public RDXRootBehaviorUI(Runnable renderTreeNodeImGuiWidgets)
   {
      this.renderTreeNodeImGuiWidgets = renderTreeNodeImGuiWidgets;
   }

   @Override
   public void create(RDXBaseUI baseUI)
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

   public String getName()
   {
      return "Behavior Module";
   }
}
