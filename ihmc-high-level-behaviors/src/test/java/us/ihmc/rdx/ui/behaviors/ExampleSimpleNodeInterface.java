package us.ihmc.rdx.ui.behaviors;

import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;

public class ExampleSimpleNodeInterface extends RDXBehaviorUIInterface
{
   public ExampleSimpleNodeInterface(String name)
   {
      getDefinition().setName(name);
   }

   @Override
   public void create(RDXBaseUI baseUI)
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
   public void destroy()
   {

   }
}
