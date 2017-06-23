package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class HideShowViewportAction extends SCSAction implements ViewportSelectorCommandListener
{
   private ViewportSelectorCommandExecutor viewportSelector;

   public HideShowViewportAction(ViewportSelectorCommandExecutor viewportSelector)
   {
      super("Hide Viewport",
              "",
              KeyEvent.VK_V,
              "Hide Viewport",
              "Stop displaying this viewport."
      );

      this.viewportSelector = viewportSelector;

      viewportSelector.registerViewportSelectorCommandListener(this);
   }

   @Override
   public void doAction()
   {
      if (viewportSelector.isViewportHidden())
      {
         viewportSelector.showViewport();
      }
      else
      {
         viewportSelector.hideViewport();
      }
   }

   @Override
   public void updateViewportStatus()
   {
      if (viewportSelector.isViewportHidden())
      {
         putValue(NAME, "Show Viewport");
      }
      else
      {
         putValue(NAME, "Hide Viewport");
      }
   }

   @Override
   public void closeAndDispose()
   {
      if (viewportSelector != null) viewportSelector.closeAndDispose();
      viewportSelector = null;
   }

}
