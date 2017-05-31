package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class HideShowViewportAction extends SCSAction implements ViewportSelectorCommandListener
{
   private static final long serialVersionUID = 1774088226210361744L;
   private ViewportSelectorCommandExecutor viewportSelector;

   public HideShowViewportAction(ViewportSelectorCommandExecutor viewportSelector)
   {
      super("Hide Viewport",
              "",
              KeyEvent.VK_V,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.viewportSelector = viewportSelector;

      viewportSelector.registerViewportSelectorCommandListener(this);
   }

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
