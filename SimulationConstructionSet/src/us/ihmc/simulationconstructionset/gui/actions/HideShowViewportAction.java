package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;

public class HideShowViewportAction extends AbstractAction implements ViewportSelectorCommandListener
{
   private static final long serialVersionUID = 1774088226210361744L;
   private ViewportSelectorCommandExecutor viewportSelector;

   public HideShowViewportAction(ViewportSelectorCommandExecutor viewportSelector)
   {
      super("Hide Viewport");
      this.viewportSelector = viewportSelector;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_V));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");

      viewportSelector.registerViewportSelectorCommandListener(this);
   }

   @Override
   public void actionPerformed(ActionEvent e)
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
