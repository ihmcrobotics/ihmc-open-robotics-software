package us.ihmc.simulationconstructionset.gui.tools;

import java.awt.event.ActionEvent;

import javax.swing.AbstractButton;
import javax.swing.JMenuItem;

public class MultiButtonSelectionDeselectionAction extends AbstractMultiButtonAction
{
   private static final long serialVersionUID = 3282636785189524215L;
   private Boolean enableAll = false;
   private JMenuItem menuItem;
   private String enableAllText;
   private String disableAllText;

   public MultiButtonSelectionDeselectionAction(String name)
   {
      super(name);
      this.enableAll = false;

   }

   public void setShowHideAllMenuItem(JMenuItem menuItem, String enableAllText, String disableAllText)
   {
      this.menuItem = menuItem;
      this.enableAllText=enableAllText;
      this.disableAllText=disableAllText;
      setText();
   }

   private void setText()
   {
      if (menuItem == null)
         throw new RuntimeException("MultiButtonSelectionDeselectionAction: JMenuItem not set.");

      if (enableAll)
      {
         menuItem.setText(enableAllText);
         menuItem.setActionCommand(enableAllText);
      }
      else
      {
         menuItem.setText(disableAllText);
         menuItem.setActionCommand(disableAllText);
      }
      menuItem.updateUI();
   }

   
   public void hideAllGraphics()
   {
      enableAll = false;
      actionPerformed(null);
   }
   
   @Override
   public void actionPerformed(ActionEvent e)
   {
//      System.out.println(e.getActionCommand());

      if (enableAll)
      {
         for (AbstractButton button : buttons)
         {
            button.setSelected(true);
         }
      }
      else
      {
         for (AbstractButton button : buttons)
         {
            button.setSelected(false);
         }
      }
      enableAll = !enableAll;
      setText();
   }

}
