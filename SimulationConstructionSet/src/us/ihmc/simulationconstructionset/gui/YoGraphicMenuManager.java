package us.ihmc.simulationconstructionset.gui;

import javax.swing.JMenu;
import javax.swing.JMenuItem;

import us.ihmc.simulationconstructionset.gui.tools.MultiButtonSelectionDeselectionAction;

public class YoGraphicMenuManager
{
   private JMenu jMenu;
   private MultiButtonSelectionDeselectionAction hideAllGraphicObjects;

   public YoGraphicMenuManager()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            jMenu = new JMenu("Graphics");
            hideAllGraphicObjects = new MultiButtonSelectionDeselectionAction("Hide All");
            JMenuItem hideAllButton = new JMenuItem(hideAllGraphicObjects);
            hideAllGraphicObjects.setShowHideAllMenuItem(hideAllButton, "Show All", "Hide All");
            jMenu.add(hideAllButton);
         }
      });
   }

   public JMenu getjMenu()
   {
      return jMenu;
   }

   public void setjMenu(JMenu jMenu)
   {
      this.jMenu = jMenu;
   }
   
   public void hideAllGraphics()
   {
      hideAllGraphicObjects.hideAllGraphics();
   }

   public void addCheckBox(YoGraphicCheckBoxMenuItem checkBox)
   {
      jMenu.add(checkBox);
      hideAllGraphicObjects.addButton(checkBox);
   }

}
