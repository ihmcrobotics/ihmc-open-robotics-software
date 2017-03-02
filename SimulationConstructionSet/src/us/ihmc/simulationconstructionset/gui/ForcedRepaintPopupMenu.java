package us.ihmc.simulationconstructionset.gui;

import javax.swing.JPopupMenu;

@SuppressWarnings("serial")
public class ForcedRepaintPopupMenu extends JPopupMenu
{
   public ForcedRepaintPopupMenu()
   {
      super();
   }

   public ForcedRepaintPopupMenu(String label)
   {
      super(label);
   }

   @Override
   public void setVisible(boolean setVisible)
   {
      super.setVisible(setVisible);
      repaint();
   }
}
