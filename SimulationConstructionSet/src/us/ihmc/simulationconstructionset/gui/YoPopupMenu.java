package us.ihmc.simulationconstructionset.gui;

import javax.swing.*;

public class YoPopupMenu extends JPopupMenu
{
   public YoPopupMenu()
   {
      super();
   }

   public YoPopupMenu(String label)
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
