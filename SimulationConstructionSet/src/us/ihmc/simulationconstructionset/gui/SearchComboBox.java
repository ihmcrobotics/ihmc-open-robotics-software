package us.ihmc.simulationconstructionset.gui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JComboBox;

class SearchComboBox implements ActionListener
{
   @Override
   public void actionPerformed(ActionEvent e)
   {
      JComboBox comboBox = (JComboBox) e.getSource();
      String searchType = (String) comboBox.getSelectedItem();
      System.out.println("Search Type : " + searchType);
   }
}