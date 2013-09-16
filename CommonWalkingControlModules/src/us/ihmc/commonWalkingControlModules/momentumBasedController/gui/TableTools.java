package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Color;
import java.util.ArrayList;

import javax.swing.JTable;
import javax.swing.UIManager;
import javax.swing.border.MatteBorder;
import javax.swing.table.DefaultTableModel;


public class TableTools
{
   private final DefaultTableModel model;
   private final JTable jTable;

   
   public TableTools()
   {
      model = new DefaultTableModel();
      jTable = new JTable(model);
   }
   
   public JTable createJTableModelForDesiredJointAcceleration(ArrayList<String> columnHeader, int rowCount)
   {
      model.setColumnCount(columnHeader.size());
      model.setRowCount(rowCount + 1);
      String prefix = " ";
      for(int i = 0; i<columnHeader.size() ; i++)
      {
         model.setValueAt(prefix + columnHeader.get(i), 0, i);
         jTable.getColumnModel().getColumn(i).setPreferredWidth(1350/columnHeader.size());
      }
      
      
      Color color = UIManager.getColor("Table.gridColor");
      MatteBorder border = new MatteBorder(2, 2, 2, 2, color);
      
      jTable.setBorder(border);
      return jTable;
   }
   
   public DefaultTableModel getModel()
   {
      return model;
   }
   
   public void setPreferredWidth(int columnNumber, int prefferedWidth)
   {
         jTable.getColumnModel().getColumn(columnNumber).setPreferredWidth(prefferedWidth);
   }
   
   public void setRowCount(int rowCount)
   {
      model.setRowCount(rowCount);
   }
   public void addRows(int addRows)
   {
      model.setRowCount(model.getRowCount() + addRows);
   }
   
    
   
  

}
