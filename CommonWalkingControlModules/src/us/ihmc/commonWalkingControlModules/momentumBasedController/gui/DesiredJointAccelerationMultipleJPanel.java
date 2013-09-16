package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Graphics;
import java.text.NumberFormat;
import java.util.ArrayList;

import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;

import org.ejml.data.DenseMatrix64F;
import org.omg.CORBA.INITIALIZE;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommandAndMotionConstraint;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;

public class DesiredJointAccelerationMultipleJPanel extends AbstractMultipleReusableJPanel<DesiredJointAccelerationJPanel>
{
   private static final long serialVersionUID = -4892971095902174475L;

   private DefaultTableModel model;
   private JTable jTable;

   private TableTools tableTools;

   public DesiredJointAccelerationMultipleJPanel()
   {
      tableTools = new TableTools();
      ArrayList<String> header = new ArrayList<String>();
      header.add("Joint Name");
      header.add("Desired Joint Accaleration");
      header.add("Achieved Joint Acceleration");
      header.add("Error");
      jTable = tableTools.createJTableModelForDesiredJointAcceleration(header, 14); //TODO: make table size dynamic
      model = tableTools.getModel();

      this.repaint();
   }
   
   public synchronized void setDesiredJointAccelerations(
         ArrayList<DesiredJointAccelerationCommandAndMotionConstraint> desiredJointAccelerationCommandAndMotionConstraints)
   {
      setTableSize(desiredJointAccelerationCommandAndMotionConstraints.size());
      for (int i = 0; i < desiredJointAccelerationCommandAndMotionConstraints.size(); i++)
      {
         DesiredJointAccelerationCommandAndMotionConstraint desiredJointAccelerationCommandAndMotionConstraint = desiredJointAccelerationCommandAndMotionConstraints
               .get(i);

         DesiredJointAccelerationJPanel desiredJointAccelerationJPanel = new DesiredJointAccelerationJPanel();
         desiredJointAccelerationJPanel.setDesiredJointAccelerationCommand(desiredJointAccelerationCommandAndMotionConstraint);
         writeDataToJointTable(i, desiredJointAccelerationJPanel);
      }

   }
   private void setTableSize(int dataSize)
   {
      if(dataSize > jTable.getRowCount())
      {
         throw new RuntimeException("TableSize is not right. Please change the size to: " + dataSize + " instead of: " + jTable.getRowCount());
      }
   }
   
   private void writeDataToJointTable(int rowCount, DesiredJointAccelerationJPanel desiredJointAccelerationJPanel)
   {
      String prefix = " ";
      model.setValueAt(prefix + desiredJointAccelerationJPanel.getJointName(), rowCount + 1, 0);
      model.setValueAt(prefix + desiredJointAccelerationJPanel.getDesiredAcceleration(), rowCount + 1, 1);
      model.setValueAt(prefix + desiredJointAccelerationJPanel.getAchievedJointAcceleration(), rowCount + 1, 2);
      model.setValueAt(prefix + desiredJointAccelerationJPanel.getErrorAcceleration(), rowCount + 1, 3);
   }

   public DesiredJointAccelerationJPanel constructNewJPanel()
   {
      return new DesiredJointAccelerationJPanel();
   }

   public synchronized void paintComponent(Graphics g)
   {
      super.paintComponent(g);
      add(jTable);
   }

}
