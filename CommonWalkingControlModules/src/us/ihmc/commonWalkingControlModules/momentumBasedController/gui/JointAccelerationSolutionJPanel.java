package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Graphics;
import java.text.NumberFormat;
import java.util.ArrayList;

import javax.swing.JPanel;
import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

public class JointAccelerationSolutionJPanel extends JPanel
{
   private static final long serialVersionUID = -7632993267486553349L;

   private final NumberFormat numberFormat;

   private ArrayList<InverseDynamicsJoint> jointsToOptimizeFor = new ArrayList<InverseDynamicsJoint>();
   private final TableTools tableTools;
   private final DefaultTableModel model;
   private final JTable jointTable;

   public JointAccelerationSolutionJPanel()
   {
      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(3);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);
      this.tableTools = new TableTools();
      ArrayList<String> columnHeader = new ArrayList<String>();
      columnHeader.add("JointName");
      columnHeader.add("Acceleration");
      jointTable = tableTools.createJTableModelForDesiredJointAcceleration(columnHeader, 34);
      this.model = tableTools.getModel();
   }

   public synchronized void setJointAccelerationSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerationsSolution)
   {
      this.jointsToOptimizeFor.clear();

      int totalDof=0;
      if (jointsToOptimizeFor != null)
      {
         for (InverseDynamicsJoint jointToOptimizeFor : jointsToOptimizeFor)
         {
        	totalDof+=jointToOptimizeFor.getDegreesOfFreedom();
            this.jointsToOptimizeFor.add(jointToOptimizeFor);
         }
      }
      setTableSize(totalDof+1); //keep one row for header

      int index = 0;
      int counter = 0;
      for (int i = 0; i < this.jointsToOptimizeFor.size(); i++)
      {
         InverseDynamicsJoint joint = this.jointsToOptimizeFor.get(i);
         int numDofs = joint.getDegreesOfFreedom();

         for (int j = 0; j < numDofs; j++)
         {
            double acceleration = jointAccelerationsSolution.get(index + j, 0);
            writeJointInfoToTable(i + j + counter, joint.getName()+" - "+j, numberFormat.format(acceleration));
         }

         index += numDofs;
         counter += numDofs - 1;
      }
   }
   
   private void setTableSize(int newSize)
   {
	   if (newSize > jointTable.getRowCount())
	   {
		   tableTools.addRows(newSize-jointTable.getRowCount());
		   System.out.println(getClass().getSimpleName()+" bumpTableSize to "+newSize);
	   }
   }

   public synchronized void paintComponent(Graphics graphics)
   {
      add(jointTable);
   }

   public String toPrettyString(DenseMatrix64F columnMatrix)
   {
      String ret = "(";

      int numRows = columnMatrix.getNumRows();
      for (int i = 0; i < numRows; i++)
      {
         ret = ret + numberFormat.format(columnMatrix.get(i, 0));
         if (i < numRows - 1)
            ret = ret + ", ";
      }

      ret = ret + ")";
      return ret;
   }

   private void writeJointInfoToTable(int rowCount, String jointName, String JointData)
   {
      String prefix = " ";
      model.setValueAt(prefix + jointName, rowCount + 1, 0);
      model.setValueAt(prefix + JointData, rowCount + 1, 1);
   }

}
