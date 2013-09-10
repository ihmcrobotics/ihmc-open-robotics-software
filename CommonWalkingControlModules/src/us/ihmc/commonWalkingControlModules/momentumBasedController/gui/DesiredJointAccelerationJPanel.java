package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Graphics;
import java.text.NumberFormat;

import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;

public class DesiredJointAccelerationJPanel extends JPanel
{
   private static final long serialVersionUID = -7632993267486553349L;
   private DesiredJointAccelerationCommand desiredJointAccelerationCommand;
   
   private InverseDynamicsJoint joint;
   
   private DenseMatrix64F desiredAcceleration = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F achievedAcceleration = new DenseMatrix64F(1);
   private final DenseMatrix64F errorAcceleration = new DenseMatrix64F(1);
   
   private final NumberFormat numberFormat;

   public DesiredJointAccelerationJPanel()
   {
      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(5);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);
   }
   
   public void setDesiredJointAccelerationCommand(DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      this.desiredJointAccelerationCommand = desiredJointAccelerationCommand;
      
      joint = desiredJointAccelerationCommand.getJoint();
      desiredAcceleration.setReshape(desiredJointAccelerationCommand.getDesiredAcceleration());
      
      desiredJointAccelerationCommand.computeAchievedJointAcceleration(achievedAcceleration);
      
      errorAcceleration.setReshape(desiredAcceleration);
      CommonOps.sub(achievedAcceleration, desiredAcceleration, errorAcceleration);
      
      this.repaint();
   }

   public void paintComponent(Graphics graphics)
   {
      if (joint == null) 
         graphics.drawString("Empty", 40, 12); 

      else
      {
         graphics.drawString(joint.getName(), 10, 12); 
         graphics.drawString(toPrettyString(desiredAcceleration), 100, 12); 
         graphics.drawString(toPrettyString(achievedAcceleration), 200, 12); 
         
         graphics.drawString(toPrettyString(errorAcceleration), 300, 12); 
      }
   }
   
   public String toPrettyString(DenseMatrix64F columnMatrix)
   {
      String ret = "(";
      
      int numRows = columnMatrix.getNumRows();
      for (int i=0; i<numRows; i++)
      {
         ret = ret + numberFormat.format(columnMatrix.get(i, 0));
         if (i < numRows - 1) ret = ret + ", ";
      }
      
      ret = ret + ")";
      return ret;
   }
}
