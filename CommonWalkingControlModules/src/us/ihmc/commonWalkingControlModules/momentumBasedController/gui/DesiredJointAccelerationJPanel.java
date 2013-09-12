package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Graphics;
import java.text.NumberFormat;

import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommandAndMotionConstraint;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;

public class DesiredJointAccelerationJPanel extends JPanel
{
   private static final long serialVersionUID = -7632993267486553349L;
   
   private InverseDynamicsJoint joint;
   
   private DenseMatrix64F desiredAcceleration = new DenseMatrix64F(1, 1);
   private DenseMatrix64F achievedJointAcceleration = new DenseMatrix64F(1, 1);
   
   private final DenseMatrix64F errorAcceleration = new DenseMatrix64F(1);
   
   private final NumberFormat numberFormat;

   public DesiredJointAccelerationJPanel()
   {
      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(5);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);
   }
   
   public synchronized void setDesiredJointAccelerationCommand(DesiredJointAccelerationCommandAndMotionConstraint desiredJointAccelerationCommandAndMotionConstraint)
   {      
      DesiredJointAccelerationCommand desiredJointAccelerationCommand = desiredJointAccelerationCommandAndMotionConstraint.getDesiredJointAccelerationCommand();
      
      joint = desiredJointAccelerationCommand.getJoint();
      desiredAcceleration.setReshape(desiredJointAccelerationCommand.getDesiredAcceleration());
      
      achievedJointAcceleration.setReshape(desiredJointAccelerationCommandAndMotionConstraint.getAchievedJointAcceleration());
      
      errorAcceleration.setReshape(desiredAcceleration);
      CommonOps.sub(achievedJointAcceleration, desiredAcceleration, errorAcceleration);
      
      this.repaint();
   }

   public synchronized void paintComponent(Graphics graphics)
   {
      if (joint == null) 
         graphics.drawString("Empty", 40, 12); 

      else
      {
         graphics.drawString(joint.getName(), 10, 12); 
         graphics.drawString(toPrettyString(numberFormat, desiredAcceleration), 100, 12); 
         graphics.drawString(toPrettyString(numberFormat, achievedJointAcceleration), 200, 12); 
         
         graphics.drawString(toPrettyString(numberFormat, errorAcceleration), 300, 12); 
      }
   }
   
   public static String toPrettyString(NumberFormat numberFormat, DenseMatrix64F columnMatrix)
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
