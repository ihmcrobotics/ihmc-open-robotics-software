package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.text.NumberFormat;

import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommandAndMotionConstraint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

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
      
      desiredAcceleration.set(desiredJointAccelerationCommand.getDesiredAcceleration());
      
      achievedJointAcceleration.set(desiredJointAccelerationCommandAndMotionConstraint.getAchievedJointAcceleration());
      
      errorAcceleration.set(desiredAcceleration);
      CommonOps.subtract(achievedJointAcceleration, desiredAcceleration, errorAcceleration);
      
      this.repaint();
   }

   public String getJointName()
   {
      return joint.getName();
   }
   
   public String getDesiredAccelerationAsString()
   {
      return toPrettyString(numberFormat, desiredAcceleration);
   }
   
   public String getAchievedJointAccelerationAsString()
   {
      return toPrettyString(numberFormat, achievedJointAcceleration);
   }
   
   public String getErrorAccelerationAsString()
   {
      return toPrettyString(numberFormat, errorAcceleration);
   }
   
   
   public static String toPrettyString(NumberFormat numberFormat, DenseMatrix64F columnMatrix)
   {
      String ret = " ";
      
      int numRows = columnMatrix.getNumRows();
      for (int i=0; i<numRows; i++)
      {
         ret = ret + numberFormat.format(columnMatrix.get(i, 0));
         if (i < numRows - 1) ret = ret + ", ";
      }
      
      ret = ret + "";
      return ret;
   }
}
