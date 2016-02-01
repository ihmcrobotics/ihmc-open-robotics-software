package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.text.NumberFormat;

import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommandAndMotionConstraint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class DesiredSpatialAccelerationJPanel extends JPanel
{
   private static final long serialVersionUID = -7632993267486553349L;
   
   private SpatialAccelerationVector desiredSpatialAccelerationVector = new SpatialAccelerationVector();
   
   private String bodyName, baseName;
   private DenseMatrix64F desiredSpatialAcceleration = new DenseMatrix64F(6, 1);
   private DenseMatrix64F desiredSpatialAcceleration2 = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F achievedSpatialAcceleration = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F errorSpatialAcceleration = new DenseMatrix64F(6, 1);
   
   private final NumberFormat numberFormat;

   public DesiredSpatialAccelerationJPanel()
   {
      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(3);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);
   }
   
   public synchronized void setDesiredSpatialAccelerationCommand(DesiredSpatialAccelerationCommandAndMotionConstraint desiredSpatialAccelerationCommandAndMotionConstraint)
   {
      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = desiredSpatialAccelerationCommandAndMotionConstraint.getDesiredSpatialAccelerationCommand();
      
      TaskspaceConstraintData taskspaceConstraintData = desiredSpatialAccelerationCommand.getTaskspaceConstraintData();
      desiredSpatialAccelerationVector.set(taskspaceConstraintData.getSpatialAcceleration());
      
      bodyName = desiredSpatialAccelerationVector.getBodyFrame().getName();
      baseName = desiredSpatialAccelerationVector.getBaseFrame().getName();
      
      desiredSpatialAccelerationVector.packMatrix(desiredSpatialAcceleration, 0);
      
      desiredSpatialAcceleration2.set(desiredSpatialAccelerationCommandAndMotionConstraint.getDesiredSpatialAcceleration());
      achievedSpatialAcceleration.set(desiredSpatialAccelerationCommandAndMotionConstraint.getAchievedSpatialAcceleration());
      
      errorSpatialAcceleration.set(desiredSpatialAcceleration2);
      CommonOps.subtract(achievedSpatialAcceleration, desiredSpatialAcceleration2, errorSpatialAcceleration);
      
      this.repaint();
   }

   public String getBodyName()
   {
      return bodyName;
   }
   public String getBaseName()
   {
      return baseName;
   }
   public String getDesiredSpatialAcceleration()
   {
      return toPrettyString(desiredSpatialAcceleration);
   }
   public String getDesiredSpatialAcceleration2()
   {
      return toPrettyString(desiredSpatialAcceleration2);
   }
   public String getAchievedSpatialAcceleration()
   {
      return toPrettyString(achievedSpatialAcceleration);
   }
   public String getErrorSpatialAcceleration()
   {
      return toPrettyString(errorSpatialAcceleration);
   }
   
   
   public String toPrettyString(DenseMatrix64F columnMatrix)
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

