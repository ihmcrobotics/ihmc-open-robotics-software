package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Graphics;
import java.text.NumberFormat;

import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;

public class MotionConstraintJMatrixJPanel extends JPanel
{
   private static final long serialVersionUID = -7632993267486553349L;
      
   private final NumberFormat numberFormat;

   private final DenseMatrix64F motionConstraintJMatrix = new DenseMatrix64F(1, 1);
   
   public MotionConstraintJMatrixJPanel()
   {
      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(3);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);
   }
   
   public synchronized void setMotionConstraintJMatrix(DenseMatrix64F motionConstraintJMatrix)
   {
      this.motionConstraintJMatrix.set(motionConstraintJMatrix);
       this.repaint();
   }


   public synchronized void paintComponent(Graphics graphics)
   {
      int yPixels = 12;
      
      for (int i = 0; i<motionConstraintJMatrix.getNumRows(); i++)
      {
         int xPixels = 12;
         
         for (int j=0; j<motionConstraintJMatrix.getNumCols(); j++)
         {
            double motionConstraintJMatrixEntry = motionConstraintJMatrix.get(i, j);
            graphics.drawString(numberFormat.format(motionConstraintJMatrixEntry), xPixels, yPixels);
            xPixels = xPixels + 40;
         }
         
         yPixels = yPixels + 12;
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


