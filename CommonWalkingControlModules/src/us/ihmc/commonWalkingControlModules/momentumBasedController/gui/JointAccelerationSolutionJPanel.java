package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Graphics;
import java.text.NumberFormat;
import java.util.ArrayList;

import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;

public class JointAccelerationSolutionJPanel extends JPanel
{
   private static final long serialVersionUID = -7632993267486553349L;
      
   private final NumberFormat numberFormat;

   private final DenseMatrix64F jointAccelerationsSolution = new DenseMatrix64F(1, 1);
   private ArrayList<InverseDynamicsJoint> jointsToOptimizeFor = new ArrayList<InverseDynamicsJoint>(); 
   
   public JointAccelerationSolutionJPanel()
   {
      this.numberFormat = NumberFormat.getInstance();
      this.numberFormat.setMaximumFractionDigits(3);
      this.numberFormat.setMinimumFractionDigits(1);
      this.numberFormat.setGroupingUsed(false);
   }
   
   public synchronized void setJointAccelerationSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerationsSolution)
   {
      this.jointAccelerationsSolution.setReshape(jointAccelerationsSolution);
      this.jointsToOptimizeFor.clear(); 
      
       for (InverseDynamicsJoint jointToOptimizeFor : jointsToOptimizeFor)
       {
          this.jointsToOptimizeFor.add(jointToOptimizeFor);
       }
       
       this.repaint();
   }


   public synchronized void paintComponent(Graphics graphics)
   {
      int index = 0;
      int yPixels = 12;
      
      for (int i = 0; i<jointsToOptimizeFor.size(); i++)
      {
         int xPixels = 12;

         InverseDynamicsJoint joint = jointsToOptimizeFor.get(i);
         int numDofs = joint.getDegreesOfFreedom();

         graphics.drawString(joint.getName(), xPixels, yPixels);
         xPixels = xPixels + 140;
         
         for (int j=0; j<numDofs; j++)
         {
            double acceleration = jointAccelerationsSolution.get(index + j, 0);
            graphics.drawString(numberFormat.format(acceleration), xPixels, yPixels);
            xPixels = xPixels + 60;
         }
         
         yPixels = yPixels + 12;
         index = index + numDofs;
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


