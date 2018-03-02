package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.flight.InertiaTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * This command specifies a whole body rotational inertia rate objective for the robot. Since the inertia matrix is
 * symmetric 3x3 matrix and has only 6 independent entries, this command is reorganized into a 6x1 objective. The 
 * weights are 
 * 
 * @author Apoorv Shrivastava
 */
public class WholeBodyInertiaCommand implements InverseDynamicsCommand<WholeBodyInertiaCommand>
{
   /**
    * Frame in which the inertia rate of change is defined
    */
   private ReferenceFrame controlFrame;
   /**
    * The required 3x3 rate of change of inertia 
    */
   private DenseMatrix64F desiredInertiaRateOfChange;

   /**
    * The 
    */
   private InertiaWeightMatrix weightMatrix;
   private InertiaSelectionMatrix selectionMatrix;

   public WholeBodyInertiaCommand()
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void set(WholeBodyInertiaCommand other)
   {
      this.controlFrame = other.controlFrame;
      this.desiredInertiaRateOfChange.set(other.desiredInertiaRateOfChange);
      this.weightMatrix.set(other.weightMatrix);
      this.selectionMatrix.set(other.selectionMatrix);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.WHOLE_BODY_INERTIA_COMMAND;
   }

   public void setControlFrame(ReferenceFrame controlFrame)
   {
      this.controlFrame = controlFrame;
   }

   public void getWeightMatrix(ReferenceFrame destinationFrame, DenseMatrix64F weightMatrixToPack)
   {
      weightMatrix.getWeightMatrix(destinationFrame, weightMatrixToPack);
   }

   public void getSelectionMatrix(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrix.getSelectionMatrix(destinationFrame, selectionMatrixToPack);
   }

   public void getDesiredInertiaRateOfChange(DenseMatrix64F inertiaRateOfChangeToPack)
   {
      inertiaRateOfChangeToPack.set(desiredInertiaRateOfChange);
   }

   public void setWeights(double scalar)
   {
      weightMatrix.setWeights(scalar);
   }

   public void selectAll()
   {
      selectionMatrix.setSelection(true, true, true, true, true, true);
   }

   private class InertiaWeightMatrix
   {
      private ReferenceFrame weightFrame;
      private double[] weights = new double[InertiaTools.inertiaComponentsSize];

      public InertiaWeightMatrix()
      {
         clear();
      }

      public void set(InertiaWeightMatrix other)
      {
         this.weightFrame = other.weightFrame;
         for (int i = 0; i < InertiaTools.inertiaComponentsSize; i++)
            this.weights[i] = other.weights[i];
      }

      public void clear()
      {
         weightFrame = null;
         for (int i = 0; i < weights.length; i++)
            weights[i] = Double.NaN;
      }

      public void setWeights(double xxWeight, double yyWeight, double zzWeight, double xyWeight, double yzWeight, double zxWeight)
      {
         this.weights[0] = xxWeight;
         this.weights[1] = yyWeight;
         this.weights[2] = zzWeight;
         this.weights[3] = xyWeight;
         this.weights[4] = yzWeight;
         this.weights[5] = zxWeight;
      }

      public void setWeights(double scalar)
      {
         for (int i = 0; i < weights.length; i++)
            weights[i] = scalar;
      }

      public void getWeightMatrix(ReferenceFrame destinationFrame, DenseMatrix64F weightMatrixToPack)
      {
         getWeightMatrix(destinationFrame, 0, 0, weightMatrixToPack);
      }

      public void getWeightMatrix(ReferenceFrame destinationFrame, int startRow, int startColumn, DenseMatrix64F weightMatrixToPack)
      {
         //FIXME hacking for now since using the same frame on both sides.
         for (int i = startRow; i < startRow + InertiaTools.inertiaComponentsSize; i++)
            for (int j = startColumn; j < startColumn + InertiaTools.inertiaComponentsSize; j++)
               weightMatrixToPack.set(i, j, 0.0);

         for (int i = 0; i < InertiaTools.inertiaComponentsSize; i++)
            weightMatrixToPack.set(i + startRow, i + startColumn, weights[i]);
      }
   }

   /**
    * Custom selection matrix for the inertia tensor
    * Stores the selection variables as the 6 x 1 tuple corresponding to the six unique elements of the inertia tensor
    * (Ixx, Iyy, Izz, Ixy, Iyz, Izx)
    * 
    * @author Apoorv Shrivastava
    */
   private class InertiaSelectionMatrix
   {
      private ReferenceFrame selectionFrame;
      private boolean[] selection = new boolean[InertiaTools.inertiaComponentsSize];

      public InertiaSelectionMatrix()
      {
         clear();
      }

      public void set(InertiaSelectionMatrix other)
      {
         this.selectionFrame = other.selectionFrame;
         for (int i = 0; i < InertiaTools.inertiaComponentsSize; i++)
            this.selection[i] = other.selection[i];
      }

      public void clear()
      {
         selectionFrame = null;
         for (int i = 0; i < InertiaTools.inertiaComponentsSize; i++)
            selection[i] = false;
      }

      public void setSelection(boolean xxSelected, boolean yySelected, boolean zzSelected, boolean xySelected, boolean yzSelected, boolean zxSelected)
      {
         this.selection[0] = xxSelected;
         this.selection[1] = yySelected;
         this.selection[2] = zzSelected;
         this.selection[3] = xySelected;
         this.selection[4] = yzSelected;
         this.selection[5] = zxSelected;
      }

      public void getSelectionMatrix(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
      {
         getSelectionMatrix(destinationFrame, 0, 0, selectionMatrixToPack);
      }

      public void getSelectionMatrix(ReferenceFrame destinationFrame, int startRow, int startColumn, DenseMatrix64F selectionMatrixToPack)
      {
         //FIXME hacking for now since using the same frame on both sides.
         for (int i = startRow; i < startRow + InertiaTools.inertiaComponentsSize; i++)
            for (int j = startColumn; j < startColumn + InertiaTools.inertiaComponentsSize; j++)
               selectionMatrixToPack.set(i, j, 0.0);

         for (int i = 0; i < InertiaTools.inertiaComponentsSize; i++)
            selectionMatrixToPack.set(i + startRow, i + startColumn, selection[i]);
      }
   }
}