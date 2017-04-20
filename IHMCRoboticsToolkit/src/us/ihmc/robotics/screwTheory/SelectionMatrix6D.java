package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SelectionMatrix6D
{
   private final SelectionMatrix3D angularPart = new SelectionMatrix3D();
   private final SelectionMatrix3D linearPart = new SelectionMatrix3D();

   public SelectionMatrix6D()
   {
   }

   public void resetSelection()
   {
      angularPart.resetSelection();
      linearPart.resetSelection();
   }

   public void setToAngularSelection()
   {
      angularPart.resetSelection();
      linearPart.clearSelection();
   }

   public void setToLinearSelection()
   {
      angularPart.clearSelection();
      linearPart.resetSelection();
   }

   public void clearSelectionFrame()
   {
      setSelectionFrame(null);
   }

   public void clearAngularSelectionFrame()
   {
      angularPart.clearSelectionFrame();
   }

   public void clearLinearSelectionFrame()
   {
      linearPart.clearSelectionFrame();
   }

   public void setSelectionFrame(ReferenceFrame selectionFrame)
   {
      setSelectionFrames(selectionFrame, selectionFrame);
   }

   public void setSelectionFrames(ReferenceFrame angularSelectionFrame, ReferenceFrame linearSelectionFrame)
   {
      angularPart.setSelectionFrame(angularSelectionFrame);
      linearPart.setSelectionFrame(linearSelectionFrame);
   }

   public void setAngularAxisSelection(boolean xSelected, boolean ySelected, boolean zSelected)
   {
      selectAngularX(xSelected);
      selectAngularY(ySelected);
      selectAngularZ(zSelected);
   }

   public void selectAngularX(boolean select)
   {
      angularPart.selectXAxis(select);
   }

   public void selectAngularY(boolean select)
   {
      angularPart.selectYAxis(select);
   }

   public void selectAngularZ(boolean select)
   {
      angularPart.selectZAxis(select);
   }
   
   public void setLinearAxisSelection(boolean xSelected, boolean ySelected, boolean zSelected)
   {
      selectLinearX(xSelected);
      selectLinearY(ySelected);
      selectLinearZ(zSelected);
   }

   public void selectLinearX(boolean select)
   {
      linearPart.selectXAxis(select);
   }

   public void selectLinearY(boolean select)
   {
      linearPart.selectYAxis(select);
   }

   public void selectLinearZ(boolean select)
   {
      linearPart.selectZAxis(select);
   }

   public void getFullSelectionMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrixToPack.reshape(6, 6);
      selectionMatrixToPack.zero();

      angularPart.getFullSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
      linearPart.getFullSelectionMatrixInFrame(destinationFrame, 3, 3, selectionMatrixToPack);
   }

   public void getEfficientSelectionMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrixToPack.reshape(6, 6);
      selectionMatrixToPack.zero();

      // Need to do the linear part first as rows might be removed.
      linearPart.getEfficientSelectionMatrixInFrame(destinationFrame, 3, 3, selectionMatrixToPack);
      angularPart.getEfficientSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
   }

   public ReferenceFrame getAngularSelectionFrame()
   {
      return angularPart.getSelectionFrame();
   }

   public boolean isAngularXSelected()
   {
      return angularPart.isXSelected();
   }
   
   public boolean isAngularYSelected()
   {
      return angularPart.isYSelected();
   }
   
   public boolean isAngularZSelected()
   {
      return angularPart.isZSelected();
   }
   
   public ReferenceFrame getLinearSelectionFrame()
   {
      return linearPart.getSelectionFrame();
   }
   
   public boolean isLinearXSelected()
   {
      return linearPart.isXSelected();
   }
   
   public boolean isLinearYSelected()
   {
      return linearPart.isYSelected();
   }
   
   public boolean isLinearZSelected()
   {
      return linearPart.isZSelected();
   }
}
