package us.ihmc.simulationconstructionset.gui;

import java.awt.dnd.DropTargetDragEvent;
import java.awt.dnd.DropTargetDropEvent;
import java.awt.dnd.DropTargetEvent;
import java.awt.dnd.DropTargetListener;

import javax.swing.TransferHandler;

public class YoGraphTargetListener implements DropTargetListener
{
   private YoGraph yoGraph;

   public YoGraphTargetListener(YoGraph yoGraph)
   {
      this.yoGraph = yoGraph;
   }

   public void dragEnter(DropTargetDragEvent dtde)
   {
   }

   public void dragExit(DropTargetEvent dte)
   {

   }

   public void dragOver(DropTargetDragEvent dtde)
   {
      YoGraph.setRecipientOfDrag(yoGraph);

      if (dtde.getSource().equals(yoGraph))
      {
         dtde.rejectDrag();
      }
   }

   public void drop(DropTargetDropEvent dtde)
   {
      YoGraph.setRecipientOfDrag(yoGraph);

      if ((YoGraph.getSourceOfDrag() == null) ||!YoGraph.getSourceOfDrag().equals(yoGraph))
      {
         yoGraph.addVariableFromSelectedVariableHolder();
      }
      else
      {
         YoGraph.setActionPerformedByDragAndDrop(TransferHandler.NONE);
         dtde.rejectDrop();
      }
   }

   public void dropActionChanged(DropTargetDragEvent dtde)
   {
      YoGraph.setActionPerformedByDragAndDrop(dtde.getDropAction());
   }

}
