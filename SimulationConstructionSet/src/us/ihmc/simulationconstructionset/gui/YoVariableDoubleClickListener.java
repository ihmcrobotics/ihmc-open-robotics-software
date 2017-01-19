package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import javax.swing.JFrame;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.simulationconstructionset.gui.dialogs.VarPropertiesDialog;

public class YoVariableDoubleClickListener implements DoubleClickListener
{
   private DataBuffer dataBuffer;
   private JFrame parentFrame;

   public YoVariableDoubleClickListener(DataBuffer dataBuffer, JFrame frame)
   {
      this.dataBuffer = dataBuffer;
      this.parentFrame = frame;

   }

   @Override
   public void doubleClicked(YoVariable<?> v)
   {
      DataBufferEntry entry = dataBuffer.getEntry(v);

      if (entry == null)
         return;

      ArrayList<DataBufferEntry> entries = new ArrayList<DataBufferEntry>();
      entries.add(entry);
      VarPropertiesDialog dialog = new VarPropertiesDialog(parentFrame, entries);
      dialog.pack();
      dialog.setVisible(true);
   }

}
