package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import javafx.application.Platform;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.simulationconstructionset.gui.DataBufferChangeListener;
import us.ihmc.simulationconstructionset.gui.dialogs.DataBufferPropertiesDialog;

import javax.swing.*;
import java.awt.*;

public class DataBufferPropertiesDialogGenerator implements DataBufferPropertiesDialogConstructor
{
   private DataBuffer dataBuffer;
   private Container parentContainer;
   private JFrame frame;
   private DataBufferChangeListener listener;

   public DataBufferPropertiesDialogGenerator(DataBuffer dataBuffer, Container parentContainer, JFrame frame, DataBufferChangeListener listener)
   {
      this.dataBuffer = dataBuffer;
      this.parentContainer = parentContainer;
      this.frame = frame;
      this.listener = listener;
   }

   @Override
   public void constructDialog()
   {
      Platform.runLater(() -> {
         new DataBufferPropertiesDialog(parentContainer, frame, dataBuffer, listener);
      });
   }

   @Override
   public void closeAndDispose()
   {
      dataBuffer = null;
      parentContainer = null;
      frame = null;
      listener = null;
   }
}
