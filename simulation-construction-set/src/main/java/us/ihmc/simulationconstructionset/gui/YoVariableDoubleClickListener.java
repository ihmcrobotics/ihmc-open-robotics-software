package us.ihmc.simulationconstructionset.gui;

import javafx.application.Platform;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntry;
import us.ihmc.simulationconstructionset.gui.dialogs.VarPropertiesDialog;

import javax.swing.*;
import java.util.ArrayList;

public class YoVariableDoubleClickListener implements DoubleClickListener {
    private DataBuffer dataBuffer;
    private JFrame parentFrame;

    public YoVariableDoubleClickListener(DataBuffer dataBuffer, JFrame frame) {
        this.dataBuffer = dataBuffer;
        this.parentFrame = frame;

    }

    @Override
    public void doubleClicked(YoVariable<?> v) {
        DataBufferEntry entry = dataBuffer.getEntry(v);

        if (entry == null)
            return;

        ArrayList<DataBufferEntry> entries = new ArrayList<DataBufferEntry>();
        entries.add(entry);

        Platform.runLater(() -> new VarPropertiesDialog(parentFrame, entries));
    }

}
