package us.ihmc.simulationconstructionset.gui;

import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;
import javafx.event.EventHandler;
import javax.swing.*;

/**
 * <p>SCSAction is used to implement both the AbstractAction class required for Swing
 * GUI as well as the EventHandler interface required for JavaFX GUI.</p>
 *
 * <p>All actions should be extensions of the SCSAction class. "" should be used for
 * an action without an icon image and KeyEvent.VK_UNDEFINED should be used for
 * an action without a shortcut key.</p>
 *
 * @author Carson Wilber
 */
@SuppressWarnings("serial")
abstract public class SCSAction extends AbstractAction implements EventHandler<javafx.event.ActionEvent> {
    String iconFilename;

    public SCSAction(String pass, String iconFilename, int shortKey, String shortDescription, String longDescription) {
        super(pass);

        this.iconFilename = iconFilename;

        if (!iconFilename.equals("")) {
            AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
        } else {
            this.putValue(Action.MNEMONIC_KEY, shortKey);
            this.putValue(Action.LONG_DESCRIPTION, longDescription);
            this.putValue(Action.SHORT_DESCRIPTION, shortDescription);
        }
    }

    @Override
    public void actionPerformed(java.awt.event.ActionEvent actionEvent) {
        doAction();
    }

    @Override
    public void handle(javafx.event.ActionEvent actionEvent) {
        doAction();
    }

    abstract public void doAction();
}
