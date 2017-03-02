package us.ihmc.simulationconstructionset.gui;

import java.awt.event.ActionEvent;
import java.awt.event.FocusEvent;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public interface YoVariableEntryContainer {

	public void update(YoEntryBox yoEntryBox);

	public void setup(YoEntryBox yoEntryBox);
	
	public void shutdown(YoEntryBox yoEntryBox);

	public YoVariable<?> getVariable();

	public void actionPerformed(YoEntryBox yoEntryBox, ActionEvent evt);

	public void bindToVariable(YoEntryBox yoEntryBox, YoVariable<?> variable);

	public boolean isEventSource(YoEntryBox yoEntryBox, FocusEvent evt);

	public void focusGained(YoEntryBox yoEntryBox);

	public void focusLost(YoEntryBox yoEntryBox);

	public void removeVariable(YoVariable<?> variable);

}