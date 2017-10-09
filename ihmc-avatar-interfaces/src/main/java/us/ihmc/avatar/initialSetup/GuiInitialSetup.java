package us.ihmc.avatar.initialSetup;

import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

/*
 * GUI Initial Setup
 * Implementations of this interface are designed to configure settings in SCS (the graphical interface).  
 * This includes cameras, buttons, VarGroups, slider boards and link graphics.
 */

public interface GuiInitialSetup
{
   public abstract void initializeGUI(SimulationConstructionSet scs, Robot robot);

   public abstract Graphics3DAdapter getGraphics3DAdapter();

   public abstract boolean isGuiShown();

   public abstract SimulationConstructionSetParameters getSimulationConstructionSetParameters();
}
