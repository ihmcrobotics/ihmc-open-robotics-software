package us.ihmc.simulationconstructionset.commands;

public interface ToggleKeyPointModeCommandExecutor
{
   public abstract boolean isKeyPointModeToggled();
   public abstract void toggleKeyPointMode();
   
   public abstract void registerToggleKeyPointModeCommandListener(ToggleKeyPointModeCommandListener commandListener);
   public abstract void closeAndDispose();

}
