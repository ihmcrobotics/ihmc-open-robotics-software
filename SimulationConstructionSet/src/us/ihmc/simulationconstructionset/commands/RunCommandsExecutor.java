package us.ihmc.simulationconstructionset.commands;

public interface RunCommandsExecutor extends PlayCommandExecutor, SimulateCommandExecutor, StopCommandExecutor
{
   public abstract void setPlaybackRealTimeRate(double realtimeRate);
   public abstract double getPlaybackRealTimeRate();
}
