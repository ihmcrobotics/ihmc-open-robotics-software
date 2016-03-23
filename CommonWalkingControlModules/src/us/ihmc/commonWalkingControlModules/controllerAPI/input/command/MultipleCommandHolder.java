package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.List;

public interface MultipleCommandHolder
{
   public abstract List<Command<?, ?>> getControllerCommands();
}
