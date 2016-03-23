package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.List;

public interface MultipleControllerCommandHolder
{
   public abstract List<ControllerCommand<?, ?>> getControllerCommands();
}
