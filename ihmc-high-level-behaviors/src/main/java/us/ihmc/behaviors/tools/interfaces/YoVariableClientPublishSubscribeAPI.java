package us.ihmc.behaviors.tools.interfaces;

import java.util.function.DoubleSupplier;

public interface YoVariableClientPublishSubscribeAPI
{
   public DoubleSupplier subscribeViaYoDouble(String variableName);

   public void publishDoubleToYoVariable(String variableName, double value);
}
