package us.ihmc.behaviors.tools.yo;

import java.util.function.DoubleSupplier;

public interface YoVariableClientPublishSubscribeAPI
{
   public DoubleSupplier subscribeToYoVariableDoubleValue(String variableName);

   public void publishDoubleValueToYoVariable(String variableName, double value);

   public YoBooleanClientHelper subscribeToYoBoolean(String variableName);

   public YoDoubleClientHelper subscribeToYoDouble(String variableName);
}
