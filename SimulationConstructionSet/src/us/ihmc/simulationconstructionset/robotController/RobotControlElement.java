package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public interface RobotControlElement
{
   public abstract void initialize();
   public abstract YoVariableRegistry getYoVariableRegistry();
   public abstract String getName();
   public abstract String getDescription();
}
