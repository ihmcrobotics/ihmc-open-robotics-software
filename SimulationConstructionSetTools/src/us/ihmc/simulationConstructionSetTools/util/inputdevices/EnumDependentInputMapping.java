package us.ihmc.simulationConstructionSetTools.util.inputdevices;

public interface EnumDependentInputMapping<T extends Enum<T>>
{
   public T getEnum();
}
