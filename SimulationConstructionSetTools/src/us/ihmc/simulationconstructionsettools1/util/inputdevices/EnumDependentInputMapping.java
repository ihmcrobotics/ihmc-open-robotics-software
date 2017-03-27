package us.ihmc.simulationconstructionsettools1.util.inputdevices;

public interface EnumDependentInputMapping<T extends Enum<T>>
{
   public T getEnum();
}
