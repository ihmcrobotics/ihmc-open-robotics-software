package us.ihmc.simulationconstructionsettools.util.inputdevices;

public interface EnumDependentInputMapping<T extends Enum<T>>
{
   public T getEnum();
}
