package us.ihmc.simulationconstructionset.util.inputdevices;

public interface EnumDependentInputMapping<T extends Enum<T>>
{
   public T getEnum();
}
