package us.ihmc.simulationconstructionset.util.environments;


public interface SelectableObject
{
   public void select();
   public void unSelect(boolean reset);
   public void addSelectedListeners(SelectableObjectListener selectedListener);
}
