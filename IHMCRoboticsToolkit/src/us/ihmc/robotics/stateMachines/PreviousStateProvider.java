package us.ihmc.robotics.stateMachines;

public interface PreviousStateProvider<E extends Enum<E>, T extends State<E>>
{
   public abstract T getPreviousState();
}
