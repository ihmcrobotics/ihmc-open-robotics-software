package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

public interface PreviousStateProvider<E extends Enum<E>, T extends State<E>>
{
   public abstract T getPreviousState();
}
