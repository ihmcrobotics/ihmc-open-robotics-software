package us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine;

@Deprecated
public interface PreviousStateProvider<E extends Enum<E>, T extends State<E>>
{
   public abstract T getPreviousState();
}
