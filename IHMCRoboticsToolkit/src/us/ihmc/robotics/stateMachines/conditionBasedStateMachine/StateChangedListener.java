package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

/**
 * <p>Title: SimulationConstructionSet</p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2000</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 */
public interface StateChangedListener <E extends Enum<E>>
{
   public abstract void stateChanged(State<E> oldState, State<E> newState, double time);
}
