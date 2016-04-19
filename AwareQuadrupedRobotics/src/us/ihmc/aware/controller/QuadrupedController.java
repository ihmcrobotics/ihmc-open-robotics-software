package us.ihmc.aware.controller;

import us.ihmc.aware.state.FiniteStateMachineState;

public interface QuadrupedController<T extends Enum<T>> extends FiniteStateMachineState<T>
{
}
