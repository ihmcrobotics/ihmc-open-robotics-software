package us.ihmc.aware.controller;

import us.ihmc.aware.controller.position.QuadrupedPositionControllerEvent;
import us.ihmc.aware.state.StateMachineState;

public interface QuadrupedController<T extends Enum<T>> extends StateMachineState<T>
{
}
