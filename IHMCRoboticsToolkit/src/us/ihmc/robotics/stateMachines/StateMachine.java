package us.ihmc.robotics.stateMachines;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class StateMachine<E extends Enum<E>> extends GenericStateMachine<E, State<E>>
{
   public StateMachine(String name, String switchTimeName, Class<E> enumType, DoubleYoVariable t, YoVariableRegistry registry)
   {
      super(name, switchTimeName, enumType, t, registry);
   }
}
