package us.ihmc.robotics.stateMachines;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;


public class StateMachine<E extends Enum<E>> extends GenericStateMachine<E, State<E>>
{
   public StateMachine(String name, String switchTimeName, Class<E> enumType, DoubleYoVariable timeVariable, YoVariableRegistry registry)
   {
      super(name, switchTimeName, enumType, timeVariable, registry);
   }

   public StateMachine(String name, String switchTimeName, Class<E> enumType, DoubleProvider timeProvider, YoVariableRegistry registry)
   {
      super(name, switchTimeName, enumType, timeProvider, registry);
   }
}
