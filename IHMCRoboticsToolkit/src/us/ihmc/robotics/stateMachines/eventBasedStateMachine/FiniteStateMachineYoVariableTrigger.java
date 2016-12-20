package us.ihmc.robotics.stateMachines.eventBasedStateMachine;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * Provides a wrapper allowing state machine events to be triggered by a changing EnumYoVariable.
 *
 * @param <E> the state machine event type.
 */
public class FiniteStateMachineYoVariableTrigger<E extends Enum<E>>
{
   public FiniteStateMachineYoVariableTrigger(final FiniteStateMachine<?, ?> stateMachine, String name, YoVariableRegistry registry,
         final Class<E> enumType)
   {
      final EnumYoVariable<E> yoVariable = new EnumYoVariable<>(name, registry, enumType, true);
      yoVariable.set(null);

      // Attach a change listener, firing state machine events for every callback.
      yoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (yoVariable.getEnumValue() != null)
            {
               stateMachine.trigger(enumType, yoVariable.getEnumValue());

               // Reset to null to be ready for another event.
               yoVariable.set(null);
            }
         }
      });
   }
}
