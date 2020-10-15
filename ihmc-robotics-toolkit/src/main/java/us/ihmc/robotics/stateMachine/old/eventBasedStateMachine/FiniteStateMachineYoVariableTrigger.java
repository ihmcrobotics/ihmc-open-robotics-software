package us.ihmc.robotics.stateMachine.old.eventBasedStateMachine;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Provides a wrapper allowing state machine events to be triggered by a changing YoEnum.
 *
 * @param <E> the state machine event type.
 */
@Deprecated
public class FiniteStateMachineYoVariableTrigger<E extends Enum<E>>
{
   public FiniteStateMachineYoVariableTrigger(final FiniteStateMachine<?, ?, ?> stateMachine, String name, YoRegistry registry,
         final Class<E> enumType)
   {
      final YoEnum<E> yoVariable = new YoEnum<>(name, registry, enumType, true);
      yoVariable.set(null);

      // Attach a change listener, firing state machine events for every callback.
      yoVariable.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
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
