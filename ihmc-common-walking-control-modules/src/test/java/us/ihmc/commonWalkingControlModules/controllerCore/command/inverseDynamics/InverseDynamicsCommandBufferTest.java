package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;
import org.reflections.Reflections;

import com.google.common.base.CaseFormat;

import us.ihmc.commons.lists.RecyclingArrayList;

public class InverseDynamicsCommandBufferTest
{
   public static final String CONTROLLER_CORE_COMMANDS_PACKAGE = "us.ihmc.commonWalkingControlModules.controllerCore.command";

   @SuppressWarnings("rawtypes")
   @Test
   public void testCommandBufferCompleteness()
         throws IllegalArgumentException, IllegalAccessException, NoSuchFieldException, SecurityException, InvocationTargetException
   {
      InverseDynamicsCommandBuffer instance = new InverseDynamicsCommandBuffer();
      int expectedSize = 0;
      Field[] fields = InverseDynamicsCommandBuffer.class.getDeclaredFields();
      Map<String, Field> nameToFieldMap = Stream.of(fields).collect(Collectors.toMap(Field::getName, Function.identity()));
      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseDynamicsCommand>> commandTypes = reflections.getSubTypesOf(InverseDynamicsCommand.class);
      commandTypes.remove(InverseDynamicsCommandList.class);
      commandTypes.remove(InverseDynamicsCommandBuffer.class);

      String errorMessage = "";

      for (Class<? extends InverseDynamicsCommand> commandType : commandTypes)
      {
         if (commandType.isInterface())
            continue;

         String expectedFieldName = CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_CAMEL, commandType.getSimpleName()) + "Buffer";
         Field field = nameToFieldMap.get(expectedFieldName);

         if (field == null)
         {
            errorMessage += "Missing buffer for " + commandType.getSimpleName() + ".\n";
            continue;
         }

         // We assert that buffers use RecyclingArrayList.

         if (field.getType() != RecyclingArrayList.class)
         {
            errorMessage += "The field " + expectedFieldName + " is not of the right type.\n";
            continue;
         }

         // We sneak in to assert that the type of RecyclingArrayList is of commandType.
         field.setAccessible(true);
         RecyclingArrayList<?> fieldInstance = (RecyclingArrayList<?>) field.get(instance);
         Field allocatorField = fieldInstance.getClass().getDeclaredField("allocator");
         allocatorField.setAccessible(true);
         Supplier<?> allocator = (Supplier<?>) allocatorField.get(fieldInstance);

         if (allocator.get().getClass() != commandType)
         {
            errorMessage += "Wrong buffer type for field: " + expectedFieldName + "\n";
            continue;
         }

         Method addCommandMethod = null;

         try
         {
            addCommandMethod = InverseDynamicsCommandBuffer.class.getDeclaredMethod("add" + commandType.getSimpleName());
         }
         catch (NoSuchMethodException e)
         {
            errorMessage += "Missing add command method for " + commandType.getSimpleName() + "'s buffer.\n";
            continue;
         }

         if (addCommandMethod.getReturnType() != commandType)
         {
            errorMessage += addCommandMethod.getName() + " does not return the proper command\n";
            continue;
         }

         Object newCommand = addCommandMethod.invoke(instance);
         expectedSize++;

         if (instance.getNumberOfCommands() != expectedSize)
         {
            expectedSize--; // So the test does not fail for the other commands.
            errorMessage += addCommandMethod.getName() + " does not register the new command to the super List.\n";
            continue;
         }

         if (instance.getCommand(expectedSize - 1) != newCommand)
         {
            errorMessage += "Unexpected command in the super List after invoking " + addCommandMethod.getName() + ", expected: " + commandType.getSimpleName()
                  + ", was: " + instance.getCommand(expectedSize - 1).getClass().getSimpleName();
         }
      }

      if (errorMessage.isEmpty())
      {
         instance.clear();

         if (instance.getNumberOfCommands() != 0)
            errorMessage += "Invoked clear, but the command is not empty.";

         for (Class<? extends InverseDynamicsCommand> commandType : commandTypes)
         {
            if (commandType.isInterface())
               continue;

            String expectedFieldName = CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_CAMEL, commandType.getSimpleName()) + "Buffer";
            Field field = nameToFieldMap.get(expectedFieldName);

            RecyclingArrayList<?> buffer = (RecyclingArrayList<?>) field.get(instance);
            if (!buffer.isEmpty())
               errorMessage += "The buffer " + field.getName() + " does not clear upon invoking the clear method.\n";
         }
      }

      if (!errorMessage.isEmpty())
         fail("The following issues were detected:\n" + errorMessage);
   }

}
