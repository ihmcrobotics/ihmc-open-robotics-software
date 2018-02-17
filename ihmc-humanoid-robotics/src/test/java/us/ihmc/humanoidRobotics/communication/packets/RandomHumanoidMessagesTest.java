package us.ihmc.humanoidRobotics.communication.packets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Method;
import java.lang.reflect.ParameterizedType;
import java.util.List;
import java.util.Random;
import java.util.concurrent.TimeUnit;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.DisableOnDebug;
import org.junit.rules.Timeout;

public class RandomHumanoidMessagesTest
{
   @Rule
   public DisableOnDebug disableOnDebug = new DisableOnDebug(new Timeout(5, TimeUnit.SECONDS));

   @Test(timeout = Integer.MAX_VALUE)
   public void testMethodName()
   { // Assert that all the methods are following the convention: ReturnedObjectType nextReturnedObjectType(Random)
      for (Method method : RandomHumanoidMessages.class.getDeclaredMethods())
      {
         String methodName = method.getName();
         if (methodName.contains("lambda")) // No idea where the hell that's coming from...
            continue;

         Class<?> returnType = method.getReturnType();

         assertTrue(methodName, methodName.startsWith("next"));
         String methodNameStripped = methodName.substring(4);

         if (returnType.isArray())
         {
            assertTrue(methodName, methodName.endsWith("s"));
            methodNameStripped = methodNameStripped.substring(0, methodNameStripped.length() - 1);
            assertEquals(methodNameStripped, returnType.getComponentType().getSimpleName());

            assertTrue(methodName, method.getParameterCount() <= 2);
            assertEquals(methodName, Random.class, method.getParameters()[0].getType());

            if (method.getParameterCount() == 2)
            {
               assertEquals(methodName, int.class, method.getParameters()[1].getType());
            }

            continue;
         }
         else if (List.class.isAssignableFrom(returnType))
         {
            assertTrue(methodName, methodName.endsWith("s"));
            methodNameStripped = methodNameStripped.substring(0, methodNameStripped.length() - 1);
            Class<?> type = (Class<?>) ((ParameterizedType) method.getGenericReturnType()).getActualTypeArguments()[0];
            assertEquals(methodNameStripped, type.getSimpleName());

            assertTrue(methodName, method.getParameterCount() <= 2);
            assertEquals(methodName, Random.class, method.getParameters()[0].getType());

            if (method.getParameterCount() == 2)
            {
               assertEquals(methodName, int.class, method.getParameters()[1].getType());
            }

            continue;
         }
         else
         {
            String returnTypeName = returnType.getSimpleName();
            assertEquals(methodNameStripped, returnTypeName);
            assertEquals(methodName, 1, method.getParameterCount());
            assertEquals(methodName, Random.class, method.getParameters()[0].getType());
         }
      }
   }

}
