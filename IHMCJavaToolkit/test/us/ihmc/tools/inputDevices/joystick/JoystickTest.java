package us.ihmc.tools.inputDevices.joystick;

import static org.junit.Assert.assertEquals;

import java.io.IOException;

import org.junit.Test;

import net.java.games.input.Component.Identifier;
import net.java.games.input.Event;
import net.java.games.input.test.ControllerReadTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.testing.Assertions;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.virtualJoystick.VirtualJoystick;
import us.ihmc.tools.thread.RunnableThatThrows;

public class JoystickTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testCreateJoystick()
   {
      try
      {
         Joystick joystick = new Joystick();
         joystick.setPollInterval(100);
         joystick.addJoystickEventListener(new JoystickEventListener()
         {
            @Override
            public void processEvent(Event event)
            {
               System.out.println(event);
            }
         });
         joystick.addJoystickStatusListener(new JoystickStatusListener()
         {
            @Override
            public void updateConnectivity(boolean connected)
            {
               System.out.println(connected);
            }
         });
         joystick.shutdown();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCreateVirtualJoystick()
   {
      final Object monitor = new Object();

      try
      {
         VirtualJoystick virtualJoystick = new VirtualJoystick("VirtualJoystick");
         virtualJoystick.setPollInterval(100);
         virtualJoystick.addJoystickEventListener(new JoystickEventListener()
         {
            @Override
            public void processEvent(Event event)
            {
               System.out.println("VirtualJoystick: " + event);

               assertEquals("not X", Identifier.Axis.X, event.getComponent().getIdentifier());
               assertEquals("not 1.0", 1.0f, event.getValue(), 1e-5);

               synchronized (monitor)
               {
                  monitor.notify();
               }
            }
         });

         virtualJoystick.queueEvent(Identifier.Axis.X, 1.0);

         try
         {
            synchronized (monitor)
            {
               monitor.wait();
            }
         }
         catch (InterruptedException e)
         {
            // do nothing
         }
         
         virtualJoystick.shutdown();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindNonExistentJoystick()
   {
      Assertions.assertExceptionThrown(JoystickNotFoundException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            new Joystick(JoystickModel.SAITEK_X52, 2);
         }
      });
   }

   boolean madCatz5Status = false;
   boolean madCatz1Status = false;

   @ContinuousIntegrationTest(estimatedDuration = 10.0, categoriesOverride = IntegrationCategory.MANUAL)
   @Test(timeout = 300000)
   public void testCreateTwoJoysticks()
   {
      final Object monitor1 = new Object();
      final Object monitor2 = new Object();

      try
      {
         Joystick madCatz5 = new Joystick(JoystickModel.MAD_CATZ_FLY5_STICK, 0);
         Joystick madCatz1 = new Joystick(JoystickModel.MAD_CATZ_V1_STICK, 0);

         madCatz5.addJoystickStatusListener(new JoystickStatusListener()
         {
            @Override
            public void updateConnectivity(boolean connected)
            {
               if (connected != madCatz5Status)
               {
                  System.out.println("MadCatz5 connected: " + connected);
                  madCatz5Status = connected;
               }
            }
         });
         madCatz1.addJoystickStatusListener(new JoystickStatusListener()
         {
            @Override
            public void updateConnectivity(boolean connected)
            {
               if (connected != madCatz1Status)
               {
                  System.out.println("MadCatz1 connected: " + connected);
                  madCatz1Status = connected;
               }
            }
         });

         madCatz5.addJoystickEventListener(new JoystickEventListener()
         {
            @Override
            public void processEvent(Event event)
            {
               if (madCatz5Status)
               {
                  System.out.println("MadCatz5: " + event.toString());
                  
                  synchronized (monitor1)
                  {
                     monitor1.notify();
                  }
               }
            }
         });
         madCatz1.addJoystickEventListener(new JoystickEventListener()
         {
            @Override
            public void processEvent(Event event)
            {
               if (madCatz1Status)
               {
                  System.out.println("MadCatz1: " + event.toString());
                  
                  synchronized (monitor2)
                  {
                     monitor2.notify();
                  }
               }
            }
         });
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      try
      {
         synchronized (monitor1)
         {
            monitor1.wait();
         }
         synchronized (monitor2)
         {
            monitor2.wait();
         }
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      ControllerReadTest.main(new String[] {});
   }
}
