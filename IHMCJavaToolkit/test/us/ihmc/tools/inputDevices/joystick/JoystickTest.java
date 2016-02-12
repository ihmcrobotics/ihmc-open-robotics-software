package us.ihmc.tools.inputDevices.joystick;

import static org.junit.Assert.*;
import static us.ihmc.tools.testing.TestPlanTarget.*;

import java.io.IOException;

import org.junit.Test;

import net.java.games.input.Component.Identifier;
import net.java.games.input.Event;
import net.java.games.input.test.ControllerReadTest;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.virtualJoystick.VirtualJoystick;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.RunnableThatThrows;

public class JoystickTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
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

   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 10000)
   public void testCreateVirtualJoystick()
   {
      Object monitor = new Object();

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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testFindNonExistentJoystick()
   {
      JUnitTools.assertExceptionThrown(JoystickNotFoundException.class, new RunnableThatThrows()
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

   @DeployableTestMethod(estimatedDuration = 10.0, targets = Manual)
   @Test(timeout = 300000)
   public void testCreateTwoJoysticks()
   {
      Object monitor1 = new Object();
      Object monitor2 = new Object();

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
