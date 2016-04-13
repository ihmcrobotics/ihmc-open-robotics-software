package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.io.IOException;

import us.ihmc.humanoidBehaviors.behaviors.KickBallBehaviorCoactiveElement;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.whiteBoard.TCPYoWhiteBoard;

public class SimpleCoactiveElementMachineSide
{
   private static final int PORT_FOR_COACTIVE_ELEMENTS = 56122;
   private final CoactiveElement coactiveElement;

   public SimpleCoactiveElementMachineSide(CoactiveElement coactiveElement)
   {
      this.coactiveElement = coactiveElement;
   }

   public void startOnAThread(int port)
   {
      final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("NullRobotMachineSide"));
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      rootRegistry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());
      rootRegistry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
      scs.startOnAThread();
      
      coactiveElement.initializeMachineSide();
      final TCPYoWhiteBoard machineSideWhiteBoard = new TCPYoWhiteBoard("MachineSideWhiteBoard", port);

      CoactiveElementYoWhiteBoardSynchronizer machineSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(machineSideWhiteBoard, HumanOrMachine.MACHINE, coactiveElement);

      Thread machineSideThread = new Thread(machineSideWhiteBoard);
      machineSideThread.start();

      final long millisecondsBetweenDataWrites = 300L;
      machineSideSynchronizer.startASynchronizerOnAThread(millisecondsBetweenDataWrites);

      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            boolean connected = false;
            
            while(!connected)
            {
               try
               {
                  System.out.println("Trying to connect");
                  machineSideWhiteBoard.connect();
                  connected = true;
                  System.out.println("Connected!!!");
               }
               catch (IOException e1)
               {
                  System.out.println(e1);
                  sleep(1000L);
               }
            }
   
            while (true)
            {
               coactiveElement.updateMachineSide();
               scs.tickAndUpdate();

               sleep(millisecondsBetweenDataWrites);       
            }
         }
      };

      Thread thread = new Thread(runnable);
      thread.start();
   }

   private void sleep(long sleepTimeMillis)
   {
      try
      {
         Thread.sleep(sleepTimeMillis);
      }
      catch(Exception e)
      {
      }
   }

   public static void main(String[] args)
   {
      KickBallBehaviorCoactiveElement coactiveElement = new KickBallBehaviorCoactiveElement();

      SimpleCoactiveElementMachineSide machineSide = new SimpleCoactiveElementMachineSide(coactiveElement);
      machineSide.startOnAThread(PORT_FOR_COACTIVE_ELEMENTS);
   }

}

