package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import java.io.IOException;

import us.ihmc.humanoidBehaviors.behaviors.KickBallBehaviorCoactiveElement;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.whiteBoard.TCPYoWhiteBoard;

public class SimpleCoactiveElementUserInterface
{
   private static final int PORT_FOR_COACTIVE_ELEMENTS = 56122;
   private final CoactiveElement coactiveElement;

   public SimpleCoactiveElementUserInterface(CoactiveElement coactiveElement)
   {
      this.coactiveElement = coactiveElement;
   }

   public void startOnAThread(String ipAddress, int port)
   {
      final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("NullRobotUserInterfaceSide"));
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      rootRegistry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());
      rootRegistry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
      scs.startOnAThread();

      coactiveElement.initializeUserInterfaceSide();
      final TCPYoWhiteBoard userInterfaceSideWhiteBoard = new TCPYoWhiteBoard("UserInterfaceSideWhiteBoard", ipAddress, port);

      CoactiveElementYoWhiteBoardSynchronizer userInterfaceSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(userInterfaceSideWhiteBoard, HumanOrMachine.HUMAN, coactiveElement);

      Thread userInterfaceSideThread = new Thread(userInterfaceSideWhiteBoard);
      userInterfaceSideThread.start();

      final long millisecondsBetweenDataWrites = 300L;
      userInterfaceSideSynchronizer.startASynchronizerOnAThread(millisecondsBetweenDataWrites);

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
                  userInterfaceSideWhiteBoard.connect();
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
               coactiveElement.updateUserInterfaceSide();
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

//      SimpleCoactiveElementMachineSide machineSide = new SimpleCoactiveElementMachineSide(coactiveElement);
//      machineSide.startOnAThread(PORT_FOR_COACTIVE_ELEMENTS);

      SimpleCoactiveElementUserInterface userInterface = new SimpleCoactiveElementUserInterface(coactiveElement);
      userInterface.startOnAThread("localhost", PORT_FOR_COACTIVE_ELEMENTS);
   }

}
