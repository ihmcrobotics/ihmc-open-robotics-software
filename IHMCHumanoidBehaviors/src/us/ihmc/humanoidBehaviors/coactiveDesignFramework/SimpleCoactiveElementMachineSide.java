package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import us.ihmc.humanoidBehaviors.behaviors.KickBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

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

      CoactiveElementYoWhiteBoardSynchronizer machineSideSynchronizer = new CoactiveElementYoWhiteBoardSynchronizer(port, HumanOrMachine.MACHINE, coactiveElement);

      final long millisecondsBetweenDataWrites = 300L;
      machineSideSynchronizer.startASynchronizerOnAThread(millisecondsBetweenDataWrites);

      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {   
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
      KickBallBehaviorCoactiveElementBehaviorSide coactiveElement = new KickBallBehaviorCoactiveElementBehaviorSide();

      SimpleCoactiveElementMachineSide machineSide = new SimpleCoactiveElementMachineSide(coactiveElement);
      machineSide.startOnAThread(PORT_FOR_COACTIVE_ELEMENTS);
   }
}
