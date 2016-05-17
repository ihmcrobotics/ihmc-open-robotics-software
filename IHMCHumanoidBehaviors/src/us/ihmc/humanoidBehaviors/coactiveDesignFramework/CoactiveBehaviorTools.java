package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

public class CoactiveBehaviorTools
{
   public static final int PORT_FOR_COACTIVE_ELEMENTS = 56122;

   public static void synchronizeCoactiveElementMachineSideUsingTCPYoWhiteBoard(CoactiveElement coactiveElement)
   {
      if (coactiveElement != null)
      {
         HumanOrMachine whichSideIsThisRunningOn = HumanOrMachine.MACHINE;
         CoactiveElementYoWhiteBoardSynchronizer synchronizer = new CoactiveElementYoWhiteBoardSynchronizer(PORT_FOR_COACTIVE_ELEMENTS, whichSideIsThisRunningOn, coactiveElement);
         
         long millisecondsBetweenDataWrites = 100L;
         synchronizer.startASynchronizerOnAThread(millisecondsBetweenDataWrites);
      }
   }
}
