package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import us.ihmc.communication.util.NetworkPorts;

public class CoactiveBehaviorTools
{
   public static void synchronizeCoactiveElementMachineSideUsingTCPYoWhiteBoard(CoactiveElement coactiveElement)
   {
      if (coactiveElement != null)
      {
         HumanOrMachine whichSideIsThisRunningOn = HumanOrMachine.MACHINE;
         CoactiveElementYoWhiteBoardSynchronizer synchronizer = new CoactiveElementYoWhiteBoardSynchronizer(NetworkPorts.COACTIVE_ELEMENTS_PORT.getPort(), whichSideIsThisRunningOn, coactiveElement);
         
         long millisecondsBetweenDataWrites = 100L;
         synchronizer.startASynchronizerOnAThread(millisecondsBetweenDataWrites);
      }
   }
}
