package us.ihmc.valkyrie.visualizer;

import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.ValkyrieSliderBoardType;

public class RemoteValkyrieWalkingVisualizer
{
   public RemoteValkyrieWalkingVisualizer(String[] networkArguments)
   {
      new RemoteValkyrieVisualizer(networkArguments, ValkyrieSliderBoardType.WALKING);
   }
   
   public static void main(String[] args)
   {
      new RemoteValkyrieWalkingVisualizer(args);
   }
}
