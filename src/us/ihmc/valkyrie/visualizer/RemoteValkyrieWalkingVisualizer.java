package us.ihmc.valkyrie.visualizer;

import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.ValkyrieSliderBoardType;

public class RemoteValkyrieWalkingVisualizer
{
   public RemoteValkyrieWalkingVisualizer()
   {
      new RemoteValkyrieVisualizer(ValkyrieSliderBoardType.WALKING);
   }
   
   public static void main(String[] args)
   {
      new RemoteValkyrieWalkingVisualizer();
   }
}
