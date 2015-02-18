package us.ihmc.valkyrie.visualizer;

import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.ValkyrieSliderBoardType;

public class RemoteValkyrieTurboDriverPositionControlSliderBoardVisualizer
{
   public RemoteValkyrieTurboDriverPositionControlSliderBoardVisualizer(String[] networkArguments)
   {
      new RemoteValkyrieVisualizer(ValkyrieSliderBoardType.ON_BOARD_POSITION);
   }
   
   public static void main(String[] args)
   {
      new RemoteValkyrieTurboDriverPositionControlSliderBoardVisualizer(args);
   }
}
