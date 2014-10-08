package us.ihmc.valkyrie.visualizer;

import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.ValkyrieSliderBoardType;

public class RemoteValkyrieTorqueControlSliderBoardVisualizer
{
   public RemoteValkyrieTorqueControlSliderBoardVisualizer(String[] networkArguments)
   {
      new RemoteValkyrieVisualizer(networkArguments, ValkyrieSliderBoardType.TORQUE_PD_CONTROL);
   }
   
   public static void main(String[] args)
   {
      new RemoteValkyrieTorqueControlSliderBoardVisualizer(args);
   }
}
