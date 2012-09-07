package com.yobotics.simulationconstructionset.gui.graphics;

public class GraphicsSynchronizer
{
   private static boolean createdOne = false;
   
   public GraphicsSynchronizer()
   {
      if (createdOne)
      {
         String errorMessage = "Should only ever create one Graphics Synchronizer in order for everything to stay synchronized!";
         System.err.println(errorMessage);
//         throw new RuntimeException(errorMessage);
      }
      createdOne = true;
   }
   
   
}
