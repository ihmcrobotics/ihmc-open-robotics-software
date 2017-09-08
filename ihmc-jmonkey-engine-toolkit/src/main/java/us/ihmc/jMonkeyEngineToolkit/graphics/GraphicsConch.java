package us.ihmc.jMonkeyEngineToolkit.graphics;

public class GraphicsConch
{
   private static boolean createdOne = false;
   
   public GraphicsConch()
   {
      if (createdOne)
      {
//         String errorMessage = "Should only ever create one Graphics Synchronizer in order for everything to stay synchronized!";
//         System.err.println(errorMessage);
//         throw new RuntimeException(errorMessage);
      }
      createdOne = true;
   }
   
   
}
