package us.ihmc.simulationconstructionset;

import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public class HeightMapFromGroundContactModel
{

   public static HeightMap getHeightMap(GroundContactModel groundContactModel)
   {
      HeightMap heightMap = null;

      if (groundContactModel != null)
      {
         GroundProfile3D groundProfile3D = groundContactModel.getGroundProfile3D();

         if (groundProfile3D != null) heightMap = groundProfile3D.getHeightMapIfAvailable();
      }

      return heightMap;
   }

}
