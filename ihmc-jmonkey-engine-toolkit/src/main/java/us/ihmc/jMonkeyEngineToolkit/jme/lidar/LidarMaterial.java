package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;

public class LidarMaterial extends Material
{
   public LidarMaterial(AssetManager assetManager)
   {
      super(assetManager, "lidar/Lidar.j3md");
   }
}
