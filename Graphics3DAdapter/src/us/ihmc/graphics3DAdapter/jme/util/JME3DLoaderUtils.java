package us.ihmc.graphics3DAdapter.jme.util;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.jme.JMEAssetLocator;
import us.ihmc.graphics3DAdapter.jme.JMEGraphicsObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;

import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;

public class JME3DLoaderUtils
{
   public static Spatial load3DModel(String modelPath, JMEAssetLocator assetManager, Graphics3DNodeType type)
   {
      return JMEGraphicsObject.createGraphics3DObjectFromModel(modelPath, null, false, null, assetManager);
   }

   public static ArrayList<Geometry> extractGeometry(Spatial spatial)
   {
      ArrayList<Geometry> geom = new ArrayList<Geometry>();
      getGeometry(spatial, geom);

      return geom;
   }

   public static Geometry extractFirstGeometry(String modelPath, JMEAssetLocator assetManager, Graphics3DNodeType type)
   {
      Spatial tmp = load3DModel(modelPath, assetManager, type);

      return extractFirstGeometry(tmp);
   }

   public static Geometry extractFirstGeometry(Spatial spatial)
   {
      ArrayList<Geometry> geom = new ArrayList<Geometry>();
      getGeometry(spatial, geom);

      return geom.get(0);
   }

   private static void getGeometry(Spatial spatial, ArrayList<Geometry> parent)
   {
      if (spatial instanceof Node)
      {
         Node n = (Node) spatial;
         for (Spatial child : n.getChildren())
         {
            getGeometry(child, parent);
         }
      }
      else if (spatial instanceof Geometry)
      {
         parent.add((Geometry) spatial);
      }
   }
}
