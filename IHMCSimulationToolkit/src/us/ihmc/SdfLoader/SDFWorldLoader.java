package us.ihmc.SdfLoader;

import us.ihmc.SdfLoader.xmlDescription.SDFWorld.Road;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGeneratedHeightMap;

import javax.xml.bind.JAXBException;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map.Entry;

public class SDFWorldLoader
{
   public LinkedHashMap<String, Graphics3DObject> visuals = new LinkedHashMap<String, Graphics3DObject>();
   private final JaxbSDFLoader jaxbSDFLoader;

   public SDFWorldLoader(File file, String resourceDirectory) throws FileNotFoundException, JAXBException
   {
      this(new FileInputStream(file), Arrays.asList(resourceDirectory));
   }

   public SDFWorldLoader(InputStream inputStream, List<String> resourceDirectories) throws FileNotFoundException, JAXBException
   {
      jaxbSDFLoader = new JaxbSDFLoader(inputStream, resourceDirectories, null);
      for (GeneralizedSDFRobotModel generalizedSDFRobotModel : jaxbSDFLoader.getGeneralizedSDFRobotModels())
      {
         String name = generalizedSDFRobotModel.getName();
         visuals.put(name, new SDFModelVisual(generalizedSDFRobotModel));
      }

      for (Road road : jaxbSDFLoader.getRoads())
      {
         visuals.put(road.getName(), new SDFRoadVisual(road));
      }
   }

   public HeightMap getGroundProfile(GroundProfileObjectFilter groundProfileObjectFilter, int resolution)
   {
      ArrayList<Graphics3DNode> nodes = new ArrayList<Graphics3DNode>();
      for (Entry<String, Graphics3DObject> visual : visuals.entrySet())
      {
         if (groundProfileObjectFilter.isTerrainObject(visual.getKey()))
         {
            Graphics3DNode node = new Graphics3DNode(visual.getKey(), Graphics3DNodeType.GROUND);
            node.setGraphicsObject(visual.getValue());
            nodes.add(node);
         }
      }
      HeightMap groundProfile = new JMEGeneratedHeightMap(nodes, resolution);

      return groundProfile;
   }

   private void removeVisualFromWorld(String modelName)
   {
      if (!visuals.containsKey(modelName))
      {
         throw new RuntimeException("Unkown robot");
      }
      visuals.remove(modelName);
   }

   public GeneralizedSDFRobotModel getGeneralizedRobotModelAndRemoveFromWorld(String modelName)
   {
      removeVisualFromWorld(modelName);
      return jaxbSDFLoader.getGeneralizedSDFRobotModel(modelName);
   }

   public Graphics3DObject createGraphics3dObject()
   {
      Graphics3DObject ret = new Graphics3DObject();
      for (Graphics3DObject visual : visuals.values())
      {
         ret.combine(visual);
      }

      return ret;
   }

   public interface GroundProfileObjectFilter
   {
      public boolean isTerrainObject(String name);
   }
}
