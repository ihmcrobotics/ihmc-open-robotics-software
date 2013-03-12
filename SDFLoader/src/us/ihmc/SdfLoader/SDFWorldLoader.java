package us.ihmc.SdfLoader;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import javax.xml.bind.JAXBException;

import org.apache.commons.math3.util.Pair;

import us.ihmc.SdfLoader.xmlDescription.SDFWorld.Road;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.jme.JMEGeneratedHeightMap;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.FileTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class SDFWorldLoader
{
   public HashMap<String, Graphics3DObject> visuals = new HashMap<String, Graphics3DObject>();
   private final JaxbSDFLoader jaxbSDFLoader;
   
   
   public SDFWorldLoader(File file, String resourceDirectory) throws FileNotFoundException, JAXBException
   {
      this(file, FileTools.createArrayListOfOneURL(resourceDirectory));
   }
   
   public SDFWorldLoader(File file, ArrayList<String> resourceDirectories) throws FileNotFoundException, JAXBException
   {
      jaxbSDFLoader = new JaxbSDFLoader(file, resourceDirectories);
      for(GeneralizedSDFRobotModel generalizedSDFRobotModel : jaxbSDFLoader.getGeneralizedSDFRobotModels())
      {
         String name = generalizedSDFRobotModel.getName();
         visuals.put(name, new SDFModelVisual(generalizedSDFRobotModel));
      }
      
      for(Road road : jaxbSDFLoader.getRoads())
      {
         visuals.put(road.getName(), new SDFRoadVisual(road));
      }
   }
   
   public HeightMap getHeightMap(HeightMapObjectFilter heightMapObjectFilter)
   {
      ArrayList<Graphics3DNode> nodes = new ArrayList<Graphics3DNode>();
      for(Entry<String, Graphics3DObject> visual : visuals.entrySet())
      {
         if(heightMapObjectFilter.isTerrainObject(visual.getKey()))
         {
            Graphics3DNode node = new Graphics3DNode(visual.getKey(), Graphics3DNodeType.GROUND);
            node.setGraphicsObject(visual.getValue());
            nodes.add(node);
         }
      }
      HeightMap heightMap = new JMEGeneratedHeightMap(nodes);
      
      return heightMap;  
   }
   
   public Pair<SDFRobot, SDFFullRobotModel> createRobotAndRemoveFromWorld(SDFJointNameMap sdfJointNameMap)
   {
      if(!visuals.containsKey(sdfJointNameMap.getModelName()))
      {
         throw new RuntimeException("Unkown robot");
      }
      
      Pair<SDFRobot, SDFFullRobotModel> ret = new Pair<SDFRobot, SDFFullRobotModel>(
            jaxbSDFLoader.createRobot(sdfJointNameMap),
            jaxbSDFLoader.createFullRobotModel(sdfJointNameMap));
      
      visuals.remove(sdfJointNameMap.getModelName());
      
      return ret;
   }
   
   public void addStaticGraphicsToSCS(SimulationConstructionSet scs)
   {
      scs.setGroundVisible(false);
      for(Graphics3DObject visual : visuals.values())
      {
         scs.addStaticLinkGraphics(visual);
      }
   }
   
   public interface HeightMapObjectFilter
   {
      public boolean isTerrainObject(String name);
   }
}
