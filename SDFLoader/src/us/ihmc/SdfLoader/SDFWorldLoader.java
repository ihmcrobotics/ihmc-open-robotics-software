package us.ihmc.SdfLoader;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.xmlDescription.SDFWorld.Road;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.utilities.FileTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class SDFWorldLoader
{
   public HashMap<String, Graphics3DObject> visuals = new HashMap<String, Graphics3DObject>();
   
   public SDFWorldLoader(File file, String resourceDirectory) throws FileNotFoundException, JAXBException
   {
      this(file, FileTools.createArrayListOfOneURL(resourceDirectory));
   }
   
   public SDFWorldLoader(File file, ArrayList<String> resourceDirectories) throws FileNotFoundException, JAXBException
   {
      JaxbSDFLoader jaxbSDFLoader = new JaxbSDFLoader(file, resourceDirectories);
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
   
   public void addStaticGraphicsToSCS(SimulationConstructionSet scs)
   {
      scs.setGroundVisible(false);
      for(Graphics3DObject visual : visuals.values())
      {
         scs.addStaticLinkGraphics(visual);
      }
   }
}
