package us.ihmc.SdfLoader;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import us.ihmc.SdfLoader.xmlDescription.SDFModel;
import us.ihmc.SdfLoader.xmlDescription.SDFRoot;

public class JaxbSDFLoader
{
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;
   private final SDFJointNameMap sdfJointNameMap;
   

   private static ArrayList<String> createArrayListOfOneURL(String oneURL)
   {
      ArrayList<String> ret = new ArrayList<String>();
      ret.add(oneURL);
      return ret;
   }
   
   public JaxbSDFLoader(File file, String modelName, String resourceDirectory, SDFJointNameMap sdfJointNameMap) throws JAXBException, FileNotFoundException
   {
      this(file, modelName, createArrayListOfOneURL(resourceDirectory), sdfJointNameMap);
   }
   
   public JaxbSDFLoader(File file, String modelName, ArrayList<String> resourceDirectories, SDFJointNameMap sdfJointNameMap) throws JAXBException, FileNotFoundException
   {
      JAXBContext context = JAXBContext.newInstance(SDFRoot.class);
      Unmarshaller um = context.createUnmarshaller();
      SDFRoot sdfRoot = (SDFRoot) um.unmarshal(new FileReader(file));

      List<SDFModel> models;
      SDFModel model = null;
      if(sdfRoot.getWorld() != null)
      {
         models = sdfRoot.getWorld().getModels();
      }
      else
      {
         models = sdfRoot.getModels();
      }
      for (SDFModel modelInstance : models)
      {
         if (modelName.equals(modelInstance.getName()))
         {
            model = modelInstance;
            break;
         }
      }
      if (model == null)
      {
         throw new RuntimeException(modelName + " not found");
      }
      generalizedSDFRobotModel = new GeneralizedSDFRobotModel(modelName, model, resourceDirectories);
      this.sdfJointNameMap = sdfJointNameMap;
      
      
     
   }
   
   public GeneralizedSDFRobotModel getGeneralizedSDFRobotModel()
   {
      return generalizedSDFRobotModel;
   }

   public SDFRobot createRobot()
   {
      return new SDFRobot(generalizedSDFRobotModel, sdfJointNameMap);
   }

   public SDFFullRobotModel createFullRobotModel()
   {
      if(sdfJointNameMap != null)
      {
         return new SDFFullRobotModel(generalizedSDFRobotModel.getRootLinks().get(0), sdfJointNameMap);
      }
      else
      {
         throw new RuntimeException("Cannot make a fullrobotmodel without a sdfJointNameMap");
      }
   }
}
