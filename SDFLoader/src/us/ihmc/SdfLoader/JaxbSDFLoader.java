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
   private final SDFRobot robot;
   private final SDFFullRobotModel fullRobotModel;

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

      GeneralizedSDFRobotModel generalizedSDFRobotModel = new GeneralizedSDFRobotModel(modelName, model, resourceDirectories);

      robot = new SDFRobot(generalizedSDFRobotModel, sdfJointNameMap);
      if(sdfJointNameMap != null)
      {
         fullRobotModel = new SDFFullRobotModel(generalizedSDFRobotModel.getRootLinks().get(0), sdfJointNameMap);
      }
      else
      {
         fullRobotModel = null;
      }
   }

   public SDFRobot getRobot()
   {
      return robot;
   }

   public SDFFullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }
   
   

   //   public static void main(String[] args) throws FileNotFoundException, JAXBException
   //   {
   //     JaxbSDFLoader jaxbSDFLoader = new JaxbSDFLoader();
   //     DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   //     RobotController controller = new AllAnglesController(jaxbSDFLoader.fullRobotModel, dynamicGraphicObjectsListRegistry);
   //     
   //     ModularRobotController modularRobotController = new ModularRobotController("jaxbController");
   //     SDFPerfectSimulatedSensorReaderAndWriter sensorReaderAndOutputWriter = new SDFPerfectSimulatedSensorReaderAndWriter(jaxbSDFLoader.robot, jaxbSDFLoader.fullRobotModel);
   //     modularRobotController.setRawSensorReader(sensorReaderAndOutputWriter);
   //     modularRobotController.addRobotController(controller);
   //     modularRobotController.setRawOutputWriter(sensorReaderAndOutputWriter);
   //
   //     
   //     jaxbSDFLoader.robot.setController(modularRobotController);
   //     
   //     
   //     SimulationConstructionSet scs = new SimulationConstructionSet(jaxbSDFLoader.robot);
   //     scs.setMaxBufferSize(65536);
   //     LinearGroundContactModel linearGroundContactModel = new LinearGroundContactModel(jaxbSDFLoader.robot, 150.0, 50.0, 25000.0, 1000.0, scs.getRootRegistry());
   //     jaxbSDFLoader.robot.setGroundContactModel(linearGroundContactModel);
   //     dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
   //     
   //     Thread thread = new Thread(scs);
   //     thread.start();
   //   }
}
