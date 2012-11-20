package us.ihmc.SdfLoader;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import us.ihmc.SdfLoader.drcRobot.DRCRobotJointMap;
import us.ihmc.SdfLoader.xmlDescription.SDFGazebo;
import us.ihmc.SdfLoader.xmlDescription.SDFModel;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.LinearGroundContactModel;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class JaxbSDFLoader
{

   private static final String fileName = "Models/GFE/models/drc_robot/gfe.sdf";
   private static final String modelName = "drc_robot";
   private static final String resourceDirectory = "Models/GFE/models/";

   private final SDFRobot robot;
   private final SDFFullRobotModel fullRobotModel;

   public JaxbSDFLoader() throws JAXBException, FileNotFoundException
   {
      JAXBContext context = JAXBContext.newInstance(SDFGazebo.class);
      Unmarshaller um = context.createUnmarshaller();
      File file = new File(fileName);
      SDFGazebo gazebo = (SDFGazebo) um.unmarshal(new FileReader(file));

      SDFModel model = null;
      for (SDFModel modelInstance : gazebo.getWorld().getModels())
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

      GeneralizedSDFRobotModel generalizedSDFRobotModel = new GeneralizedSDFRobotModel(modelName, model, resourceDirectory);

      DRCRobotJointMap sdfJointNameMap = new DRCRobotJointMap();
      robot = new SDFRobot(generalizedSDFRobotModel, sdfJointNameMap);
      fullRobotModel = new SDFFullRobotModel(generalizedSDFRobotModel.getRootLinks().get(0), sdfJointNameMap);
   }

   public static void main(String[] args) throws FileNotFoundException, JAXBException
   {
     JaxbSDFLoader jaxbSDFLoader = new JaxbSDFLoader();
     DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
     RobotController controller = new AllAnglesController(jaxbSDFLoader.fullRobotModel, dynamicGraphicObjectsListRegistry);
     
     ModularRobotController modularRobotController = new ModularRobotController("jaxbController");
     SDFPerfectSimulatedSensorReaderAndWriter sensorReaderAndOutputWriter = new SDFPerfectSimulatedSensorReaderAndWriter(jaxbSDFLoader.robot, jaxbSDFLoader.fullRobotModel);
     modularRobotController.setRawSensorReader(sensorReaderAndOutputWriter);
     modularRobotController.addRobotController(controller);
     modularRobotController.setRawOutputWriter(sensorReaderAndOutputWriter);

     
     jaxbSDFLoader.robot.setController(modularRobotController);
     
     
     SimulationConstructionSet scs = new SimulationConstructionSet(jaxbSDFLoader.robot);
     scs.setMaxBufferSize(65536);
     LinearGroundContactModel linearGroundContactModel = new LinearGroundContactModel(jaxbSDFLoader.robot, 150.0, 50.0, 25000.0, 1000.0, scs.getRootRegistry());
     jaxbSDFLoader.robot.setGroundContactModel(linearGroundContactModel);
     dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
     
     Thread thread = new Thread(scs);
     thread.start();
   }
}
