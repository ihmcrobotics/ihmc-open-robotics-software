package us.ihmc.avatar.polaris;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.net.MalformedURLException;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFModelVisual;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class DRCVehicleSDFLoader extends DRCWorld
{
   
   public DRCVehicleSDFLoader()
   {
      
   }
   
   public SDFModelVisual loadDRCVehicle(boolean loadCollisionModel)
   {
      ArrayList<String> resourceDirectories = new ArrayList<String>();
      resourceDirectories.add("models/");
      resourceDirectories.add("models/gazebo/");
      
      InputStream inputStream = DRCVehicleSDFLoader.class.getClassLoader().getResourceAsStream("models/polaris_ranger_xp900_no_roll_cage/model.sdf");
      try
      {
         JaxbSDFLoader jaxbSDFLoader = new JaxbSDFLoader(inputStream, resourceDirectories);
         GeneralizedSDFRobotModel generalizedSDFRobotModel = jaxbSDFLoader.getGeneralizedSDFRobotModel("polaris_ranger_xp900");
         return new SDFModelVisual(generalizedSDFRobotModel, loadCollisionModel);
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public static void main(String argv[]) throws FileNotFoundException, JAXBException, MalformedURLException
   {
      Robot robot = new Robot("robot");
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      
      DRCVehicleSDFLoader drcVehicleSDFLoader = new DRCVehicleSDFLoader();
      scs.addStaticLinkGraphics(drcVehicleSDFLoader.loadDRCVehicle(false));

      RigidBodyTransform vehicleToWorldTransform = new RigidBodyTransform();
      ReferenceFrame vehicleFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("vehicle", ReferenceFrame.getWorldFrame(),
            vehicleToWorldTransform);
      VehicleModelObjects vehicleModelObjects = new VehicleModelObjects();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = scs.getRootRegistry();
      VehicleModelObjectVisualizer vehicleModelObjectVisualizer = new VehicleModelObjectVisualizer(vehicleFrame, vehicleModelObjects, yoGraphicsListRegistry, registry);
      vehicleModelObjectVisualizer.setVisible(true);
      vehicleModelObjectVisualizer.update();

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      Thread thread = new Thread(scs);
      thread.start();
   }
   
}
