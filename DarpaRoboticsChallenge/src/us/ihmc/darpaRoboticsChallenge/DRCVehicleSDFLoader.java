package us.ihmc.darpaRoboticsChallenge;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.MalformedURLException;
import java.net.URL;

import javax.media.j3d.Transform3D;
import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFModelVisual;
import us.ihmc.SdfLoader.SDFWorldLoader;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWorld;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving.VehicleModelObjectVisualizer;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCVehicleSDFLoader extends DRCWorld
{
   
   public DRCVehicleSDFLoader()
   {
      super();
   }
   
   public SDFModelVisual loadDRCVehicle(boolean loadCollisionModel)
   {
      URL fileURL = DRCVehicleSDFLoader.class.getClassLoader().getResource("models/GFE/drc_vehicle.sdf");
      try
      {
         SDFWorldLoader sdfWorldLoader = new SDFWorldLoader(new File(fileURL.getFile()), resourceDirectories);
         GeneralizedSDFRobotModel generalizedSDFRobotModel = sdfWorldLoader.getGeneralizedRobotModelAndRemoveFromWorld("drc_vehicle");
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
      scs.addStaticLinkGraphics(drcVehicleSDFLoader.loadDRCVehicle(true));
      scs.addStaticLinkGraphics(drcVehicleSDFLoader.loadDRCVehicle(false));

      Transform3D vehicleToWorldTransform = new Transform3D();
      ReferenceFrame vehicleFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("vehicle", ReferenceFrame.getWorldFrame(),
            vehicleToWorldTransform, false, true, true);
      DRCVehicleModelObjects vehicleModelObjects = new DRCVehicleModelObjects();
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      YoVariableRegistry registry = scs.getRootRegistry();
      VehicleModelObjectVisualizer vehicleModelObjectVisualizer = new VehicleModelObjectVisualizer(vehicleFrame, vehicleModelObjects, dynamicGraphicObjectsListRegistry, registry);
      vehicleModelObjectVisualizer.setVisible(true);
      vehicleModelObjectVisualizer.update();

      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

      Thread thread = new Thread(scs);
      thread.start();
   }
   
}
