package us.ihmc.darpaRoboticsChallenge;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.MalformedURLException;
import java.net.URL;

import javax.xml.bind.JAXBException;

import com.yobotics.simulationconstructionset.Robot;
import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.SdfLoader.SDFModelVisual;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFWorldLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWorld;
import us.ihmc.utilities.net.TimestampProvider;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;

public class DRCVehicleSDFLoader extends DRCWorld
{
   
   public DRCVehicleSDFLoader()
   {
      super();
   }
   
   public SDFModelVisual loadDRCVehicle()
   {
      URL fileURL = DRCVehicleSDFLoader.class.getResource("models/GFE/models/polaris_ranger_ev/model-1_3.sdf");
      try
      {
         SDFWorldLoader sdfWorldLoader = new SDFWorldLoader(new File(fileURL.getFile()), resourceDirectories);
         GeneralizedSDFRobotModel generalizedSDFRobotModel = sdfWorldLoader.getGeneralizedRobotModelAndRemoveFromWorld("polaris_ranger_ev");
         return new SDFModelVisual(generalizedSDFRobotModel);
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
      scs.addStaticLinkGraphics(drcVehicleSDFLoader.loadDRCVehicle());

      DRCVehicleModelObjectVisualizer drcVehicleModelObjectVisualizer = new DRCVehicleModelObjectVisualizer(scs, false);


      Thread thread = new Thread(scs);
      thread.start();
   }

   public TerrainObject getTerrainObject()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public SDFRobot getRobot()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public SDFFullRobotModelFactory getFullRobotModelFactory()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public DRCRobotJointMap getJointMap()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public double getSimulateDT()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   public boolean simulateDelay()
   {
      // TODO Auto-generated method stub
      return false;
   }

   public TimestampProvider getTimeStampProvider()
   {
      // TODO Auto-generated method stub
      return null;
   }
   
}
