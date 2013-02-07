package us.ihmc.darpaRoboticsChallenge;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class DRCVehicleSDFLoader
{
   public static JaxbSDFLoader loadDRCVehicle()
   {
      ArrayList<String> resourceDirectories = new ArrayList<String>();
      Class<DRCVehicleSDFLoader> myClass = DRCVehicleSDFLoader.class;
      
      URL fileURL = myClass.getResource("models/polaris_ranger_ev/model-1_3.sdf");
      resourceDirectories.add(myClass.getResource("models/").getFile());
      String modelName = "polaris_ranger_ev";
      

      JaxbSDFLoader jaxbSDFLoader;
      try
      {
         jaxbSDFLoader = new JaxbSDFLoader(new File(fileURL.getFile()), modelName, resourceDirectories, null);
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException("Cannot find SDF file: " + e.getMessage());
      }
      catch (JAXBException e)
      {
         e.printStackTrace();

         throw new RuntimeException("Invalid SDF file: " + e.getMessage());
      }

      return jaxbSDFLoader;
   }
   
   public static void main(String argv[]) throws FileNotFoundException, JAXBException, MalformedURLException
   {
      JaxbSDFLoader loader = loadDRCVehicle();
      Robot robot = loader.getRobot();
      
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      Thread thread = new Thread(scs);
      thread.start();
   }
   
}
