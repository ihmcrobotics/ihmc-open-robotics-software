package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.net.MalformedURLException;
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
      
      InputStream fileInputStream = myClass.getResourceAsStream("models/polaris_ranger_ev/model-1_3.sdf");
      resourceDirectories.add(myClass.getResource("models/").getFile());
      
      

      JaxbSDFLoader jaxbSDFLoader;
      try
      {
         jaxbSDFLoader = new JaxbSDFLoader(fileInputStream, resourceDirectories);
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
      String modelName = "polaris_ranger_ev";
      JaxbSDFLoader loader = loadDRCVehicle();
      Robot robot = loader.createRobot(modelName);
      
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      Thread thread = new Thread(scs);
      thread.start();
   }
   
}
