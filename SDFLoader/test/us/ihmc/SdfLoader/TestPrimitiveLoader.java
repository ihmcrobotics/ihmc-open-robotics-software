package us.ihmc.SdfLoader;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.MalformedURLException;

import javax.xml.bind.JAXBException;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class TestPrimitiveLoader
{
   public static void main(String argv[]) throws FileNotFoundException, JAXBException, MalformedURLException
   {
//      JaxbSDFLoader loader = new JaxbSDFLoader(new File("Models/unitBox.sdf"), "Models/");
//      Robot robot = loader.createRobot("unit_box_1");
      SDFWorldLoader loader = new SDFWorldLoader(new File("Models/unitBox.sdf"), "Models/");
      
      SimulationConstructionSet scs = new SimulationConstructionSet();
      loader.addStaticGraphicsToSCS(scs);
      
      
      Thread thread = new Thread(scs);
      thread.start();
   }
}
