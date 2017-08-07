package us.ihmc.modelFileLoaders.SdfLoader;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.MalformedURLException;

import javax.xml.bind.JAXBException;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PrimitiveLoaderTest
{
   public static void main(String argv[]) throws FileNotFoundException, JAXBException, MalformedURLException
   {
//      JaxbSDFLoader loader = new JaxbSDFLoader(new File("Models/unitBox.sdf"), "Models/");
//      Robot robot = loader.createRobot("unit_box_1");
      SDFWorldLoader loader = new SDFWorldLoader(new File("Models/unitBox.sdf"), "Models/");
      
      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.addStaticLinkGraphics(loader.createGraphics3dObject());
      
      
      Thread thread = new Thread(scs);
      thread.start();
   }
}
