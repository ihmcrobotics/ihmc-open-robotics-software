package us.ihmc.SdfLoader;

import java.io.FileNotFoundException;

import javax.xml.bind.JAXBException;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class TestPrimitiveLoader
{
   public static void main(String argv[]) throws FileNotFoundException, JAXBException
   {
      JaxbSDFLoader loader = new JaxbSDFLoader("Models/unitBox.sdf", "unit_box_1", "Models/", null);
      Robot robot = loader.getRobot();
      
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      Thread thread = new Thread(scs);
      thread.start();
   }
}
