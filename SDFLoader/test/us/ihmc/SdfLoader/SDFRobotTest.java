package us.ihmc.SdfLoader;

import java.io.FileNotFoundException;

import javax.xml.bind.JAXBException;

import org.junit.Test;

import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;

public class SDFRobotTest
{
   @EstimatedDuration(duration = 1.0)
   @Test(timeout = 3000)
   public void testNullJointMap() throws FileNotFoundException, JAXBException
   {
      JaxbSDFLoader loader = new JaxbSDFLoader(getClass().getClassLoader().getResourceAsStream("sdfRobotTest.sdf"), null);
      
      GeneralizedSDFRobotModel generalizedSDFRobotModel = loader.getGeneralizedSDFRobotModel("atlas");
      new SDFRobot(generalizedSDFRobotModel, null, true);
   }

}
