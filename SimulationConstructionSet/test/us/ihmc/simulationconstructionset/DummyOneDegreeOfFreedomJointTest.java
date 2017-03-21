package us.ihmc.simulationconstructionset;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;

public class DummyOneDegreeOfFreedomJointTest
{
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testDummyOneDegreeOfFreedomJoint()
   {
      Robot robot = new Robot("testDummyOneDegreeOfFreedomJoint");
      Vector3D jointAxis = new Vector3D(0.0, 1.0, 0.0);
      DummyOneDegreeOfFreedomJoint jointOne = new DummyOneDegreeOfFreedomJoint("jointOne", new Vector3D(), robot, jointAxis);
      
      Link linkOne = new Link("linkOne");
      linkOne.setMassAndRadiiOfGyration(1.0, 0.1, 0.1, 0.1);
      
      Graphics3DObject linkOneGraphics = new Graphics3DObject();
      linkOneGraphics.addCylinder(0.1, 0.01);
      
      linkOne.setLinkGraphics(linkOneGraphics);
      jointOne.setLink(linkOne);
      
      robot.addRootJoint(jointOne);
      
      SimulationConstructionSetParameters parameters = SimulationTestingParameters.createFromEnvironmentVariables();
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      
      scs.startOnAThread();
      
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         ThreadTools.sleepForever();
      }
   }
}
