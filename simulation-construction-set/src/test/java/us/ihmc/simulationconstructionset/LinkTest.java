package us.ihmc.simulationconstructionset;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.tools.thread.ThreadTools;

public class LinkTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testLinkInertia()
   {
      // This is a manual test. Run it with visualize = true and verify that the ellipsoid graphics rotates around the long axis.
      // If not, then addEllipsoidFromMassProperties() isn't working properly.
      
      boolean visualize = false;
      
      Robot robot = new Robot("linkTest");
      
      FloatingJoint rootJoint = new FloatingJoint("root", new Vector3D(), robot);
      
      Link link = new Link("testLink");
      
      double mass = 1.7;
      link.setMass(mass);
      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setM00(0.1);
      momentOfInertia.setM11(0.1);
      momentOfInertia.setM22(0.001);
      
      Vector3D rotationAxis = new Vector3D(0.0, 0.0, 1.0);
      
      RotationMatrix rotation = new RotationMatrix();
      rotation.setToPitchMatrix(Math.PI/3.0);
      momentOfInertia = InertiaTools.rotate(rotation, momentOfInertia);
      rotation.transform(rotationAxis);
      
      rotation = new RotationMatrix();
      rotation.setToYawMatrix(Math.PI/5.0);
      momentOfInertia = InertiaTools.rotate(rotation, momentOfInertia);
      rotation.transform(rotationAxis);

      link.setMomentOfInertia(momentOfInertia);
      
      link.addEllipsoidFromMassProperties(YoAppearance.Gold());
      link.addEllipsoidFromMassProperties2(YoAppearance.Green());
      
      link.addCoordinateSystemToCOM(1.0);
      
      rootJoint.setLink(link);      
      robot.addRootJoint(rootJoint);
      
      robot.setGravity(0.0);
      rootJoint.setAngularVelocityInBody(rotationAxis);
      
      if (visualize)
      {
         SimulationConstructionSet scs = new SimulationConstructionSet(robot);
         scs.startOnAThread();
         
         scs.setSimulateDuration(1.0);
         
         scs.simulate();
         
         ThreadTools.sleepForever();
      }
   }

}
