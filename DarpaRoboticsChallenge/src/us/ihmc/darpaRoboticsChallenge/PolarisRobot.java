package us.ihmc.darpaRoboticsChallenge;

import java.net.URL;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.environment.DRCCarEgressEnvironment;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.RotationalInertiaCalculator;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

/**
 * This is a non-contactable Polaris model only for visualizing in SCS
 */

public class PolarisRobot extends Robot
{   
   private final String modelFile = "models/polarisModel.obj";
   private final FloatingJoint floatingJoint;
   private final Link link;
   private final Graphics3DObject linkGraphics;

   public PolarisRobot(String name, RigidBodyTransform rootJointTransform)
   {
      super(name);

      link = new Link(name + "Link");
      linkGraphics = new Graphics3DObject();
      linkGraphics.addModelFile(modelFile);
      link.setLinkGraphics(linkGraphics);
      
      link.setMass(1.0);
      link.setComOffset(new Vector3d());
      Matrix3d inertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(1.0, 1.0, 1.0, Axis.Z);
      link.setMomentOfInertia(inertia);
      
      floatingJoint = new FloatingJoint(name + "Base", name, new Vector3d(), this);
      floatingJoint.setRotationAndTranslation(rootJointTransform);
      floatingJoint.setLink(link);
      floatingJoint.setDynamic(false);
      this.addRootJoint(floatingJoint);
   }
}
