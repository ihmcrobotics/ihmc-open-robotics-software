package us.ihmc.exampleSimulations.nonrewindableGraphicsExample;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

import javax.vecmath.Vector3d;

public class PointCloudRobot extends Robot
{
   public PointCloudRobot(String name)
   {
      super(name);

      Link rootJointLink = new Link("rootJointLink");
      Graphics3DObject rootLinkGraphics = new Graphics3DObject();
      rootLinkGraphics.addCube(0.2, 0.2, 0.2, YoAppearance.Yellow());
      rootJointLink.setLinkGraphics(rootLinkGraphics);

      Joint rootJoint = new FloatingJoint("rootJoint", new Vector3d(0, 0, 1.0), this);
      rootJoint.setLink(rootJointLink);
      addRootJoint(rootJoint);
   }
}
