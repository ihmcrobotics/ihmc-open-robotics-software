package us.ihmc.exampleSimulations.recursivePendulum;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class RecursivePendulumRobot extends Robot
{
   
   private static final long serialVersionUID = -6253424203892567345L;

   private static final int LEVELS = 5;

   private static final double
      L = 0.4, R = 0.03, DENSITY = 1000.0;


   public RecursivePendulumRobot()
   {
      super("RecursivePendulum");
      this.addRootJoint(recursiveJoint(1, 0.0, 3 * L));
   }

   private int jointNum = 0;

   private PinJoint recursiveJoint(int level, double yOffset, double zOffset)
   {
      if (level > LEVELS)
         return null;

      PinJoint parentJoint = joint_n(jointNum, yOffset, zOffset);
      Link link = link_n(jointNum, L / level, R / level);
      parentJoint.setLink(link);
      jointNum++;

      PinJoint child1 = recursiveJoint(level + 1, 3.0 * R / (Math.pow(2, level)), -L / level);
      PinJoint child2 = recursiveJoint(level + 1, -3.0 * R / (Math.pow(2, level)), -L / level);

      if (child1 != null)
         parentJoint.addJoint(child1);
      if (child2 != null)
         parentJoint.addJoint(child2);

      return parentJoint;
   }

   private PinJoint joint_n(int joint_num, double yOffset, double zOffset)
   {
      PinJoint ret = new PinJoint("joint" + joint_num, new Vector3d(0.0, yOffset, zOffset), this, Axis.Y);
      ret.setInitialState(Math.random() - 0.5, 0.0);

      return ret;
   }

   private Link link_n(int link_num, double length, double radius)
   {
      Link ret = new Link("link" + link_num);

      double mass = Math.PI * radius * radius * length * DENSITY;
      double Iyy = 0.66 * mass * length * length;

      ret.setMass(mass);
      ret.setComOffset(0.0, 0.0, -length / 2.0);
      ret.setMomentOfInertia(0.0, Iyy, 0.0);
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -length);
      linkGraphics.addCylinder(length, radius);
      ret.setLinkGraphics(linkGraphics);
      
      return ret;
   }

}
