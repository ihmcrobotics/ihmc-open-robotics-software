package us.ihmc.exampleSimulations.RobotArm;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

import javax.vecmath.Vector3d;

public class RobotArm extends Robot
{
   public static final double mass = 181.0;

   public static final double length = 7.6;
   public static final double distance = 1.0*3.0;

   private static final double midAngle = Math.asin(distance/(2.0*length));

   private final SideDependentList<PinJoint> joints = new SideDependentList<>();

   public RobotArm()
   {
      super("BuildingPendulumRobot");

      NullJoint rootJoint = new NullJoint("CeilingJoint", new Vector3d(), this);



      Link ceiling = new Link("link1");
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(5, 5, 0.1);

      ceiling.setLinkGraphics(linkGraphics);
      rootJoint.setLink(ceiling);

      this.addRootJoint(rootJoint);
   }

}
