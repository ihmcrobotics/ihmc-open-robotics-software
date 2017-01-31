package us.ihmc.exampleSimulations.collisionExample;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;


public class CollisionExampleRobot extends Robot
{
   private static final long serialVersionUID = 6294679946351726669L;
   public static final double
      M1 = 1.0, R1 = 0.2, GYR1 = 0.6;
   public static final double
      Ixx1 = GYR1 * M1 * R1 * R1, Iyy1 = GYR1 * M1 * R1 * R1, Izz1 = GYR1 * M1 * R1 * R1;

   public static final double
      M2 = 2.0, R2 = 0.2, GYR2 = 0.6;
   public static final double
      Ixx2 = GYR2 * M2 * R2 * R2, Iyy2 = GYR2 * M2 * R2 * R2, Izz2 = GYR2 * M2 * R2 * R2;


   public CollisionExampleRobot()
   {
      super("CollisionExample");
      this.setGravity(0.0, 0.0, 0.0);

      FloatingJoint floatingJoint1 = new FloatingJoint("ball1", "ball1", new Vector3d(0.0, 0.0, 0.0), this);
      Link ball1 = ball1();
      floatingJoint1.setLink(ball1);

      this.addRootJoint(floatingJoint1);

      FloatingJoint floatingJoint2 = new FloatingJoint("ball2", "ball2", new Vector3d(0.0, 0.0, 0.0), this);
      Link ball2 = ball2();
      floatingJoint2.setLink(ball2);

      this.addRootJoint(floatingJoint2);
   }

   public Link ball1()
   {
      Link ret = new Link("ball1");
      ret.setMass(M1);
      ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(R1, YoAppearance.EarthTexture());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   public Link ball2()
   {
      Link ret = new Link("ball2");
      ret.setMass(M2);
      ret.setMomentOfInertia(Ixx2, Iyy2, Izz2);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(R2, YoAppearance.EarthTexture());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

}
