package us.ihmc.moonwalking.models.aaronhopper;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class AaronHopperRobot extends Robot
{
   FloatingPlanarJoint Body;
   PinJoint Hip;
   SliderJoint Knee;
   GroundContactPoint Foot;
   YoVariable q_z;

   public AaronHopperRobot(String name)
   {
      super(name);

      this.setGravity(-9.81);

      Body = new FloatingPlanarJoint("body", this, FloatingPlanarJoint.XZ);
      Link bodyMass = Body();
      Body.setLink(bodyMass);
      this.addRootJoint(Body);

      Hip = new PinJoint("hip", new Vector3d(0, 0, 0), this, Axis.Y);
      Link hipMass = Hip();
      Hip.setLink(hipMass);
      Body.addJoint(Hip);

      Knee = new SliderJoint("knee", new Vector3d(0, 0, -1), this, Axis.Z);
      Link kneeMass = Knee();
      Knee.setLink(kneeMass);
      Hip.addJoint(Knee);

      Foot = new GroundContactPoint("foot", new Vector3d(0, 0, -1), this);
      Knee.addGroundContactPoint(Foot);

      Startup();
   }

   private Link Knee()
   {
      Link Knee = new Link("hip");
      Knee.setMassAndRadiiOfGyration(5, 0.1, 0.08, 0.1);
      Knee.setComOffset(new Vector3d(0, 0, -0.5));
      Graphics3DObject Graphics = new Graphics3DObject();
      Knee.setLinkGraphics(Graphics);
      Graphics.translate(new Vector3d(0, 0, -1));
      Graphics.addCylinder(1, 0.05);

      return Knee;
   }

   private Link Hip()
   {
      Link Hip = new Link("hip");
      Hip.setMassAndRadiiOfGyration(7, 0.1, 0.1, 0.1);
      Hip.setComOffset(new Vector3d(0, 0, -0.5));
      Graphics3DObject Graphics = new Graphics3DObject();
      Hip.setLinkGraphics(Graphics);
      Graphics.translate(new Vector3d(0, 0, -1));
      Graphics.addCylinder(1, 0.1);

      return Hip;
   }

   private Link Body()
   {
      Link Body = new Link("body");
      Body.setMassAndRadiiOfGyration(50, 0.1, 0.2, 0.1);
      Body.setComOffset(new Vector3d(0, 0, 0.3));
      Graphics3DObject Graphics = new Graphics3DObject();
      Body.setLinkGraphics(Graphics);
      Graphics.addCone(0.8, 0.25);

      return Body;
   }

   private void Startup()
   {
      q_z = this.getVariable("q_z");
      q_z.setValueFromLongBits(2);
   }

}
