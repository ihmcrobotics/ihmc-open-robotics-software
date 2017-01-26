package us.ihmc.exampleSimulations.doubleMassSpring;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class DoubleMassSpringRobot extends Robot
{
   private static final double mass = 1.0;

   private final SliderJoint x1Joint, x2Joint;

   public DoubleMassSpringRobot()
   {
      super("DoubleMassSpring");

      x1Joint = new SliderJoint("x1", new Vector3d(0.0, 0.0, 0.1), this, Axis.X);
      x2Joint = new SliderJoint("x2", new Vector3d(0.1, 0.0, 0.1), this, Axis.X);

      Link massOneLink = new Link("mass1");
      massOneLink.setMass(mass);

      Link massTwoLink = new Link("mass2");
      massTwoLink.setMass(mass);

      Graphics3DObject linkGraphicsOne = new Graphics3DObject();
      linkGraphicsOne.addCube(0.05, 0.05, 0.05, YoAppearance.Green());
      massOneLink.setLinkGraphics(linkGraphicsOne);

      Graphics3DObject linkGraphicsTwo = new Graphics3DObject();
      linkGraphicsTwo.addCube(0.05, 0.05, 0.05, YoAppearance.Red());
      massTwoLink.setLinkGraphics(linkGraphicsTwo);

      x1Joint.setLink(massOneLink);
      x2Joint.setLink(massTwoLink);

      this.addRootJoint(x1Joint);
      this.addRootJoint(x2Joint);

      // Modes are (1 1) and (-1 1)
      x1Joint.setQ(-0.02);
      x2Joint.setQ(0.02);
   }

   public void setX1Force(double x1Force)
   {
      x1Joint.setTau(x1Force);
   }

   public void setX2Force(double x2Force)
   {
      x2Joint.setTau(x2Force);
   }

   public void addX1Force(double x1Force)
   {
      x1Joint.setTau(x1Joint.getTauYoVariable().getDoubleValue() + x1Force);
   }

   public void addX2Force(double x2Force)
   {
      x2Joint.setTau(x2Joint.getTauYoVariable().getDoubleValue() + x2Force);
   }

   public double getX1()
   {
      return x1Joint.getQYoVariable().getDoubleValue();
   }

   public double getX2()
   {
      return x2Joint.getQYoVariable().getDoubleValue();
   }

   public double getX1Dot()
   {
      return x1Joint.getQDYoVariable().getDoubleValue();
   }

   public double getX2Dot()
   {
      return x2Joint.getQDYoVariable().getDoubleValue();
   }


}
