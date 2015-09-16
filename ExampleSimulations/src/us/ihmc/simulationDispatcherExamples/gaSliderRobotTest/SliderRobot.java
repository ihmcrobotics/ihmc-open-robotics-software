package us.ihmc.simulationDispatcherExamples.gaSliderRobotTest;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class SliderRobot extends Robot
{
   private static final long serialVersionUID = -2219598772288238111L;
   
   private static final double L1 = 1.0, L2 = 1.0, M1 = 1.0, M2 = 2.0, Iyy1 = 0.0, Iyy2 = 0.0, R1 = 0.1, R2 = 0.01;

   public SliderRobot(String[] structuralParameterNames, double[] structuralParameterValues)
   {
      super("SliderRobot");

      if (structuralParameterNames != null)
      {
         System.out.println("Creating Slider Robot with Structural Parameters: ");
         for(int i=0; i<structuralParameterNames.length; i++)
         {
            String name = structuralParameterNames[i];
            double value = structuralParameterValues[i];

            System.out.println(name + ": " + value);
         }
         System.out.println();
      }
      else
      {
         System.out.println("structuralParameterNames == null");
      }
      Joint joint1 = new SliderJoint("joint1", new Vector3d(), this, Axis.X);
      Link link1 = link1();
      joint1.setLink(link1);
      this.addRootJoint(joint1);

      Joint joint2 = new PinJoint("joint2", new Vector3d(0.0, 0.0, .1), this, Axis.Y);
      Link link2 = link2();
      joint2.setLink(link2);
      joint1.addJoint(joint2);
   }

   private Link link1()
   {
      Link ret = new Link("link1");
      ret.setMass(M1);
      ret.setComOffset(0.0, 0.0, 0.0);
      ret.setMomentOfInertia(0.0, 0.0, 0.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      ret.setLinkGraphics(linkGraphics);
      // ret.addCylinder(L1,R1);
      linkGraphics.addCube(1, .5, .1);

      return ret;
   }

   private Link link2()
   {
      Link ret = new Link("link2");
      ret.setMass(M2);
      ret.setComOffset(0.0, 0.0, L2 / 2);
      ret.setMomentOfInertia(0.0, 0.0, 0.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      ret.setLinkGraphics(linkGraphics);
      linkGraphics.addCylinder(L2, R2);

      return ret;
   }
}
