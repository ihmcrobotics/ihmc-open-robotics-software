package us.ihmc.simulationConstructionSetTools.util.dataProcessors;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;


public class TwoLinkRobotForTesting extends Robot
{
   private final boolean SHOW_COORDINATE_SYSTEM = true;
   public static final boolean SHOW_MASS_BASED_GRAPHICS = true;

   public static final double linkLength = 0.3;
   public static final double bodyLength = 2.0*linkLength;

   public static final double upperArmMass = 9.0;
   public static double lowerArmMass = 3.0;
   public static double bodyMass = 256.0/2.0;


   private final double activeLegLength = 1.0;

   private final double hopperBodyRadius = activeLegLength / 4.0;

   private final double gravity;


   private final PinJoint upperJoint;
   private final PinJoint elbowJoint;
   private DoubleYoVariable bodyPitch;

   public TwoLinkRobotForTesting()
   {
      super("TwoLink");
      this.gravity = 9.81;

      this.setGravity(0.0, 0.0, -gravity);

      upperJoint = new PinJoint("upper", new Vector3D(0.0, 0.0, 4.0 * linkLength), this, Axis.Y);

      //upperJoint.setCartesianPosition(0.0, activeLegLength + passiveLegLength + footLength + 0.13);

      Link upperArmLink = upperArm();
      upperJoint.setLink(upperArmLink);

      //upperJoint.setDamping(1.0);

      upperJoint.setLimitStops(-2.0, 5.0, 2.0, 1000.0);
      upperJoint.setVelocityLimits(7.0, 1000.0);
      
      upperJoint.setDynamic(true);
      this.addRootJoint(upperJoint);


      elbowJoint = new PinJoint("elbow", new Vector3D(0.0, 0.0, -linkLength), this, Axis.Y);
      upperJoint.addJoint(elbowJoint);
      Link lowerArmLink = lowerArm();
      elbowJoint.setLink(lowerArmLink);

      //elbowJoint.setDamping(1.0);

      elbowJoint.setLimitStops(-4.0, 3.0, 2.0, 1.0);
      elbowJoint.setVelocityLimits(4.0, 1.0);
   }

   public double getGravity()
   {
      return gravity;
   }

   public double getBodyPitch()
   {
      return bodyPitch.getValueAsDouble();
   }


   private Link upperArm()
   {
      Link link = new Link("upperArm");

      Graphics3DObject linkGraphics = new Graphics3DObject();

      link.setLinkGraphics(linkGraphics);

      link.setMass(upperArmMass);
      link.setComOffset(0.0, 0.0, -0.5*linkLength);
      link.setMomentOfInertia(upperArmMass * hopperBodyRadius * hopperBodyRadius, upperArmMass * hopperBodyRadius * hopperBodyRadius, upperArmMass * hopperBodyRadius * hopperBodyRadius);

      if (SHOW_COORDINATE_SYSTEM)
         linkGraphics.addCoordinateSystem(linkLength * 0.3);

      linkGraphics.addCube(0.1*linkLength, 0.1*linkLength, -linkLength, YoAppearance.Red());
      return link;
   }

   public double getElbowAngle()
   {
      return elbowJoint.getQYoVariable().getDoubleValue();
   }

 
   public double getElbowVelocity()
   {
      return elbowJoint.getQDYoVariable().getDoubleValue();
   }

 
   public double getElbowTorque()
   {
      return elbowJoint.getTauYoVariable().getDoubleValue();
   }

 
   public void setElbowTorque(double torque)
   {
      elbowJoint.setTau(torque);
   }
   
   public void setElbowPosition(double value)
   {
      elbowJoint.setQ(value);
   }

   public void setUpperPosition(double value)
   {
      upperJoint.setQ(value);
   }
   
   public void setElbowVelocity(double velocity)
   {
      elbowJoint.setQd(velocity);
   }

   public void setUpperVelocity(double velocity)
   {
      upperJoint.setQd(velocity);
   }

   public void setElbowAcceleration(double value)
   {
      elbowJoint.setQdd(value);
   }

   public void setUpperAcceleration(double value)
   {
      upperJoint.setQdd(value);
   }

   private Link lowerArm()
   {
      Link link = new Link("upperArm");

      Graphics3DObject linkGraphics = new Graphics3DObject();

      link.setLinkGraphics(linkGraphics);

      link.setMass(lowerArmMass);
      link.setComOffset(0.0, 0.0, -0.5*linkLength);
      link.setMomentOfInertia(lowerArmMass * hopperBodyRadius * hopperBodyRadius, lowerArmMass * hopperBodyRadius * hopperBodyRadius, lowerArmMass * hopperBodyRadius * hopperBodyRadius);

      if (SHOW_COORDINATE_SYSTEM)
         linkGraphics.addCoordinateSystem(linkLength * 0.3);

      linkGraphics.addCube(0.1*linkLength, 0.1*linkLength, -linkLength, YoAppearance.White());
      return link;
   }


}

