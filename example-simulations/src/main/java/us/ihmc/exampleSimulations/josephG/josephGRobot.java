package us.ihmc.exampleSimulations.josephG;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class josephGRobot extends Robot
{
   private final AppearanceDefinition red = YoAppearance.Red();
   private final AppearanceDefinition black = YoAppearance.Black();
   private final AppearanceDefinition blue = YoAppearance.Blue();
   
   private static final double BASE_HEIGHT = 1.0;
   private static final double BASE_RADIUS = 3.0;
   private static final double BASE_MASS = 5.0;
   
   private static final double LINK_RADIUS = 0.1;
   
   private static final double LINK1_LENGTH = 3.0;
   private static final double LINK1_MASS = 1.0;
   private static final double LINK2_LENGTH = 2.0;
   private static final double LINK2_MASS = 0.5;
   
   private double jointVelocity = 3.0;
   private double jointPosition;
   
   public josephGRobot()
   {
      super("JosephRobot");
      
      red.setTransparency(0.0);
      black.setTransparency(0.0);
      blue.setTransparency(0.0);
      
      /*
         Create the base
       */
      Link base = new Link("base");
      Graphics3DObject baseGraphics = new Graphics3DObject();
      baseGraphics.addCylinder(BASE_HEIGHT, BASE_RADIUS, black);
      baseGraphics.addCoordinateSystem(0.3);
      base.setLinkGraphics(baseGraphics);
      this.addStaticLink(base);
     
      
      /*
         Creating yawJoint (root joint)
       */
      PinJoint yawJoint = new PinJoint("yaw", new Vector3D(0.0, 0.0, BASE_HEIGHT), this, Axis3D.Y);
      yawJoint.setQd(jointVelocity);
      yawJoint.setLink(makeLink1());
      this.addRootJoint(yawJoint);
      
      this.jointPosition = yawJoint.getQ();
      
      /*
         Create pitchJoint
       */
      PinJoint pitchJoint = new PinJoint("pitch", new Vector3D(0.0, 0.0, LINK1_LENGTH), this, Axis3D.X);
      pitchJoint.setLink(makeLink2());
      yawJoint.addJoint(pitchJoint);
      
   }
   
   private Link makeLink1() 
   {
      Link link1 = new Link("link1");
      link1.setMomentOfInertia(0.0, 0.0, 2);
      link1.setMass(2.0);
      link1.setComOffset(0.0, 0.0, LINK1_LENGTH/2);
      
      Graphics3DObject link1Graphics = new Graphics3DObject();
      link1Graphics.rotate(Math.PI/2, Axis3D.X);
      link1Graphics.translate(0.0, 0.0, -0.5);
      link1Graphics.addCylinder(1, 0.25, YoAppearance.Red());
      link1Graphics.translate(0.0, 0.0, 0.5);
      link1Graphics.rotate(-Math.PI/2, Axis3D.X);
      link1Graphics.addCylinder(LINK1_LENGTH, LINK_RADIUS, blue);
      link1.setLinkGraphics(link1Graphics);
      
      return link1;
   }
   
   private Link makeLink2()
   {
      Link link2 = new Link("link2");
      link2.setMomentOfInertia(0.0, 0.0, 2);
      link2.setMass(2.0);
      link2.setComOffset(0.0, 0.0, LINK2_LENGTH/2);
      
      Graphics3DObject link2Graphics = new Graphics3DObject();
      link2Graphics.rotate(Math.PI/2, Axis3D.Y);
      link2Graphics.translate(0.0, 0.0, -0.25);
      link2Graphics.addCylinder(0.5, 0.25, YoAppearance.Red());
      link2Graphics.translate(0.0, 0.0, 0.25);
      link2Graphics.rotate(-Math.PI/2, Axis3D.Y);
      link2Graphics.addCylinder(LINK1_LENGTH, LINK_RADIUS, blue);
      link2.setLinkGraphics(link2Graphics);
      
      return link2;
   }
   
   public void setJointVelocity(double velocity)
   {
      this.jointVelocity = velocity;
   }
   public double getJointVelocity()
   {
      return this.jointVelocity;
   }
   
   public void setJointPosition(double angle)
   {
      this.jointPosition = angle;
   }
   public double getJointPosition()
   {
      return this.jointPosition;
   }
   
   
   
}
