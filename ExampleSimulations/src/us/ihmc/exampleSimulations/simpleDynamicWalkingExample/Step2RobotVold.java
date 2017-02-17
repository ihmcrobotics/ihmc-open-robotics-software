package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class Step2RobotVold extends Robot
{

   /**
    * Variables
    */
   private FloatingJoint bodyJoint;
   private PinJoint hipJoint;
   private SliderJoint kneeJoint;
   
   private double cubeL = 0.8, cubeW = 0.8, cubeH = 0.8;
   private double bodyMass = 20.0, lowerLinkMass = 4.0, upperLinkMass = 7.0;
   private double lowerLinkLength = 0.8, upperLinkLength = 0.9;
   private double lowerLinkRadius = 0.1, upperLinkRadius = 0.15;
   private double legHeight = lowerLinkLength + upperLinkLength;
 
   
   /**
    * Joints
    */
   public Step2RobotVold() 
   {
      super("v2Robot");
      this.setGravity(-9.81);
      
      // Body
      bodyJoint = new FloatingJoint("body", new Vector3D(0.0, 0.0, 0.0), this, true);
      bodyJoint.setDynamic(true);
      Link bodyLink = body();
      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);
      bodyJoint.setPosition(0.0,0.0,legHeight);
      
      // Upper Joint
      hipJoint = new PinJoint("hip", new Vector3D(0.0, 0.0, 0.0), this, Axis.Y);
      hipJoint.setDynamic(true);
      hipJoint.setLimitStops(0.0, 1.0, 1e6, 1e3);;
      Link upperLink = upperLink();
      hipJoint.setLink(upperLink);
      bodyJoint.addJoint(hipJoint);

      // Lower Joint
      kneeJoint = new SliderJoint("knee", new Vector3D(0.0, 0.0, -upperLinkLength), this, Axis.Z);
      kneeJoint.setDynamic(true);
      kneeJoint.setLimitStops(0.0, 0.6, 1e9, 1e2);
      Link lowerLink = lowerLink();
      kneeJoint.setLink(lowerLink);
      hipJoint.addJoint(kneeJoint);
      
      // Ground contact points
      GroundContactPoint contactPoint = new GroundContactPoint("Foot", new Vector3D(0.0, 0.0, -lowerLinkLength), this);
      kneeJoint.addGroundContactPoint(contactPoint);
      
      // Ground Model (If no GroundProfile is specified, then a flat ground will be simulated.)
      GroundContactModel groundModel = new LinearGroundContactModel(this, 1500, 150, 50000.0, 1e5, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);
      
   }
   
   /**
    * Links
    */
   private Link body()
   {

      Link ret = new Link("body");
      ret.setMass(bodyMass);
     
      //Inertia tensor
      double IxxCube = (bodyMass / 12.0) * (Math.pow(cubeW, 2.0) + Math.pow(cubeL, 2.0));
      double IyyCube = (bodyMass / 12.0) * (Math.pow(cubeH, 2.0) + Math.pow(cubeW, 2.0));
      double IzzCube = (bodyMass / 12.0) * (Math.pow(cubeH, 2.0) + Math.pow(cubeL, 2.0));
      ret.setMomentOfInertia(IxxCube, IyyCube, IzzCube);
      
      //Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(cubeL, cubeW, cubeH, YoAppearance.Glass());
      //linkGraphics.addCoordinateSystem(0.5); 
      
      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, cubeH / 2.0); //Move the CoM to the center of the body
      ret.addCoordinateSystemToCOM(0.4);

      return ret;
   }

   private Link upperLink()
   {
      
      Link ret = new Link("upperLink");
      ret.setMass(upperLinkMass);
      
      //Inertia tensor
      double IxxCyl = (upperLinkMass / 3) * (Math.pow(upperLinkLength, 2.0));
      double IzzCyl = IxxCyl;
      ret.setMomentOfInertia(IxxCyl, 0.0, IzzCyl);

      //Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(-upperLinkLength, upperLinkRadius, YoAppearance.Glass());
      //linkGraphics.addCoordinateSystem(0.5); 
      
      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, -upperLinkLength / 2.0);
      ret.addCoordinateSystemToCOM(0.4);
      
      return ret;
   }

   private Link lowerLink()
   {
     
      Link ret = new Link("lowerLink");
      ret.setMass(lowerLinkMass);

      //Inertia tensor
      double IxxCyl = (lowerLinkMass / 3) * (Math.pow(lowerLinkLength, 2.0));
      double IzzCyl = IxxCyl;
      ret.setMomentOfInertia(IxxCyl, 0.0, IzzCyl);

      //Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(-lowerLinkLength, lowerLinkRadius, YoAppearance.Glass());
      //linkGraphics.addCoordinateSystem(0.5); 
      
      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, -lowerLinkLength / 2.0);
      ret.addCoordinateSystemToCOM(0.4);
      
      return ret;
   }
   
   
   /**
    * Getter and Setter methods for the controller
    */
   
   //*************************  Part 1
   public void setKneeForce(double desiredTauKnee)
   {
      this.kneeJoint.setTau(desiredTauKnee);
   }
   
   public double getBodyPositionZ()
   {
      return bodyJoint.q_z.getDoubleValue();
   }
   
   public double getBodyVelocityZ()
   {
      return bodyJoint.qd_z.getDoubleValue();
   }  
   
   //*************************  Part 2
   public double getBodyPitch()
   {
      return bodyJoint.q_pitch.getDoubleValue();
   }
   
   public void setHipForce(double desiredTauHip)
   {
      this.hipJoint.setTau(desiredTauHip);
   }
   
   public double getBodyPitchVel()
   {
      return bodyJoint.qd_wy.getDoubleValue();
   }
   
   
}
