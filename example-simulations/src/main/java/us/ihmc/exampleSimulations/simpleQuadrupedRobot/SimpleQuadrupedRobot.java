package us.ihmc.exampleSimulations.simpleQuadrupedRobot;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.Plane;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class SimpleQuadrupedRobot extends Robot
{
   public static final double COORD_LENGTH = 0.1;
   public static final double BASE_X = 0.267, BASE_Y = 0.194, BASE_Z = 0.114;
   public static final double BASE_MASS = 4.713, BASE_Ixx = 0.01683993, BASE_Iyy = 0.056579028, BASE_Izz = 0.064713601;

   public static final double HIP_H = 0.04, HIP_R = 0.046;
   public static final double HIP_MASS = 0.696, HIP_Ixx = 0.000469246, HIP_Iyy = 0.00080749, HIP_Izz = 0.000552929;

   public static final double THIGH_X = 0.2, THIGH_Y = 0.0245, THIGH_Z = 0.034;
   public static final double THIGH_MASS = 1.013, THIGH_Ixx = 0.005529065, THIGH_Iyy = 0.005139339, THIGH_Izz = 0.001367788;

   public static final double CALF_X = 0.2, CALF_Y = 0.016, CALF_Z = 0.016;
   public static final double CALF_MASS = 0.166, CALF_Ixx = 0.002997972, CALF_Iyy = 0.003014022, CALF_Izz = 3.2426e-5;

   public static final double FOOT_R = 0.01;

   private final FloatingJoint floatingBase;
   private final PinJoint FRHip, FRThigh, FRCalf , FLHip, FLThigh, FLCalf; 
   private final PinJoint RRHip, RRThigh, RRCalf, RLHip, RLThigh, RLCalf;
   //   
   private GroundContactPoint gc_foot_FR, gc_foot_FL, gc_foot_RR, gc_foot_RL;

   public SimpleQuadrupedRobot()
   {
      super("SimpleQuadrupedRobot");
      
      
      floatingBase = new FloatingJoint("floatingBase", new Vector3D(0.0, 0.0, 1.0) ,this);
      floatingBase.setLink(baseLink());
      this.addRootJoint(floatingBase);
      
      // Front Right links and joints
      FRHip = new PinJoint("FRHip", new Vector3D(0.183, -0.047, 0.0), this, Axis3D.X);
      FRHip.setLink(FRHipLink());
      floatingBase.addJoint(FRHip);
      
      FRThigh = new PinJoint("FRThigh", new Vector3D(0.0, -0.08505, 0.0), this, Axis3D.Y);
      FRThigh.setLink(FRThighLink());
      FRHip.addJoint(FRThigh);
      
      FRCalf = new PinJoint("FRCalf", new Vector3D(0.0, 0.0, -THIGH_X), this, Axis3D.Y);
      FRCalf.setLink(FRCalfLink());
      FRThigh.addJoint(FRCalf);
      
      // Front Left links and joints
      FLHip = new PinJoint("FLHip", new Vector3D(0.183, 0.047, 0.0), this, Axis3D.X);
      FLHip.setLink(FLHipLink());
      floatingBase.addJoint(FLHip);
      
      FLThigh = new PinJoint("FLThigh", new Vector3D(0.0, 0.08505, 0.0), this, Axis3D.Y);
      FLThigh.setLink(FLThighLink());
      FLHip.addJoint(FLThigh);
      
      FLCalf = new PinJoint("FLCalf", new Vector3D(0.0, 0.0, -THIGH_X), this, Axis3D.Y);
      FLCalf.setLink(FLCalfLink());
      FLThigh.addJoint(FLCalf);
      
      // Rear Right links and joints
      RRHip = new PinJoint("RRHip", new Vector3D(-0.183, -0.047, 0.0), this, Axis3D.X);
      RRHip.setLink(RRHipLink());
      floatingBase.addJoint(RRHip);
      
      RRThigh = new PinJoint("RRThigh", new Vector3D(0.0, -0.08505, 0.0), this, Axis3D.Y);
      RRThigh.setLink(RRThighLink());
      RRHip.addJoint(RRThigh);
      
      RRCalf = new PinJoint("RRCalf", new Vector3D(0.0, 0.0, -THIGH_X), this, Axis3D.Y);
      RRCalf.setLink(RRCalfLink());
      RRThigh.addJoint(RRCalf);
      
      // Rear Left links and joints
      RLHip = new PinJoint("RLHip", new Vector3D(-0.183, 0.047, 0.0), this, Axis3D.X);
      RLHip.setLink(RLHipLink());
      floatingBase.addJoint(RLHip);
      
      RLThigh = new PinJoint("RLThigh", new Vector3D(0.0, 0.08505, 0.0), this, Axis3D.Y);
      RLThigh.setLink(RLThighLink());
      RLHip.addJoint(RLThigh);
      
      RLCalf = new PinJoint("RLCalf", new Vector3D(0.0, 0.0, -THIGH_X), this, Axis3D.Y);
      RLCalf.setLink(RLCalfLink());
      RLThigh.addJoint(RLCalf);
      
      gc_foot_FR = new GroundContactPoint("gc_foot_FR", new Vector3D(0.0, 0.0, -THIGH_X), this);
      FRCalf.addGroundContactPoint(gc_foot_FR);
      
      gc_foot_FL = new GroundContactPoint("gc_foot_FL", new Vector3D(0.0, 0.0, -THIGH_X), this);
      FLCalf.addGroundContactPoint(gc_foot_FL);
      
      gc_foot_RR = new GroundContactPoint("gc_foot_RR", new Vector3D(0.0, 0.0, -THIGH_X), this);
      RRCalf.addGroundContactPoint(gc_foot_RR);
      
      gc_foot_RL = new GroundContactPoint("gc_foot_RL", new Vector3D(0.0, 0.0, -THIGH_X), this);
      RLCalf.addGroundContactPoint(gc_foot_RL);
      
      GroundContactModel groundModel = new LinearGroundContactModel(this, this.getRobotsYoRegistry());
      this.setGroundContactModel(groundModel);
   }

   private Link baseLink()
   {
      Link baseLink = new Link("BaseLink");
      baseLink.setMass(BASE_MASS);
      baseLink.setMomentOfInertia(BASE_Ixx, BASE_Iyy, BASE_Izz);

      Graphics3DObject baseGraphics = new Graphics3DObject();
      baseGraphics.addCoordinateSystem(COORD_LENGTH);
      baseGraphics.addCube(BASE_X, BASE_Y, BASE_Z, YoAppearance.AluminumMaterial());
      baseLink.setLinkGraphics(baseGraphics);

      return baseLink;
   }

   private Link FRHipLink()
   {
      Link FRHipLink = new Link("FRHipLink");
      FRHipLink.setMass(HIP_MASS);
      FRHipLink.setMomentOfInertia(HIP_Ixx, HIP_Iyy, HIP_Izz);

      Graphics3DObject FRHipGraphics = new Graphics3DObject();
      FRHipGraphics.addCoordinateSystem(COORD_LENGTH);
      FRHipGraphics.translate(0.0, 0.0, 0.0);
      FRHipGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      FRHipGraphics.addCylinder(HIP_H, HIP_R, YoAppearance.Aquamarine());
      
      FRHipLink.setLinkGraphics(FRHipGraphics);

      return FRHipLink;
   }

   private Link FRThighLink()
   {
      Link FRThighLink = new Link("FRThighLink");
      FRThighLink.setMass(THIGH_MASS);
      FRThighLink.setMomentOfInertia(THIGH_Ixx, THIGH_Iyy, THIGH_Izz);

      Graphics3DObject FRThighGraphics = new Graphics3DObject();
      FRThighGraphics.addCoordinateSystem(COORD_LENGTH);
      FRThighGraphics.translate(0.0, 0.0, -THIGH_X / 2.0);
      FRThighGraphics.rotate(Math.PI / 2.0, Axis3D.Y); // TODO(Solved): translate first and then rotate
      FRThighGraphics.addCube(THIGH_X, THIGH_Y, THIGH_Z, YoAppearance.Aqua());

      FRThighLink.setLinkGraphics(FRThighGraphics);

      return FRThighLink;
   }

   private Link FRCalfLink()
   {
      Link FRCalfLink = new Link("FRCalfLink");
      FRCalfLink.setMass(CALF_MASS);
      FRCalfLink.setMomentOfInertia(CALF_Ixx, CALF_Iyy, CALF_Izz);

      Graphics3DObject FRCalfGraphics = new Graphics3DObject();
      FRCalfGraphics.addCoordinateSystem(COORD_LENGTH);
      FRCalfGraphics.translate(0.0, 0.0, -CALF_X / 2.0);
      FRCalfGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      FRCalfGraphics.addCube(CALF_X, CALF_Y, CALF_Z, YoAppearance.CadetBlue());

      FRCalfGraphics.translate(CALF_X / 2.0, 0.0, CALF_Z / 2.0);
      FRCalfGraphics.addSphere(FOOT_R, YoAppearance.AliceBlue());

      FRCalfLink.setLinkGraphics(FRCalfGraphics);

      return FRCalfLink;
   }
   
   private Link FLHipLink()
   {
      Link FLHipLink = new Link("FLHipLink");
      FLHipLink.setMass(HIP_MASS);
      FLHipLink.setMomentOfInertia(HIP_Ixx, HIP_Iyy, HIP_Izz);

      Graphics3DObject FLHipGraphics = new Graphics3DObject();
      FLHipGraphics.addCoordinateSystem(COORD_LENGTH);
      FLHipGraphics.translate(0.0, 0.0, 0.0);
      FLHipGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      FLHipGraphics.addCylinder(HIP_H, HIP_R, YoAppearance.Aquamarine());
      
      FLHipLink.setLinkGraphics(FLHipGraphics);

      return FLHipLink;
   }

   private Link FLThighLink()
   {
      Link FLThighLink = new Link("FLThighLink");
      FLThighLink.setMass(THIGH_MASS);
      FLThighLink.setMomentOfInertia(THIGH_Ixx, THIGH_Iyy, THIGH_Izz);

      Graphics3DObject FLThighGraphics = new Graphics3DObject();
      FLThighGraphics.addCoordinateSystem(COORD_LENGTH);
      FLThighGraphics.translate(0.0, 0.0, -THIGH_X / 2.0);
      FLThighGraphics.rotate(Math.PI / 2.0, Axis3D.Y); 
      FLThighGraphics.addCube(THIGH_X, THIGH_Y, THIGH_Z, YoAppearance.Aqua());

      FLThighLink.setLinkGraphics(FLThighGraphics);

      return FLThighLink;
   }

   private Link FLCalfLink()
   {
      Link FLCalfLink = new Link("FLCalfLink");
      FLCalfLink.setMass(CALF_MASS);
      FLCalfLink.setMomentOfInertia(CALF_Ixx, CALF_Iyy, CALF_Izz);

      Graphics3DObject FLCalfGraphics = new Graphics3DObject();
      FLCalfGraphics.addCoordinateSystem(COORD_LENGTH);
      FLCalfGraphics.translate(0.0, 0.0, -CALF_X / 2.0);
      FLCalfGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      FLCalfGraphics.addCube(CALF_X, CALF_Y, CALF_Z, YoAppearance.CadetBlue());

      FLCalfGraphics.translate(CALF_X / 2.0, 0.0, CALF_Z / 2.0);
      FLCalfGraphics.addSphere(FOOT_R, YoAppearance.AliceBlue());

      FLCalfLink.setLinkGraphics(FLCalfGraphics);

      return FLCalfLink;
   }
   
   private Link RRHipLink()
   {
      Link RRHipLink = new Link("RRHipLink");
      RRHipLink.setMass(HIP_MASS);
      RRHipLink.setMomentOfInertia(HIP_Ixx, HIP_Iyy, HIP_Izz);

      Graphics3DObject RRHipGraphics = new Graphics3DObject();
      RRHipGraphics.addCoordinateSystem(COORD_LENGTH);
      RRHipGraphics.translate(0.0, 0.0, 0.0);
      RRHipGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      RRHipGraphics.addCylinder(HIP_H, HIP_R, YoAppearance.Aquamarine());
      
      RRHipLink.setLinkGraphics(RRHipGraphics);

      return RRHipLink;
   }

   private Link RRThighLink()
   {
      Link RRThighLink = new Link("RRThighLink");
      RRThighLink.setMass(THIGH_MASS);
      RRThighLink.setMomentOfInertia(THIGH_Ixx, THIGH_Iyy, THIGH_Izz);

      Graphics3DObject RRThighGraphics = new Graphics3DObject();
      RRThighGraphics.addCoordinateSystem(COORD_LENGTH);
      RRThighGraphics.translate(0.0, 0.0, -THIGH_X / 2.0);
      RRThighGraphics.rotate(Math.PI / 2.0, Axis3D.Y); 
      RRThighGraphics.addCube(THIGH_X, THIGH_Y, THIGH_Z, YoAppearance.Aqua());

      RRThighLink.setLinkGraphics(RRThighGraphics);

      return RRThighLink;
   }

   private Link RRCalfLink()
   {
      Link RRCalfLink = new Link("RRCalfLink");
      RRCalfLink.setMass(CALF_MASS);
      RRCalfLink.setMomentOfInertia(CALF_Ixx, CALF_Iyy, CALF_Izz);

      Graphics3DObject RRCalfGraphics = new Graphics3DObject();
      RRCalfGraphics.addCoordinateSystem(COORD_LENGTH);
      RRCalfGraphics.translate(0.0, 0.0, -CALF_X / 2.0);
      RRCalfGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      RRCalfGraphics.addCube(CALF_X, CALF_Y, CALF_Z, YoAppearance.CadetBlue());

      RRCalfGraphics.translate(CALF_X / 2.0, 0.0, CALF_Z / 2.0);
      RRCalfGraphics.addSphere(FOOT_R, YoAppearance.AliceBlue());

      RRCalfLink.setLinkGraphics(RRCalfGraphics);

      return RRCalfLink;
   }
   
   private Link RLHipLink()
   {
      Link RLHipLink = new Link("RLHipLink");
      RLHipLink.setMass(HIP_MASS);
      RLHipLink.setMomentOfInertia(HIP_Ixx, HIP_Iyy, HIP_Izz);

      Graphics3DObject RLHipGraphics = new Graphics3DObject();
      RLHipGraphics.addCoordinateSystem(COORD_LENGTH);
      RLHipGraphics.translate(0.0, 0.0, 0.0);
      RLHipGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      RLHipGraphics.addCylinder(HIP_H, HIP_R, YoAppearance.Aquamarine());
      
      RLHipLink.setLinkGraphics(RLHipGraphics);

      return RLHipLink;
   }

   private Link RLThighLink()
   {
      Link RLThighLink = new Link("RLThighLink");
      RLThighLink.setMass(THIGH_MASS);
      RLThighLink.setMomentOfInertia(THIGH_Ixx, THIGH_Iyy, THIGH_Izz);

      Graphics3DObject RLThighGraphics = new Graphics3DObject();
      RLThighGraphics.addCoordinateSystem(COORD_LENGTH);
      RLThighGraphics.translate(0.0, 0.0, -THIGH_X / 2.0);
      RLThighGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      RLThighGraphics.addCube(THIGH_X, THIGH_Y, THIGH_Z, YoAppearance.Aqua());

      RLThighLink.setLinkGraphics(RLThighGraphics);

      return RLThighLink;
   }

   private Link RLCalfLink()
   {
      Link RLCalfLink = new Link("RLCalfLink");
      RLCalfLink.setMass(CALF_MASS);
      RLCalfLink.setMomentOfInertia(CALF_Ixx, CALF_Iyy, CALF_Izz);

      Graphics3DObject RLCalfGraphics = new Graphics3DObject();
      RLCalfGraphics.addCoordinateSystem(COORD_LENGTH);
      RLCalfGraphics.translate(0.0, 0.0, -CALF_X / 2.0);
      RLCalfGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      RLCalfGraphics.addCube(CALF_X, CALF_Y, CALF_Z, YoAppearance.CadetBlue());

      RLCalfGraphics.translate(CALF_X / 2.0, 0.0, CALF_Z / 2.0);
      RLCalfGraphics.addSphere(FOOT_R, YoAppearance.AliceBlue());

      RLCalfLink.setLinkGraphics(RLCalfGraphics);

      return RLCalfLink;
   }
}
