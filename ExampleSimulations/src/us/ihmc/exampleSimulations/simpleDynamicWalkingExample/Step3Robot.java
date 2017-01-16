package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class Step3Robot extends Robot
{

   /**
    * Variables
    */
   private SliderJoint bodyJoint1;
   private PinJoint bodyJoint2;
   SideDependentList<GroundContactPoint> GCpoints = new SideDependentList<GroundContactPoint>();
   SideDependentList<PinJoint> hipJoints = new SideDependentList<PinJoint>();
   SideDependentList<SliderJoint> kneeJoints = new SideDependentList<SliderJoint>();

   private double cubeL = 0.8, cubeW = 0.8, cubeH = 0.8;
   private double bodyMass = 20.0, lowerLinkMass = 4.0, upperLinkMass = 7.0;
   private double lowerLinkLength = 0.8, upperLinkLength = 0.9;
   private double lowerLinkRadius = 0.1, upperLinkRadius = 0.15;
   private double legHeight = lowerLinkLength + upperLinkLength;
   private double gcOffset = -lowerLinkLength;
   private double hipOffsetY = cubeL / 2.0;
   private double g = -9.81;

   /**
    * Joints
    */
   public Step3Robot()
   {

      super("v3Robot");
      this.setGravity(g); //TODO change 

      // (1) Body (2DOF = Z + Pitch)
      bodyJoint1 = new SliderJoint("bodyZ", new Vector3d(0.0, 0.0, 0.0), this, Axis.Z);
      bodyJoint1.setDynamic(true);
      Link bodyLinkSlider = setNullLink();
      bodyJoint1.setLink(bodyLinkSlider);
      this.addRootJoint(bodyJoint1);
      bodyJoint1.setInitialState(legHeight - 0.4, 0.0); //TODO change height if necessary

      bodyJoint2 = new PinJoint("bodyPitch", new Vector3d(0.0, 0.0, 0.0), this, Axis.Y);
      bodyJoint2.setDynamic(true);
      bodyJoint2.setLimitStops(0.0, 1.0, 1e6, 1e3);
      Link bodyLinkPitch = body();
      bodyJoint2.setLink(bodyLinkPitch);
      bodyJoint1.addJoint(bodyJoint2);

      // (2) Hips, Knees and Feet
      for (RobotSide robotSide : RobotSide.values())
      {

         PinJoint hipJoint = new PinJoint(robotSide.getSideNameFirstLetter() + "Hip", new Vector3d(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0), this,
               Axis.Y);
         hipJoints.put(robotSide, hipJoint);
         hipJoint.setDynamic(true);
         hipJoint.setLimitStops(-1.0, 1.0, 1e6, 1e3);
         Link upperLink = upperLink(robotSide);
         hipJoint.setLink(upperLink);
         bodyJoint2.addJoint(hipJoint);

         /************************************************************/

         SliderJoint kneeJoint = new SliderJoint(robotSide.getSideNameFirstLetter() + "Knee", new Vector3d(0.0, 0.0, -upperLinkLength + 0.4), this, Axis.Z); //TODO change offset depending on height
         kneeJoints.put(robotSide, kneeJoint);
         kneeJoint.setDynamic(true);
         kneeJoint.setLimitStops(-0.3, 0.3, 1e5, 1e4); //TODO change limits depending on initial position. Eg: (0.0, 0.6)
         Link lowerLink = lowerLink(robotSide);
         kneeJoint.setLink(lowerLink);
         hipJoint.addJoint(kneeJoint);

         /*************************************************************/

         GroundContactPoint contactPoint = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "Foot", new Vector3d(0.0, 0.0, gcOffset), this);
         GCpoints.set(robotSide, contactPoint);
         kneeJoints.get(robotSide).addGroundContactPoint(contactPoint);
         Graphics3DObject graphics = kneeJoints.get(robotSide).getLink().getLinkGraphics();
         graphics.identity();
         graphics.translate(0.0, 0.0, gcOffset);
         double radius = 0.03;
         graphics.addSphere(radius, YoAppearance.Orange());

      }

      // (3) Ground Model
      GroundContactModel groundModel = new LinearGroundContactModel(this, 1500, 150, 50000.0, 1e5, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);
   }

   /**
    * Links
    */
   public Link setNullLink()
   {
      Link nullLink = new Link("null");
      nullLink.setMass(0.0);
      nullLink.setMomentOfInertia(0.0, 0.0, 0.0);
      nullLink.setComOffset(0.0, 0.0, 0.0);

      return nullLink;
   }

   private Link body()
   {

      Link ret = new Link("body");
      ret.setMass(bodyMass);

      // Inertia tensor
      double IxxCube = (bodyMass / 12.0) * (Math.pow(cubeW, 2.0) + Math.pow(cubeL, 2.0));
      double IyyCube = (bodyMass / 12.0) * (Math.pow(cubeH, 2.0) + Math.pow(cubeW, 2.0));
      double IzzCube = (bodyMass / 12.0) * (Math.pow(cubeH, 2.0) + Math.pow(cubeL, 2.0));
      ret.setMomentOfInertia(IxxCube, IyyCube, IzzCube);

      // Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(cubeL, cubeW, cubeH, YoAppearance.Glass());

      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, cubeH / 2.0); // TODO would I need to change the CoM location?
      ret.addCoordinateSystemToCOM(0.4);

      return ret;
   }

   private Link upperLink(RobotSide robotSide)
   {

      Link ret = new Link("upperLink");
      ret.setMass(upperLinkMass);

      // Inertia tensor
      double IxxCyl = (upperLinkMass / 3) * (Math.pow(upperLinkLength, 2.0));
      double IzzCyl = IxxCyl;
      ret.setMomentOfInertia(IxxCyl, 0.0, IzzCyl);

      // Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      if (robotSide == RobotSide.RIGHT)
      {
         linkGraphics.addCylinder(-upperLinkLength, upperLinkRadius, YoAppearance.Glass());
      }
      else
      {
         linkGraphics.addCylinder(-upperLinkLength, upperLinkRadius, YoAppearance.AluminumMaterial());
      }
      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, -upperLinkLength / 2.0);
      ret.addCoordinateSystemToCOM(0.4);

      return ret;
   }

   private Link lowerLink(RobotSide robotSide)
   {

      Link ret = new Link("lowerLink");
      ret.setMass(lowerLinkMass);

      // Inertia tensor
      double IxxCyl = (lowerLinkMass / 3) * (Math.pow(lowerLinkLength, 2.0));
      double IzzCyl = IxxCyl;
      ret.setMomentOfInertia(IxxCyl, 0.0, IzzCyl);

      // Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      if (robotSide == RobotSide.RIGHT)
      {
         linkGraphics.addCylinder(-lowerLinkLength, lowerLinkRadius, YoAppearance.Glass());
      }
      else
      {
         linkGraphics.addCylinder(-lowerLinkLength, lowerLinkRadius, YoAppearance.AluminumMaterial());
      }
      
      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, -lowerLinkLength / 2.0);
      ret.addCoordinateSystemToCOM(0.4);

      return ret;
   }

   /**
    * Getter and Setter methods for the controller
    */

   // ************************* Part 1 --> control height of the robot
   public double getBodyPositionZ()
   {
      return bodyJoint1.getQ();
   }

   public double getBodyVelocityZ()
   {
      return bodyJoint1.getQD();
   }

   public void setKneeForce(RobotSide robotSide, double desiredTauKnee)
   {
      kneeJoints.get(robotSide).setTau(desiredTauKnee);
   }

   // ************************* Part 2 --> control body pitch
   public double getBodyPitch()
   {
      return bodyJoint2.getQ();
   }

   public double getBodyPitchVel()
   {
      return bodyJoint2.getQD();
   }

   public void setHipTau(RobotSide robotSide, double desiredTauBody)
   {
      hipJoints.get(robotSide).setTau(desiredTauBody);
   }

   //************************** Part 3 --> control right leg pitch
   public double getLegPitch(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQ();
   }

   public double getLegPitchVel(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQD();
   }

 
   //*************************************
   public double getKneePositionZ(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQ();
   }

   public double getKneeVelocityZ(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQD();
   }

   //*******************************************
   public double getGCHeight(RobotSide robotSide)
   {
      return GCpoints.get(robotSide).getZ();
   }
   
   public boolean isInContactV2(RobotSide robotSide)
   {
      return GCpoints.get(robotSide).getYoFootSwitch().getDoubleValue() > 0.5;
   }
}
