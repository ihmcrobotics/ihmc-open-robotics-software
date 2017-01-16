package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class Step5SCSRobot extends Robot
{
   /**
    * Variables
    */
   private FloatingPlanarJoint bodyJoint;
   SideDependentList<GroundContactPoint> GCpoints = new SideDependentList<GroundContactPoint>();
   SideDependentList<PinJoint> hipJoints = new SideDependentList<PinJoint>();
   SideDependentList<SliderJoint> kneeJoints = new SideDependentList<SliderJoint>();
   SideDependentList<KinematicPoint> feetPoints = new SideDependentList<KinematicPoint>();
   SideDependentList<KinematicPoint> hipPoints = new SideDependentList<KinematicPoint>();

   private static double cubeL = 0.8, cubeW = 0.8, cubeH = 0.8;
   private static double bodyMass = 20.0, lowerLinkMass = 4.0, upperLinkMass = 7.0;
   private double lowerLinkLength = 0.8, upperLinkLength = 0.9;
   private double lowerLinkRadius = 0.1, upperLinkRadius = 0.15;
   private double legHeight = lowerLinkLength + upperLinkLength;
   private double gcOffset = -lowerLinkLength;
   private double hipOffsetY = cubeL / 2.0;
   private double g = -9.81;

   /**
    * Joints
    */
   public Step5SCSRobot()
   {
      super("v4Robot");
      this.setGravity(g);

      // (1) Body (3DOF = Z + X + Pitch)
      bodyJoint = new FloatingPlanarJoint("body", this);
      bodyJoint.setDynamic(true);
      Link bodyLink = bodyLink();
      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);
      bodyJoint.setCartesianPosition(0.0, 1.15); 
      
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
         bodyJoint.addJoint(hipJoint);

         KinematicPoint hipPoint = new KinematicPoint(robotSide.getSideNameFirstLetter() + "hipPoint", new Vector3d(0.0, 0.0, 0.0), this);
         hipPoints.put(robotSide, hipPoint);
         hipJoint.addKinematicPoint(hipPoint);
         
         if (robotSide == RobotSide.RIGHT) //Start in "splits" position
         {
            hipJoints.get(robotSide).setQ(0.5);
         }
         else
         {
            hipJoints.get(RobotSide.LEFT).setQ(-0.5);
         }
 
         
         /************************************************************/

         SliderJoint kneeJoint = new SliderJoint(robotSide.getSideNameFirstLetter() + "Knee", new Vector3d(0.0, 0.0, -upperLinkLength + 0.4), this, Axis.Z); //TODO change offset depending on height
         kneeJoints.put(robotSide, kneeJoint);
         kneeJoint.setDynamic(true);
         kneeJoint.setLimitStops(-0.4, 0.4, 1e5, 1e4); //TODO change limits depending on initial position. Eg: (0.0, 0.6)
         Link lowerLink = lowerLink(robotSide);
         kneeJoint.setLink(lowerLink);
         hipJoint.addJoint(kneeJoint);

         KinematicPoint footPoint = new KinematicPoint(robotSide.getSideNameFirstLetter() + "footPoint", new Vector3d(0.0, 0.0, -upperLinkLength + 0.4), this);
         feetPoints.put(robotSide, footPoint); //PUT is only used while constructing the list. It is NOT the same as SET, which is used to change the value if needed in the future
         kneeJoint.addKinematicPoint(footPoint);

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
      GroundContactModel groundModel = new LinearGroundContactModel(this, 150000, 15000, 50000.0, 1e5, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      groundModel.setGroundProfile3D(profile);
      this.setGroundContactModel(groundModel);
   }

   /**
    * Links
    */
   private Link bodyLink()
   {

      Link ret = new Link("body");
      ret.setMass(bodyMass);

      // Inertia tensor
      Matrix3d inertiaCube = new Matrix3d();
      inertiaCube = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidBox(cubeL, cubeW, cubeH, bodyMass);
      ret.setMomentOfInertia(inertiaCube);

      // Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(cubeL, cubeW, cubeH, YoAppearance.Glass());

      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, cubeH / 2.0); 
      ret.addCoordinateSystemToCOM(0.4);

      return ret;
   }

   private Link upperLink(RobotSide robotSide)
   {

      Link ret = new Link("upperLink");
      ret.setMass(upperLinkMass);

      // Inertia tensor
      Matrix3d inertiaUpperCylinder = new Matrix3d();
      inertiaUpperCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(upperLinkMass, upperLinkRadius, upperLinkLength, Axis.Z);
      ret.setMomentOfInertia(inertiaUpperCylinder);

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
      Matrix3d inertiaLowerCylinder = new Matrix3d();
      inertiaLowerCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(lowerLinkMass, lowerLinkRadius, lowerLinkLength, Axis.Z);
      ret.setMomentOfInertia(inertiaLowerCylinder);

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

   // ************** BODY
   public double getBodyPositionZ()
   {
      return bodyJoint.q_t2.getDoubleValue();
   }

   public double getBodyVelocityZ()
   {
      return bodyJoint.qd_t2.getDoubleValue();
   }

   public double getBodyPositionX()
   {
      return bodyJoint.q_t1.getDoubleValue();
   }

   public double getBodyVelocityX()
   {
      return bodyJoint.qd_t1.getDoubleValue();
   }

   public double getBodyPitch()
   {
      return bodyJoint.q_rot.getDoubleValue();
   }
   
   // ***************** HIP
   public void setHipTau(RobotSide robotSide, double desiredTauBody)
   {
      hipJoints.get(robotSide).setTau(desiredTauBody);
   }

   public double getHipPitch(RobotSide robotSide) //same as leg pitch
   {
      return hipJoints.get(robotSide).getQ();
   }

   public double getLegLength(RobotSide robotSide)
   {
      double hipX = hipPoints.get(robotSide).getX();
      double footX = feetPoints.get(robotSide).getX();
      double hipZ = hipPoints.get(robotSide).getZ();
      double footZ = feetPoints.get(robotSide).getZ();
      double footHipDistanceX = Math.abs(hipX - footX);
      double footHipDistanceZ = Math.abs(hipZ - footZ);

      double legLength = Math.sqrt(Math.pow(footHipDistanceX, 2) + Math.pow(footHipDistanceZ, 2));

      return legLength;
   }
   
   public double getHipVelocity(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQD();
   }

   // ****************** KNEE
   public void setKneeTau(RobotSide robotSide, double desiredTauKnee)
   {
      kneeJoints.get(robotSide).setTau(desiredTauKnee);
   }

   public double getKneePositionZ(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQ();
   }

   public double getKneeVelocityZ(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQD();
   }

   // ****************** FEET
   public double getFeetDistance()
   {
      return feetPoints.get(RobotSide.LEFT).getX() - feetPoints.get(RobotSide.RIGHT).getX();
   }
}
