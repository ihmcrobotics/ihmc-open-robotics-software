package us.ihmc.exampleSimulations.simple3DWalker;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleWalkerRobot extends Robot
{
   private final double initalHipAngle = 0.0;
   private final double initalBodyVelocity = 0.0;
   private double GRAVITY = -9.81;
   private FloatingJoint bodyJoint;
   private SideDependentList<PinJoint> hipRollJoints = new SideDependentList<PinJoint>();
   private SideDependentList<PinJoint> hipPitchJoints = new SideDependentList<PinJoint>();
   private SideDependentList<PinJoint> hipYawJoints = new SideDependentList<PinJoint>();
   private SideDependentList<SliderJoint> kneeJoints = new SideDependentList<SliderJoint>();
   private SideDependentList<List<GroundContactPoint>> gCpoints = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private SideDependentList<PinJoint> anklePitchJoints = new SideDependentList<>();
   private SideDependentList<PinJoint> ankleRollJoints = new SideDependentList<>();
   private SideDependentList<PinJoint> ankleYawJoints = new SideDependentList<>();
   private SideDependentList<YoGraphicVector> reactionForceVizs = new SideDependentList<>();


   private GroundContactPoint gcHeelL;
   private GroundContactPoint gcHeelR;
   private GroundContactPoint gcToeL;
   private GroundContactPoint gcToeR;
   private SideDependentList<GroundContactPointBasedWrenchCalculator> gcPointBasedWrenchCalculators = new SideDependentList<>();
   private SideDependentList<YoFixedFrameWrench> yoWrenchs = new SideDependentList<>();

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame bodyZUpFrame;
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameVector3D bodyVelocity;
   private final YoFrameVector3D bodyAcceleration;



   //private PinJointDescription
   private double bodyMass = 10.0, lowerLinkMass = 0.1, upperLinkMass = 0.2;
   public final double lowerLinkLength = 0.8, upperLinkLength = 0.7;
   private double lowerLinkRadius = 0.04, upperLinkRadius = 0.05;
   private double legHeight = lowerLinkLength + upperLinkLength;
   private double gcOffset = -lowerLinkLength;
   private double bodyLength = 0.3, bodyWidth = 0.3, bodyHeight = 0.2;
   private double hipOffsetY = bodyWidth / 2.0;
   private double maxLegExtension = lowerLinkLength;
   private double footZMin = 0;
   private double footZmax = -0.01;
   private double footY = 0.3;
   private double footX = 0.3;
   private double footOffsetPercent = 0;
   private double footForward = (footX * footOffsetPercent);
   private final Vector3D bodyOffset = new Vector3D(0.0, 0.0, 0.0);
   private final boolean withFeet;
   private final boolean withYaw;

   public final double nominalHeight = upperLinkLength + lowerLinkLength / 2.0;

   public SimpleWalkerRobot(boolean withFeet, boolean withYaw)
   {
      super("Walker3D");
      this.setGravity(GRAVITY);
      this.withFeet=withFeet;
      this.withYaw=withYaw;

      bodyJoint = new FloatingJoint("body", bodyOffset, this, true);
      Link bodyLink = getBodyLink();

      bodyVelocity = new YoFrameVector3D("bodyVelocity", worldFrame, getRobotsYoRegistry());
      bodyAcceleration = new YoFrameVector3D("bodyAcceleration", worldFrame, getRobotsYoRegistry());

      Quaternion bodyRotation = new Quaternion();
      bodyFrame = new ReferenceFrame("bodyFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            bodyJoint.getRotationToWorld(bodyRotation);
            transformToParent.getRotation().set(bodyRotation);
         }
      };
      bodyZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), bodyFrame,"bodyZUpFrame");
      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      for (RobotSide robotSide : RobotSide.values())
      {
         PinJoint hipPitchJoint = new PinJoint(robotSide.getSideNameFirstLetter() + "HipPitch", new Vector3D(0.0, 0.0*robotSide.negateIfRightSide(hipOffsetY), 0.0),
                                               this, Axis3D.Y);
         hipPitchJoints.put(robotSide, hipPitchJoint);
         hipPitchJoint.setDynamic(true);
         hipPitchJoint.setLimitStops(-Math.PI, Math.PI, 1e6, 1e3);
         Link trunnionLink = trunnionLink(robotSide);
         hipPitchJoint.setLink(trunnionLink);
         bodyJoint.addJoint(hipPitchJoint);

         if(withYaw)
         {
            PinJoint hipYawJoint = new PinJoint(robotSide.getSideNameFirstLetter() + "HipYaw",
                                                new Vector3D(0.0, 0.0* robotSide.negateIfRightSide(hipOffsetY), 0.0), this, Axis3D.Z);
            hipYawJoints.put(robotSide, hipYawJoint);
            hipYawJoint.setDynamic(true);
            hipYawJoint.setLimitStops(-Math.PI, Math.PI, 1e6, 1e3);
            hipYawJoint.setLink(trunnionLink);
            hipPitchJoint.addJoint(hipYawJoint);

         }
         PinJoint hipRollJoint = new PinJoint(robotSide.getSideNameFirstLetter() + "HipRoll", new Vector3D(0.0, 0.0*robotSide.negateIfRightSide(hipOffsetY), 0.0),
                                              this, Axis3D.X);
         hipRollJoints.put(robotSide, hipRollJoint);
         hipRollJoint.setDynamic(true);
         hipRollJoint.setLimitStops(-Math.PI, Math.PI, 1e6, 1e3);
         Link upperLink = upperLink(robotSide);
         hipRollJoint.setLink(upperLink);
         if(withYaw)
         {
            hipYawJoints.get(robotSide).addJoint(hipRollJoint);
         }
         else
         {
            hipPitchJoint.addJoint(hipRollJoint);
         }

         /************************************************************/

         SliderJoint kneeJoint = new SliderJoint(robotSide.getSideNameFirstLetter() + "Knee", new Vector3D(0.0, 0.0, -upperLinkLength), this,
                                                 new Vector3D(0.0, 0.0, -1.0)); //TODO change offset depending on height
         kneeJoints.put(robotSide, kneeJoint);
         kneeJoint.setDynamic(true);
         kneeJoint.setLimitStops(0.0, maxLegExtension, 1e5, 1e4); //TODO change limits depending on initial position. Eg: (0.0, 0.6)
         Link lowerLink = lowerLink(robotSide);
         kneeJoint.setLink(lowerLink);
         hipRollJoint.addJoint(kneeJoint);

         /*************************************************************/

         if (!withFeet)
         {
            GroundContactPoint contactPoint = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "Foot", new Vector3D(0.0, 0.0, 0.0), this);
            gCpoints.get(robotSide).add(contactPoint);
            kneeJoints.get(robotSide).addGroundContactPoint(contactPoint);
            Graphics3DObject graphics = kneeJoints.get(robotSide).getLink().getLinkGraphics();
            graphics.identity();
            graphics.translate(0.0, 0.0, 0.0);
            double radius = 1.5 * lowerLinkRadius;
            graphics.addSphere(radius, YoAppearance.Orange());
         }

         else
         {


            PinJoint anklePitchJoint = new PinJoint(robotSide.getSideNameFirstLetter() + "AnklePitch", new Vector3D(0.0, 0.0, 0.0), this, Axis3D.Y);
            anklePitchJoints.put(robotSide, anklePitchJoint);
            anklePitchJoint.setDynamic(true);
            anklePitchJoint.setLimitStops(-0.15*Math.PI, 0.15*Math.PI, 1e6, 1e3);
            anklePitchJoint.setLink(trunnionLink);
            kneeJoint.addJoint(anklePitchJoint);

            if (withYaw)
            {
               PinJoint ankleYawJoint = new PinJoint(robotSide.getSideNameFirstLetter() + "AnkleYaw", new Vector3D(0.0, 0.0, 0.0), this, Axis3D.Z);
               ankleYawJoints.put(robotSide, ankleYawJoint);
               ankleYawJoint.setDynamic(true);
               ankleYawJoint.setLimitStops(-0.2 * Math.PI, 0.2 * Math.PI, 1e6, 1e3);
               ankleYawJoint.setLink(trunnionLink);
               anklePitchJoint.addJoint(ankleYawJoint);
            }

            PinJoint ankleRollJoint = new PinJoint(robotSide.getSideNameFirstLetter() + "AnkleRoll", new Vector3D(0.0, 0.0, 0.0),
                                                   this, Axis3D.X);
            ankleRollJoints.put(robotSide, ankleRollJoint);
            ankleRollJoint.setDynamic(true);
            ankleRollJoint.setLimitStops(-0.15*Math.PI, 0.15*Math.PI, 1e6, 1e3);
            if (withYaw)
            {
               ankleYawJoints.get(robotSide).addJoint(ankleRollJoint);
            }
            else
            {
               anklePitchJoint.addJoint(ankleRollJoint);
            }

            gcHeelL = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcHeelL",
                                                                new Vector3D(0.5 * footX, -0.5 * footY, footZMin), this);


            gcHeelR = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcHeelR",
                                                                new Vector3D(-0.5 * footX, -0.5 * footY, footZMin), this);


            gcToeL = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcToeL",
                                                               new Vector3D(-0.5 * footX, 0.5 * footY, footZMin), this);


            gcToeR = new GroundContactPoint(robotSide.getSideNameFirstLetter() + "gcToeR",
                                                               new Vector3D(0.5 * footX, 0.5 * footY, footZMin), this);



            gCpoints.get(robotSide).add(gcHeelL);
            gCpoints.get(robotSide).add(gcHeelR);
            gCpoints.get(robotSide).add(gcToeL);
            gCpoints.get(robotSide).add(gcToeR);

            ankleRollJoint.addGroundContactPoint(gcHeelL);
            ankleRollJoint.addGroundContactPoint(gcHeelR);
            ankleRollJoint.addGroundContactPoint(gcToeL);
            ankleRollJoint.addGroundContactPoint(gcToeR);

            JointWrenchSensor ankleWrenchSensor = new JointWrenchSensor(robotSide.getSideNameFirstLetter() + "AnkleWrenchSensor", new Vector3D(), this);
            anklePitchJoint.addJointWrenchSensor(ankleWrenchSensor);


            GroundContactPointBasedWrenchCalculator gcPointBasedWrenchCalculator = new GroundContactPointBasedWrenchCalculator(robotSide.getSideNameFirstLetter()+"gcWrench", gCpoints.get(robotSide), ankleRollJoint, new RigidBodyTransform(), getRobotsYoRegistry());
            gcPointBasedWrenchCalculators.put(robotSide,gcPointBasedWrenchCalculator);
            YoFixedFrameWrench yoWrench = new YoFixedFrameWrench(robotSide.getSideNameFirstLetter()+ "wrench",ReferenceFrame.getWorldFrame(),ReferenceFrame.getWorldFrame(),getRobotsYoRegistry());
            yoWrenchs.put(robotSide,yoWrench);

            YoGraphicVector reactionForceViz = new YoGraphicVector(robotSide.getSideNameFirstLetter()+"ReactionForceViz",  gCpoints.get(robotSide).get(0).getYoPosition(), yoWrenchs.get(robotSide).getLinearPart(),
                                                                   YoAppearance.Red());
            reactionForceVizs.put(robotSide,reactionForceViz);

            YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
            yoGraphicsListRegistry.registerYoGraphic(robotSide.getSideNameFirstLetter()+"reactionviz", reactionForceViz);
            yoGraphicsListRegistries.add(yoGraphicsListRegistry);

            Link foot = foot(robotSide,gCpoints.get(robotSide));
            ankleRollJoint.setLink(foot);

         }


      }

      double groundKxy = 1e4;
      double groundBxy = 1e2;
      double groundKz = 80.0;
      double groundBz = 500.0;
      LinearGroundContactModel ground = new LinearGroundContactModel(this, groundKxy, groundBxy, groundKz, groundBz, this.getRobotsYoRegistry());



      this.setGroundContactModel(ground);

      initialize();
   }

   private void initialize()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         kneeJoints.get(robotSide).getQYoVariable().set(maxLegExtension / 2.0);

         double sign = robotSide.negateIfLeftSide(1.0);
         hipPitchJoints.get(robotSide).getQYoVariable().set(initalHipAngle * sign);
         hipRollJoints.get(robotSide).getQYoVariable().set(initalHipAngle * sign);
      }

      bodyJoint.getQdx().set(initalBodyVelocity);
      bodyJoint.getQdy().set(initalBodyVelocity);
      this.setRobotXYZ(0.0, 0.0, nominalHeight);

   }

   public void setRobotXYZ(double x, double y, double z)
   {
      bodyJoint.getQx().set(x);
      bodyJoint.getQy().set(y);
      bodyJoint.getQz().set(z);
   }

   private Link trunnionLink(RobotSide robotSide)
   {

      Link ret = new Link(robotSide.getSideNameFirstLetter()+"trunnionLink");
      ret.setMass(upperLinkMass);

      // Inertia tensor
      ret.setMomentOfInertia(0.0, 0.0, 0.0);
      ret.setComOffset(0.0, 0.0, 0.0);
      //ret.addCoordinateSystemToCOM(0.4);

      return ret;
   }

   private Link upperLink(RobotSide robotSide)
   {

      Link ret = new Link(robotSide.getSideNameFirstLetter()+"upperLink");
      ret.setMass(upperLinkMass);

      // Inertia tensor
      double IxxCyl = (upperLinkMass / 3) * (Math.pow(upperLinkLength, 2.0));
      double IzzCyl = IxxCyl;
      // NOT SURE:
      double IyyCyl = IxxCyl;
      ret.setMomentOfInertia(IxxCyl, IyyCyl, IzzCyl);

      // Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      if (robotSide == RobotSide.RIGHT)
      {
         linkGraphics.addCylinder(-upperLinkLength, upperLinkRadius, YoAppearance.Pink());
      }
      else
      {
         linkGraphics.addCylinder(-upperLinkLength, upperLinkRadius, YoAppearance.CadetBlue());
      }
      ret.setLinkGraphics(linkGraphics);
      ret.setComOffset(0.0, 0.0, -upperLinkLength / 2.0);
      //ret.addCoordinateSystemToCOM(0.4);

      return ret;
   }

   private Link lowerLink(RobotSide robotSide)
   {

      Link ret = new Link(robotSide.getSideNameFirstLetter()+"lowerLink");
      ret.setMass(lowerLinkMass);

      // Inertia tensor
      double IxxCyl = (lowerLinkMass / 3.0) * (Math.pow(lowerLinkLength, 2.0));
      double IzzCyl = IxxCyl;
      // NOT SURE:
      double IyyCyl = IxxCyl;
      ret.setMomentOfInertia(IxxCyl, IyyCyl, IzzCyl);
      // Graphics
      Graphics3DObject linkGraphics = new Graphics3DObject();
      //linkGraphics.addCoordinateSystem(0.4);
      if (robotSide == RobotSide.RIGHT)
      {
         linkGraphics.addCylinder(lowerLinkLength, lowerLinkRadius, YoAppearance.Red());
      }
      else
      {
         linkGraphics.addCylinder(lowerLinkLength, lowerLinkRadius, YoAppearance.Blue());
      }

      ret.setLinkGraphics(linkGraphics);
      //ret.addCoordinateSystemToCOM(0.4);
      return ret;
   }

   private Link foot(RobotSide robotSide, List<GroundContactPoint> groundContactPoints)
   {
      Link ret = new Link(robotSide.getSideNameFirstLetter() + "foot");

      ret.setMass(0.05);
      ret.setMomentOfInertia(0.04, 0.04, 0.02);
      ret.setComOffset(0.0, 0.0, -0.0309);



      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      //linkGraphics.translate(footX * (0.5 - footOffsetPercent), 0.0, footZMin);
      linkGraphics.translate(0.0,0.0,footZMin);
      linkGraphics.addCube(footX, footY, 0.04 - footZMin);

      for (int i=0;i<groundContactPoints.size();i++)
      {
         linkGraphics.identity();
         linkGraphics.translate(groundContactPoints.get(i).getOffsetCopy());
         linkGraphics.addSphere(0.01,YoAppearance.Orange());
      }
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   public double getKneePosition(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQYoVariable().getDoubleValue();
   }

   public double getLegLenght(RobotSide robotSide) { return upperLinkLength + getKneePosition(robotSide); }

   public double getKneeVelocity(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQDYoVariable().getDoubleValue();
   }

   public double getHipPitchPosition(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getQYoVariable().getDoubleValue();
   }

   public double getHipPitchVelocity(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getQDYoVariable().getDoubleValue();
   }

   public double getHipRollPosition(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide).getQYoVariable().getDoubleValue();
   }

   public double getHipRollVelocity(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide).getQDYoVariable().getDoubleValue();
   }

   public double getHipYawPosition(RobotSide robotSide)
   {
      return hipYawJoints.get(robotSide).getQYoVariable().getDoubleValue();
   }

   public double getHipYawVelocity(RobotSide robotSide)
   {
      return hipYawJoints.get(robotSide).getQDYoVariable().getDoubleValue();
   }

   public double getAnklePitchPosition(RobotSide robotSide) { return anklePitchJoints.get(robotSide).getQYoVariable().getDoubleValue();}

   public double getAnklePitchVelocity(RobotSide robotSide) { return anklePitchJoints.get(robotSide).getQDYoVariable().getDoubleValue();}

   public double getAnkleRollPosition(RobotSide robotSide) { return ankleRollJoints.get(robotSide).getQYoVariable().getDoubleValue();}

   public double getAnkleRollVelocity(RobotSide robotSide) { return ankleRollJoints.get(robotSide).getQDYoVariable().getDoubleValue();}

   public double getAnkleYawPosition(RobotSide robotSide) { return ankleYawJoints.get(robotSide).getQYoVariable().getDoubleValue();}

   public double getAnkleYawVelocity(RobotSide robotSide) { return ankleYawJoints.get(robotSide).getQDYoVariable().getDoubleValue();}

   YoDouble xInWorldToPack = new YoDouble( "AnkleX", getRobotsYoRegistry());
   YoDouble yInWorldToPack = new YoDouble("AnkleY", getRobotsYoRegistry());
   YoDouble zInWorldToPack = new YoDouble("AnkleZ", getRobotsYoRegistry());

   public double getAnklePositionInWorldX(RobotSide robotSide)
   {
      ankleRollJoints.get(robotSide).getXYZToWorld(xInWorldToPack,yInWorldToPack,zInWorldToPack);
      return xInWorldToPack.getDoubleValue();
   }

   public double getAnklePositionInWorldY(RobotSide robotSide)
   {
      ankleRollJoints.get(robotSide).getXYZToWorld(xInWorldToPack,yInWorldToPack,zInWorldToPack);
      return yInWorldToPack.getDoubleValue();
   }


   public void setKneeTorque(RobotSide robotSide, double torque)
   {
      kneeJoints.get(robotSide).getTauYoVariable().set(torque);
   }

   public double getKneeTorque(RobotSide robotside)
   {
      return kneeJoints.get(robotside).getTauYoVariable().getDoubleValue();
   }

   public void setHipPitchTorque(RobotSide robotSide, double torque)
   {
      hipPitchJoints.get(robotSide).getTauYoVariable().set(torque);
   }

   public double getHipPitchTorque(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide).getTauYoVariable().getDoubleValue();
   }

   public void setHipRollTorque(RobotSide robotSide, double torque)
   {
      hipRollJoints.get(robotSide).getTauYoVariable().set(torque);
   }

   public double getHipRollTorque(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide).getTauYoVariable().getDoubleValue();
   }

   public void setHipYawTorque(RobotSide robotSide, double torque)
   {
      hipYawJoints.get(robotSide).getTauYoVariable().set(torque);
   }

   public double getHipYawTorque(RobotSide robotSide)
   {
      return hipYawJoints.get(robotSide).getTauYoVariable().getDoubleValue();
   }

   public void setAnklePitchTorque(RobotSide robotSide, double torque) { anklePitchJoints.get(robotSide).getTauYoVariable().set(torque); }

   public double getAnklePitchTorque(RobotSide robotSide) { return anklePitchJoints.get(robotSide).getTauYoVariable().getDoubleValue();}

   public void setAnkleRollTorque(RobotSide robotSide, double torque) { ankleRollJoints.get(robotSide).getTauYoVariable().set(torque); }

   public double getAnkleRollTorque(RobotSide robotSide) { return ankleRollJoints.get(robotSide).getTauYoVariable().getDoubleValue();}

   public void setAnkleYawTorque(RobotSide robotSide, double torque) { ankleYawJoints.get(robotSide).getTauYoVariable().set(torque); }

   public double getAnkleYawTorque(RobotSide robotSide) { return ankleYawJoints.get(robotSide).getTauYoVariable().getDoubleValue();}

   public double getFootSizeX() { return footX;}

   public double getFootSizeY() { return footY;}

   public boolean isFootOnGround(RobotSide robotSide)
   {
      for (int i = 0; i < gCpoints.get(robotSide).size(); i++)
      {
         if (gCpoints.get(robotSide).get(i).isInContact())
            return true;
      }
      return false;
   }

   public SideDependentList<List<GroundContactPoint>> getgCpoints()
   {
      return gCpoints;
   }

   public double getBodyYaw()
   {
      return bodyJoint.getYawPitchRoll()[0];
   }

   public double getBodyPitch()
   {
      return bodyJoint.getYawPitchRoll()[1];
   }

   public double getBodyRoll()
   {
      return bodyJoint.getYawPitchRoll()[2];
   }

   public double getBodyYawVelocity()
   {
      return bodyJoint.getAngularVelocityInBody().getZ();
   }

   public double getBodyPitchVelocity()
   {
      return bodyJoint.getAngularVelocityInBody().getY();
   }

   public double getBodyRollVelocity()
   {
      return bodyJoint.getAngularVelocityInBody().getX();
   }

   public double getBodyHeight()
   {
      return bodyJoint.getQz().getDoubleValue();
   }

   public double getZ0()
   {
      double z0 = 1.12;
      return z0;
   }
   public double getGravity() { return GRAVITY;}

   public double getBodyPositionX(){ return  bodyJoint.getQx().getDoubleValue();}

   public double getBodyPositionY(){ return bodyJoint.getQy().getDoubleValue();}

   public double getBodyHeightVelocity()
   {
      return bodyVelocity.getZ();
   }

   public double getBodyVelocityX()
   {
      return bodyVelocity.getX();
   }

   public double getBodyVelocityY()
   {
      return bodyVelocity.getY();
   }

   public double getBodyHeightVelocityInWorld()
   {
      return bodyJoint.getQdz().getDoubleValue();
   }

   public double getBodyVelocityInWorldX()
   {
      return bodyJoint.getQdx().getDoubleValue();
   }

   public double getBodyVelocityYInWorld()
   {
      return bodyJoint.getQdy().getDoubleValue();
   }

   public double getBodyAccelerationX() { return bodyAcceleration.getX();}

   public double getBodyAccelerationY() { return bodyAcceleration.getY();}

   public double getBodyAccelerationXInWorld() { return bodyJoint.getQddx().getDoubleValue();}

   public double getBodyAccelerationYInWorld() { return bodyJoint.getQddy().getDoubleValue();}

   public double getBodyMass() { return bodyMass;}


   public boolean withFeet() { return withFeet;}
   public boolean withYaw() { return withYaw;}

   public Joint getAnkleJoint(RobotSide robotSide)
   {
      return ankleRollJoints.get(robotSide);
   }



   private Link getBodyLink()
   {
      Link body = new Link("body");
      body.setMassAndRadiiOfGyration(bodyMass, bodyLength, bodyWidth, bodyHeight);
      //body.setMomentOfInertia((1/12)*bodyMass*(Math.pow(bodyWidth,2)+Math.pow(bodyHeight,2)), (1/12)*bodyMass*(Math.pow(bodyLength,2)+Math.pow(bodyHeight,2)), (1/12)*bodyMass*(Math.pow(bodyWidth,2)+Math.pow(bodyLength,2)));
      //body.setMomentOfInertia(0.15,0.15,0.20);

      body.setComOffset(new Vector3D(0.0, 0.0, 0.0));
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.addCube(bodyLength, bodyWidth, bodyHeight, YoAppearance.AliceBlue());
      graphics.addCoordinateSystem(0.6);
      body.setLinkGraphics(graphics);
      return body;
   }

   private final FrameVector3D bodyVelocityInWorld = new FrameVector3D();
   private final FrameVector3D bodyAccelerationInWorld = new FrameVector3D();

   @Override
   public void update()
   {
      super.update();

      bodyFrame.update();
      bodyZUpFrame.update();

      bodyVelocityInWorld.setToZero(ReferenceFrame.getWorldFrame());
      bodyJoint.getVelocity(bodyVelocityInWorld);
      bodyVelocityInWorld.changeFrame(bodyZUpFrame);

      bodyVelocity.set(bodyVelocityInWorld.getX(), bodyVelocityInWorld.getY(), bodyVelocityInWorld.getZ());

      bodyAccelerationInWorld.setToZero(ReferenceFrame.getWorldFrame());
      bodyJoint.getLinearAcceleration(bodyAccelerationInWorld);
      bodyAccelerationInWorld.changeFrame(bodyZUpFrame);

      bodyAcceleration.set(bodyAccelerationInWorld.getX(), bodyAccelerationInWorld.getY(), bodyAccelerationInWorld.getZ());



      if (withFeet)
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            gcPointBasedWrenchCalculators.get(robotSide).calculate();
            DMatrixRMaj wrenchMatrix = gcPointBasedWrenchCalculators.get(robotSide).getWrench();
            Wrench wrench = new Wrench(worldFrame, worldFrame, wrenchMatrix);
            this.yoWrenchs.get(robotSide).set(wrench);
            this.yoWrenchs.get(robotSide).scale(0.01);
            reactionForceVizs.get(robotSide).set(gCpoints.get(robotSide).get(0).getYoPosition(), yoWrenchs.get(robotSide).getLinearPart());
         }
      }

   }

}
