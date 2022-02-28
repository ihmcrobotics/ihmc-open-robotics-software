package us.ihmc.exampleSimulations.lipmWalker;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotDescription.FloatingPlanarJointDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.Plane;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.*;

public class LIPMWalkerRobot
{
   private double bodyXRadius = 0.1, bodyYRadius = 0.2, bodyZRadius = 0.4;
   private double bodyRadiusOfGyrationY = 0.2;
   private double bodyRadiusOfGyrationZ = 0.2;
   private double bodyRadiusOfGyrationX = 0.2;
   private double bodyMass = 30.0;
   private final Robot robot;
   private double hipWidth = 0.3;
   private double thighMass = 0.2;
   private double thighRadiusOfGyrationX = 0.01;
   private double thighRadiusOfGyrationY = 0.01;
   private double thighRadiusOfGyrationZ = 0.01;
   private double thighLength = 0.6;
   private double thighRadius = 0.05;
   private double shinMass = 0.05;
   private double shinRadiusOfGyrationX = 0.01;
   private double shinRadiusOfGyrationY = 0.01;
   private double shinRadiusOfGyrationZ = 0.01;
   private double shinLength = 0.6;
   private double shinRadius = 0.03;

   private final ArrayList<GroundContactPointDescription> gcPoints = new ArrayList<GroundContactPointDescription>(2);

   private final FloatingPlanarJoint bodyJoint;
   private final PinJoint leftHipJoint, rightHipJoint;
   private final SliderJoint leftKneeJoint, rightKneeJoint;
   private final SideDependentList<PinJoint> hipJoints;
   private final SideDependentList<SliderJoint> kneeJoints;
   private final SideDependentList<GroundContactPoint> heelPoints;
   
//   private SimpleMatrix leftlegJacobian, rightlegJacobian;
//   private final SideDependentList<SimpleMatrix> legJacobians;

   public LIPMWalkerRobot()
   {
      RobotDescription description = getRobotDescription();
      robot = new RobotFromDescription(description);

      bodyJoint = (FloatingPlanarJoint) robot.getRootJoints().get(0);

      leftHipJoint = (PinJoint) robot.getJoint("leftHip");
      rightHipJoint = (PinJoint) robot.getJoint("rightHip");

      hipJoints = new SideDependentList<>(leftHipJoint, rightHipJoint);

      leftKneeJoint = (SliderJoint) robot.getJoint("leftKnee");
      rightKneeJoint = (SliderJoint) robot.getJoint("rightKnee");

      kneeJoints = new SideDependentList<>(leftKneeJoint, rightKneeJoint);

      heelPoints = new SideDependentList<GroundContactPoint>();
      
      List<GroundContactPoint> contactPoints = robot.getAllGroundContactPoints();
      for (GroundContactPoint point : contactPoints)
      {
         if (point.getName() == "gc_rheel")
         {
            heelPoints.set(RobotSide.RIGHT, point);
         }
         if (point.getName() == "gc_lheel")
         {
            heelPoints.set(RobotSide.LEFT, point);
         }
      }

      setupInitialConditions();

      LogTools.info("Robot: {}", robot.toString());
   }
   
//   public double getTime()
//   {
//      return robot.getTime();  // GMN: Why do I need to do this??
//   }
   
   public double getMass()
   {
      Point3DBasics comPosition = new Point3D();
      double mass = robot.computeCenterOfMass(comPosition);
      return mass;
   }

   public Point3DReadOnly getCenterOfMassPosition()
   {
      Point3DBasics comPosition = new Point3D();
      robot.computeCenterOfMass(comPosition);
      return comPosition;
   }

   public Vector3DReadOnly getCenterOfMassVelocity()
   {
      Vector3DBasics comVelocity = new Vector3D();
      double mass = robot.computeLinearMomentum(comVelocity);
      comVelocity.scale(1.0 / mass);
      return comVelocity;
   }

   public double getBodyXPosition()
   {
      return this.bodyJoint.getQ_t1().getValue();
   }

   public double getBodyZPosition()
   {
      return this.bodyJoint.getQ_t2().getValue();
   }

   public double getBodyZVelocity()
   {
      return this.bodyJoint.getQd_t2().getValue();
   }

   public double getBodyPitchAngle()
   {
      return this.bodyJoint.getQ_rot().getValue();
   }

   public double getBodyPitchAngularVelocity()
   {
      return this.bodyJoint.getQd_rot().getValue();
   }

   public double getHipAngle(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQ();
   }

   public double getKneeLength(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQ();
   }

   public double getHipVelocity(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQD();
   }

   public double getKneeVelocity(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQD();
   }

   public void setHipTorque(RobotSide robotSide, double torque)
   {
      hipJoints.get(robotSide).setTau(torque);
   }

   public void setKneeForce(RobotSide robotSide, double force)
   {
      kneeJoints.get(robotSide).setTau(force);
   }

   public Point3D getFootPosition(RobotSide robotSide)
   {
      return heelPoints.get(robotSide).getPositionCopy();
   }

   public double getFootZForce(RobotSide robotSide)
   {
      return heelPoints.get(robotSide).getYoForce().getZ();
   }
   
   public boolean getFootSwitch(RobotSide robotSide)
   {
      return (heelPoints.get(robotSide).getYoFootSwitch().getValue() > 0.5);
   }

   public double getKneeForce(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getTau();
   }

   public double getHipTorque(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getTau();
   }
   
//   public DMatrixRMaj getlegJacobian(RobotSide robotSide)
   public DMatrixRMaj getlegJacobian(double th1, double ph2, double L3)
   {
//      double th1 = getBodyPitchAngle();
//      double ph2 = getHipAngle(robotSide);
//      double L3 = Math.abs(getKneeLength(robotSide));

      Matrix3D Cw1 = new Matrix3D(new RotationMatrix(0,th1,0));
      Matrix3D C12 = new Matrix3D(new RotationMatrix(0,ph2,0));
      Matrix3D Cw2 = new Matrix3D(new RotationMatrix(0,th1+ph2,0)); // GMN: don't like, but can't find what I want...
      
      Tuple3DReadOnly Lv2 = new Vector3D(0,0,-L3);
      
      Tuple3DBasics Wv1 = new Vector3D(); // pos foot rt & ewrt body-frame
      C12.transform(Lv2,Wv1);  // Wv1 = C12*Lv2
      
      Tuple3DBasics Jcol1 = new Vector3D(Wv1.getZ(),0,-Wv1.getX()); // skew(Wv1)'*[0;1;0]
      Cw1.transform(Jcol1); // transform to world-frame
      
      Tuple3DBasics Jcol2 = new Vector3D(Lv2.getZ(),0,-Lv2.getX()); // skew(Lv2)'*[0;1;0]
      Cw2.transform(Jcol2); // transform to world-frame
      
      Tuple3DBasics Jcol3 = new Vector3D(0,0,-1);  // -zhat = -[0;0;1]
      Cw2.transform(Jcol3); // transform to world-frame
      
      DMatrixRMaj Jleg = new DMatrixRMaj(2,3);
      Jleg.set(0,0,Jcol1.getX());
      Jleg.set(1,0,Jcol1.getZ());
      Jleg.set(0,1,Jcol2.getX());
      Jleg.set(1,1,Jcol2.getZ());
      Jleg.set(0,2,Jcol3.getX());
      Jleg.set(1,2,Jcol3.getZ());
      
      return Jleg;      
   }

   private void setupInitialConditions()
   {
      bodyJoint.setCartesianPosition(0.0, 0.8);
      bodyJoint.setCartesianVelocity(0.3, 0.0);
      bodyJoint.setRotation(0.0);

      leftHipJoint.setQ(0.0);
      leftKneeJoint.setQ(0.79);
      
      rightHipJoint.setQ(0.0);
      rightKneeJoint.setQ(0.6);
   }

   private RobotDescription getRobotDescription()
   {
      RobotDescription description = new RobotDescription("LIPMWalker");
      FloatingPlanarJointDescription bodyJoint = new FloatingPlanarJointDescription("RootJoint", Plane.XZ);

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRadiusOfGyrationX, bodyRadiusOfGyrationY, bodyRadiusOfGyrationZ);
      bodyJoint.setLink(bodyLink);
      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.addEllipsoid(bodyXRadius, bodyYRadius, bodyZRadius, YoAppearance.AluminumMaterial());
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      PinJointDescription leftHipJoint = new PinJointDescription("leftHip", new Vector3D(0.0, hipWidth / 2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      bodyJoint.addJoint(leftHipJoint);

      PinJointDescription rightHipJoint = new PinJointDescription("rightHip", new Vector3D(0.0, -hipWidth / 2.0, 0.0), new Vector3D(0.0, 1.0, 0.0));
      bodyJoint.addJoint(rightHipJoint);

      LinkDescription leftThigh = createThighLink("leftThigh");
      leftHipJoint.setLink(leftThigh);

      LinkDescription rightThigh = createThighLink("rightThigh");
      rightHipJoint.setLink(rightThigh);

      SliderJointDescription leftKneeJoint = new SliderJointDescription("leftKnee", new Vector3D(), new Vector3D(0.0, 0.0, -1.0));
      SliderJointDescription rightKneeJoint = new SliderJointDescription("rightKnee", new Vector3D(), new Vector3D(0.0, 0.0, -1.0));

      LinkDescription leftShin = createShinLink("leftShin", YoAppearance.Red());
      leftKneeJoint.setLink(leftShin);
      leftHipJoint.addJoint(leftKneeJoint);

      LinkDescription rightShin = createShinLink("rightShin", YoAppearance.Green());
      rightKneeJoint.setLink(rightShin);
      rightHipJoint.addJoint(rightKneeJoint);

      GroundContactPointDescription gc_rheel = new GroundContactPointDescription("gc_rheel", new Vector3D(0.0, 0.0, 0.0));
      GroundContactPointDescription gc_lheel = new GroundContactPointDescription("gc_lheel", new Vector3D(0.0, 0.0, 0.0));

      gcPoints.add(gc_rheel);
      gcPoints.add(gc_lheel);

      leftKneeJoint.addGroundContactPoint(gc_lheel);
      rightKneeJoint.addGroundContactPoint(gc_rheel);

      description.addRootJoint(bodyJoint);
      return description;
   }

   private LinkDescription createShinLink(String shinLink, AppearanceDefinition appearance)
   {
      LinkDescription shinLinkDescription = new LinkDescription(shinLink);
      shinLinkDescription.setMassAndRadiiOfGyration(shinMass, shinRadiusOfGyrationX, shinRadiusOfGyrationY, shinRadiusOfGyrationZ);
      LinkGraphicsDescription shinGraphics = new LinkGraphicsDescription();
      shinGraphics.addCylinder(shinLength, shinRadius, appearance);
      shinGraphics.addSphere(shinRadius, YoAppearance.AluminumMaterial());
      shinLinkDescription.setLinkGraphics(shinGraphics);
      return shinLinkDescription;
   }

   private LinkDescription createThighLink(String thighName)
   {
      LinkDescription thighLinkDescription = new LinkDescription(thighName);
      thighLinkDescription.setMassAndRadiiOfGyration(thighMass, thighRadiusOfGyrationX, thighRadiusOfGyrationY, thighRadiusOfGyrationZ);
      LinkGraphicsDescription thighGraphics = new LinkGraphicsDescription();
      thighGraphics.rotate(Math.PI, Axis3D.Y);
      thighGraphics.addCylinder(thighLength, thighRadius, YoAppearance.AluminumMaterial());
      thighLinkDescription.setLinkGraphics(thighGraphics);
      return thighLinkDescription;
   }

   public double getCenterOfMassXDistanceFromSupportFoot()
   {
      Point3D leftFootPosition = getFootPosition(RobotSide.LEFT);
      Point3D rightFootPosition = getFootPosition(RobotSide.RIGHT);

      double leftForce = getFootZForce(RobotSide.LEFT);
      double rightForce = getFootZForce(RobotSide.RIGHT);

      Point3D loadedFootPosition = leftFootPosition;
      if (Math.abs(rightForce) > Math.abs(leftForce))
      {
         loadedFootPosition = rightFootPosition;
      }

      return getCenterOfMassPosition().getX() - loadedFootPosition.getX();
   }

   public Robot getRobot()
   {
      return robot;
   }

}
