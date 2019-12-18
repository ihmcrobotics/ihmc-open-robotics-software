package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.variable.YoDouble;

public class SphereRobot extends SCSRobotFromInverseDynamicsRobotModel
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double mass = 1.0;
   private static final double Ixx1 = 0.1, Iyy1 = 0.1, Izz1 = 0.1;
   private static final Matrix3D inertia = new Matrix3D(Ixx1, 0.0, 0.0, 0.0, Iyy1, 0.0, 0.0, 0.0, Izz1);

   private static final RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
   private static final SixDoFJoint floatingJoint = new SixDoFJoint("floatingJoint", elevator);
   private static final RigidBodyBasics body = new RigidBody("body", floatingJoint, inertia, mass, new Vector3D());

   private static final double radius = 0.1;


   private final ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);

   private final double totalMass;

   public SphereRobot(String name, double gravity, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(name, floatingJoint);

      setGravity(0.0, 0.0, -gravity);

      Joint floatingJoint = this.getRootJoints().get(0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(radius/2.0, YoAppearance.EarthTexture());
      floatingJoint.getLink().setLinkGraphics(linkGraphics);

      String gcName = name + "_GC";
      GroundContactPoint gc = new GroundContactPoint(gcName, new Vector3D(0.0, 0.0, 0.0), this);
      floatingJoint.addGroundContactPoint(gc);

      ExternalForcePoint externalForcePoint = new ExternalForcePoint(name + "ForcePoint", new Vector3D(), this);
      floatingJoint.addExternalForcePoint(externalForcePoint);

      YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition(gcName + "_Position", gc.getYoPosition(), 0.01, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("SphereGCPoints", dynamicGraphicPosition);

      YoGraphicVector dynamicGraphicVector = new YoGraphicVector(gcName + "_Force", gc.getYoPosition(), gc.getYoForce(), 1.0, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("SphereForces", dynamicGraphicVector);

      this.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      this.update();

      totalMass = TotalMassCalculator.computeSubTreeMass(body);
   }

   public double getTotalMass()
   {
      return totalMass;
   }

   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   public RigidBodyBasics getBody()
   {
      return body;
   }

   public SixDoFJoint getRootJoint()
   {
      return floatingJoint;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public void updateFrames()
   {
      elevator.updateFramesRecursively();
      centerOfMassFrame.update();
   }

   public void initRobot(Vector3DReadOnly initialPosition, Vector3DReadOnly initialVelocity)
   {
      getYoTime().set(0.0);

      YoDouble q_x = (YoDouble) getVariable("q_x");
      YoDouble q_y = (YoDouble) getVariable("q_y");
      YoDouble q_z = (YoDouble) getVariable("q_z");
      YoDouble qd_x = (YoDouble) getVariable("qd_x");
      YoDouble qd_y = (YoDouble) getVariable("qd_y");
      YoDouble qd_z = (YoDouble) getVariable("qd_z");

      YoDouble q_qs = (YoDouble) getVariable("q_qs");
      YoDouble q_qx = (YoDouble) getVariable("q_qx");
      YoDouble q_qy = (YoDouble) getVariable("q_qy");
      YoDouble q_qz = (YoDouble) getVariable("q_qz");
      YoDouble qd_wx = (YoDouble) getVariable("qd_wx");
      YoDouble qd_wy = (YoDouble) getVariable("qd_wy");
      YoDouble qd_wz = (YoDouble) getVariable("qd_wz");

      q_x.set(initialPosition.getX());
      q_y.set(initialPosition.getY());
      q_z.set(initialPosition.getZ());

      qd_x.set(initialVelocity.getX());
      qd_y.set(initialVelocity.getY());
      qd_z.set(initialVelocity.getZ());

      q_qs.set(0.707106);
      q_qx.set(0.0);
      q_qy.set(0.707106);
      q_qz.set(0.0);

      qd_wx.set(0.0);
      qd_wy.set(0.0);
      qd_wz.set(0.0);

      update();

      updateJointPositions_SCS_to_ID();
      updateJointVelocities_SCS_to_ID();


      updateFrames();
   }
}
