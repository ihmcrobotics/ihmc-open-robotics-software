package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SphereRobot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final double mass = 1.0;
   private static final double Ixx1 = 0.1, Iyy1 = 0.1, Izz1 = 0.1;
   private static final Matrix3D inertia = new Matrix3D(Ixx1, 0.0, 0.0, 0.0, Iyy1, 0.0, 0.0, 0.0, Izz1);

   private final RigidBodyBasics elevator;
   private final SixDoFJoint floatingJoint;
   private final RigidBodyBasics body;
   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private static final double radius = 0.1;

   private final YoFramePoint3D yoCenterOfMass;
   private final YoFrameVector3D yoCenterOfMassVelocity;

   private final YoFramePoint3D desiredDCM;
   private final YoFrameVector3D desiredDCMVelocity;

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final YoFramePoint3D dcm;


   private final SCSRobotFromInverseDynamicsRobotModel scsRobot;

   private final double totalMass;
   private final double controlDT;
   private final double desiredHeight;
   private final double gravityZ;

   public SphereRobot(String name, double gravity, double controlDt, double desiredHeight, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDt;
      this.desiredHeight = desiredHeight;
      this.gravityZ = Math.abs(gravity);

      elevator = new RigidBody(name + "_elevator", worldFrame);
      floatingJoint = new SixDoFJoint(name + "_floatingJoint", elevator);
      body = new RigidBody(name + "_body", floatingJoint, inertia, mass, new Vector3D());

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);
      centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);

      yoCenterOfMass = new YoFramePoint3D(name + "_CenterOfMass", worldFrame, registry);
      yoCenterOfMassVelocity = new YoFrameVector3D(name + "_CenterOfMassVelocity", worldFrame, registry);
      desiredDCM = new YoFramePoint3D(name + "_DesiredDCM", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D(name + "_DesiredDCMVelocity", worldFrame, registry);
      dcm = new YoFramePoint3D(name + "_DCM", worldFrame, registry);

      double omega = Math.sqrt(gravity / desiredHeight);
      omega0.set(omega);

      scsRobot = new SCSRobotFromInverseDynamicsRobotModel(name, floatingJoint);
      scsRobot.getRobotsYoRegistry().addChild(registry);

      scsRobot.setGravity(0.0, 0.0, -gravity);

      Joint floatingJoint = scsRobot.getRootJoints().get(0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(radius/2.0, YoAppearance.EarthTexture());
      floatingJoint.getLink().setLinkGraphics(linkGraphics);

      String gcName = name + "_GC";
      GroundContactPoint gc = new GroundContactPoint(gcName, new Vector3D(0.0, 0.0, 0.0), scsRobot);
      floatingJoint.addGroundContactPoint(gc);

      ExternalForcePoint externalForcePoint = new ExternalForcePoint(name + "ForcePoint", new Vector3D(), scsRobot);
      floatingJoint.addExternalForcePoint(externalForcePoint);

      YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition(gcName + "_Position", gc.getYoPosition(), 0.01, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("SphereGCPoints", dynamicGraphicPosition);

      YoGraphicVector dynamicGraphicVector = new YoGraphicVector(gcName + "_Force", gc.getYoPosition(), gc.getYoForce(), 1.0, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("SphereForces", dynamicGraphicVector);

      String graphicListName = getClass().getSimpleName();

      YoGraphicPosition desiredICPViz = new YoGraphicPosition(name + "Desired DCM", desiredDCM, 0.01, YoAppearance.Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition icpViz = new YoGraphicPosition(name + "DCM", dcm, 0.01, YoAppearance.Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition(name + "Center of Mass", yoCenterOfMass, 0.01, YoAppearance.Grey(), GraphicType.BALL_WITH_ROTATED_CROSS);

      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, comViz.createArtifact());

      scsRobot.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scsRobot.update();

      totalMass = TotalMassCalculator.computeSubTreeMass(body);
   }

   public SCSRobotFromInverseDynamicsRobotModel getScsRobot()
   {
      return scsRobot;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public double getDesiredHeight()
   {
      return desiredHeight;
   }

   public DoubleProvider getOmega0Provider()
   {
      return omega0;
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   public double getTotalMass()
   {
      return totalMass;
   }

   public double getGravityZ()
   {
      return gravityZ;
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

   public FixedFramePoint3DBasics getDesiredDCM()
   {
      return desiredDCM;
   }

   public FixedFrameVector3DBasics getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   public FramePoint3DReadOnly getDCM()
   {
      return dcm;
   }

   public FramePoint3DReadOnly getCenterOfMass()
   {
      return yoCenterOfMass;
   }

   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return yoCenterOfMassVelocity;
   }

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();

   public void updateFrames()
   {
      scsRobot.update();
      elevator.updateFramesRecursively();
      centerOfMassFrame.update();

      centerOfMassJacobian.reset();

      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassVelocity.setIncludingFrame(centerOfMassJacobian.getCenterOfMassVelocity());

      yoCenterOfMass.setMatchingFrame(centerOfMassJacobian.getCenterOfMass());
      yoCenterOfMassVelocity.setMatchingFrame(centerOfMassVelocity);

      dcm.scaleAdd(1.0 / omega0.getDoubleValue(), yoCenterOfMassVelocity, yoCenterOfMass);

      centerOfMassPosition.setToZero(elevator.getChildrenJoints().get(0).getFrameAfterJoint());
   }


   public void initRobot(Vector3DReadOnly initialPosition, Vector3DReadOnly initialVelocity)
   {
      scsRobot.getYoTime().set(0.0);

      YoDouble q_x = (YoDouble) scsRobot.findVariable("q_x");
      YoDouble q_y = (YoDouble) scsRobot.findVariable("q_y");
      YoDouble q_z = (YoDouble) scsRobot.findVariable("q_z");
      YoDouble qd_x = (YoDouble) scsRobot.findVariable("qd_x");
      YoDouble qd_y = (YoDouble) scsRobot.findVariable("qd_y");
      YoDouble qd_z = (YoDouble) scsRobot.findVariable("qd_z");

      YoDouble q_qs = (YoDouble) scsRobot.findVariable("q_qs");
      YoDouble q_qx = (YoDouble) scsRobot.findVariable("q_qx");
      YoDouble q_qy = (YoDouble) scsRobot.findVariable("q_qy");
      YoDouble q_qz = (YoDouble) scsRobot.findVariable("q_qz");
      YoDouble qd_wx = (YoDouble) scsRobot.findVariable("qd_wx");
      YoDouble qd_wy = (YoDouble) scsRobot.findVariable("qd_wy");
      YoDouble qd_wz = (YoDouble) scsRobot.findVariable("qd_wz");

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

      scsRobot.update();

      updateJointPositions_SCS_to_ID();
      updateJointVelocities_SCS_to_ID();

      updateFrames();
   }

   public void updateJointPositions_SCS_to_ID()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
   }

   public void updateJointVelocities_SCS_to_ID()
   {
      scsRobot.updateJointVelocities_SCS_to_ID();
   }

   public void updateJointPositions_ID_to_SCS()
   {
      scsRobot.updateJointPositions_ID_to_SCS();
   }

   public void updateJointVelocities_ID_to_SCS()
   {
      scsRobot.updateJointVelocities_ID_to_SCS();
   }

   public void updateJointTorques_ID_to_SCS()
   {
      scsRobot.updateJointTorques_ID_to_SCS();
   }
}
