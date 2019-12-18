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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
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
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

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

   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, getRobotsYoVariableRegistry());
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, getRobotsYoVariableRegistry());

   private final ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);
   private final YoFramePoint3D desiredDCM = new YoFramePoint3D("desiredDCM", worldFrame, getRobotsYoVariableRegistry());
   private final YoFrameVector3D desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, getRobotsYoVariableRegistry());

   private final YoDouble omega0 = new YoDouble("omega0", getRobotsYoVariableRegistry());

   private final YoFramePoint3D dcm = new YoFramePoint3D("dcm", worldFrame, getRobotsYoVariableRegistry());

   private final CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);

   private final double totalMass;
   private final double controlDT;
   private final double desiredHeight;

   public SphereRobot(String name, double gravity, double controlDt, double desiredHeight, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(name, floatingJoint);

      this.controlDT = controlDt;
      this.desiredHeight = desiredHeight;

      double omega = Math.sqrt(gravity / desiredHeight);
      omega0.set(omega);

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

      String graphicListName = getClass().getSimpleName();

      YoGraphicPosition desiredICPViz = new YoGraphicPosition("Desired DCM", desiredDCM, 0.01, YoAppearance.Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition icpViz = new YoGraphicPosition("DCM", dcm, 0.01, YoAppearance.Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("Center of Mass", yoCenterOfMass, 0.01, YoAppearance.Grey(), GraphicType.BALL_WITH_ROTATED_CROSS);

      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, comViz.createArtifact());

      this.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      this.update();

      totalMass = TotalMassCalculator.computeSubTreeMass(body);
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public double getDesiredHeight()
   {
      return desiredHeight;
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
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
