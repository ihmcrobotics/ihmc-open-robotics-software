package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SphereControlToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint3D desiredDCM = new YoFramePoint3D("desiredDCM", worldFrame, registry);
   private final YoFrameVector3D desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);

   private final YoFramePoint3D dcm = new YoFramePoint3D("dcm", worldFrame, registry);

   private final YoFramePoint3D body = new YoFramePoint3D("body", worldFrame, registry);
   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final ReferenceFrame centerOfMassFrame;

   private final CenterOfMassJacobian centerOfMassJacobian;
   private final RigidBodyReadOnly elevator;
   private final double controlDT;
   private final double desiredHeight;

   private final double totalMass;
   private YoDouble yoTime;

   private final SphereRobot sphereRobot;
   private final double gravityZ;

   public SphereControlToolbox(SphereRobot sphereRobot, double controlDT, double desiredHeight, double gravity, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.sphereRobot = sphereRobot;
      this.controlDT = controlDT;
      this.desiredHeight = desiredHeight;
      this.totalMass = sphereRobot.getTotalMass();
      this.yoTime = sphereRobot.getYoTime();
      this.elevator = sphereRobot.getElevator();
      this.gravityZ = gravity;

      double omega = Math.sqrt(gravity / desiredHeight);
      omega0.set(omega);

      this.centerOfMassFrame = sphereRobot.getCenterOfMassFrame();

      String graphicListName = getClass().getSimpleName();

      YoGraphicPosition desiredICPViz = new YoGraphicPosition("Desired DCM", desiredDCM, 0.01, YoAppearance.Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition icpViz = new YoGraphicPosition("DCM", dcm, 0.01, YoAppearance.Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("Center of Mass", yoCenterOfMass, 0.01, YoAppearance.Grey(), GraphicType.BALL_WITH_ROTATED_CROSS);

      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, comViz.createArtifact());

      centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);

      sphereRobot.getRobotsYoVariableRegistry().addChild(registry);
   }

   public double getGravityZ()
   {
      return gravityZ;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public double getDesiredHeight()
   {
      return desiredHeight;
   }

   public double getTotalMass()
   {
      return totalMass;
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public FramePoint3DReadOnly getCenterOfMass()
   {
      return yoCenterOfMass;
   }

   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return yoCenterOfMassVelocity;
   }

   public YoDouble getYoTime()
   {
      return yoTime;
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

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();

   public void update()
   {
      sphereRobot.updateFrames();

      centerOfMassJacobian.reset();

      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassVelocity.setIncludingFrame(centerOfMassJacobian.getCenterOfMassVelocity());

      yoCenterOfMass.setMatchingFrame(centerOfMassJacobian.getCenterOfMass());
      yoCenterOfMassVelocity.setMatchingFrame(centerOfMassVelocity);

      dcm.scaleAdd(1.0 / omega0.getDoubleValue(), yoCenterOfMassVelocity, yoCenterOfMass);

      centerOfMassPosition.setToZero(elevator.getChildrenJoints().get(0).getFrameAfterJoint());
      body.setMatchingFrame(centerOfMassPosition);
   }
}
