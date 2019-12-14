package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SphereControlToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint3D desiredICP = new YoFramePoint3D("desiredICP", worldFrame, registry);
   private final YoFrameVector3D desiredICPVelocity = new YoFrameVector3D("desiredICPVelocity", worldFrame, registry);

   private final YoFramePoint3D icp = new YoFramePoint3D("icp", worldFrame, registry);

   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final ReferenceFrame centerOfMassFrame;

   private final CenterOfMassJacobian centerOfMassJacobian;

   private final double controlDT;
   private final double desiredHeight;

   private final double totalMass;
   private YoDouble yoTime;

   public SphereControlToolbox(RigidBodyBasics elevator, double controlDT, double desiredHeight, double gravity, YoDouble yoTime, double totalMass,
                               YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.desiredHeight = desiredHeight;
      this.totalMass = totalMass;
      this.yoTime = yoTime;

      double omega = Math.sqrt(gravity / desiredHeight);
      omega0.set(omega);

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);

      String graphicListName = getClass().getSimpleName();

      YoGraphicPosition desiredICPViz = new YoGraphicPosition("Desired Capture Point", desiredICP, 0.01, YoAppearance.Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition icpViz = new YoGraphicPosition("Capture Point", icp, 0.01, YoAppearance.Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("Center of Mass", yoCenterOfMass, 0.01, YoAppearance.Grey(), GraphicType.BALL_WITH_ROTATED_CROSS);

      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, comViz.createArtifact());

      centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);

      parentRegistry.addChild(registry);
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

   public FixedFramePoint3DBasics getDesiredICP()
   {
      return desiredICP;
   }

   public FixedFrameVector3DBasics getDesiredICPVelocity()
   {
      return desiredICPVelocity;
   }

   public FramePoint3DReadOnly getICP()
   {
      return icp;
   }

   public void update()
   {
      centerOfMassFrame.update();
      centerOfMassJacobian.reset();

      yoCenterOfMass.set(centerOfMassJacobian.getCenterOfMass());
      yoCenterOfMassVelocity.set(centerOfMassJacobian.getCenterOfMassVelocity());

      icp.scaleAdd(1.0 / omega0.getDoubleValue(), yoCenterOfMassVelocity, yoCenterOfMass);
   }
}
