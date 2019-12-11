package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class SphereControlToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint3D eCMP = new YoFramePoint3D("eCMP", worldFrame, registry);
   private final YoFramePoint3D desiredICP = new YoFramePoint3D("desiredICP", worldFrame, registry);
   private final YoFrameVector3D desiredICPVelocity = new YoFrameVector3D("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint3D desiredCMP = new YoFramePoint3D("desiredCMP", worldFrame, registry);

   private final YoFramePoint3D icp = new YoFramePoint3D("icp", worldFrame, registry);
   private final FilteredVelocityYoFrameVector icpVelocity;
   private final YoDouble capturePointVelocityAlpha = new YoDouble("capturePointVelocityAlpha", registry);

   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);
   private final YoFramePoint2D yoCenterOfMass2d = new YoFramePoint2D("centerOfMass2d", worldFrame, registry);
   private final YoFrameVector2D yoCenterOfMassVelocity2d = new YoFrameVector2D("centerOfMassVelocity2d", worldFrame, registry);

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final ReferenceFrame centerOfMassFrame;


   private final CenterOfMassJacobian centerOfMassJacobian;

   private final double controlDT;
   private final double desiredHeight;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private YoDouble yoTime;

   public SphereControlToolbox(RigidBodyBasics elevator, double controlDT, double desiredHeight, double gravity, YoDouble yoTime,
                               YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controlDT = controlDT;
      this.desiredHeight = desiredHeight;
      this.yoTime = yoTime;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      double omega = Math.sqrt(gravity / desiredHeight);
      omega0.set(omega);

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);

      capturePointVelocityAlpha.set(0.5);
      icpVelocity = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector("capturePointVelocity", "", capturePointVelocityAlpha, controlDT, registry, icp);

      String graphicListName = getClass().getSimpleName();

      YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", desiredCMP, 0.012, YoAppearance.Purple(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition desiredICPViz = new YoGraphicPosition("Desired Capture Point", desiredICP, 0.01, YoAppearance.Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition icpViz = new YoGraphicPosition("Capture Point", icp, 0.01, YoAppearance.Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("Center of Mass", yoCenterOfMass, 0.01, YoAppearance.Grey(), GraphicType.BALL_WITH_ROTATED_CROSS);

      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCMPViz.createArtifact());
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

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
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

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public YoDouble getYoTime()
   {
      return yoTime;
   }

   public FixedFramePoint3DBasics getDesiredCMP()
   {
      return desiredCMP;
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

   public FrameVector3DReadOnly getICPVelocity()
   {
      return icpVelocity;
   }

   public FramePoint2DReadOnly getCapturePoint2d()
   {
      return capturePoint2d;
   }

   public void update()
   {
      centerOfMassFrame.update();

      centerOfMassJacobian.reset();

      computeCenterOfMass();
      computeCapturePoint();
   }

   private final FramePoint3D centerOfMass = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FramePoint2D centerOfMass2d = new FramePoint2D();
   private final FrameVector2D centerOfMassVelocity2d = new FrameVector2D();

   private void computeCenterOfMass()
   {
      centerOfMass.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMass2d.setIncludingFrame(centerOfMass);
      centerOfMassVelocity2d.setIncludingFrame(centerOfMassVelocity);

      yoCenterOfMass.set(centerOfMass);
      yoCenterOfMassVelocity.set(centerOfMassVelocity);

      yoCenterOfMass2d.set(centerOfMass2d);
      yoCenterOfMassVelocity2d.set(centerOfMassVelocity2d);
   }

   private final FramePoint2D capturePoint2d = new FramePoint2D();

   public void computeCapturePoint()
   {
      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMassVelocity.setIncludingFrame(centerOfMassJacobian.getCenterOfMassVelocity());

      CapturePointCalculator.computeCapturePoint(capturePoint2d, centerOfMass2d, centerOfMassVelocity2d, omega0.getDoubleValue());

      capturePoint2d.changeFrame(icp.getReferenceFrame());
      icp.set(capturePoint2d, 0.0);
   }

}
