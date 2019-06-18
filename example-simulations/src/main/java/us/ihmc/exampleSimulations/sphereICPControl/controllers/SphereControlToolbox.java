package us.ihmc.exampleSimulations.sphereICPControl.controllers;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

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
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class SphereControlToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double footLengthForControl = 0.25;
   private static final double footWidthForControl = 0.12;
   private static final double toeWidthForControl = 0.12;

   private static final double initialTransferDuration = 1.0;
   private static final double singleSupportDuration = 0.7;
   private static final double doubleSupportDuration = 0.2; //0.25;
   private static final double doubleSupportSplitFraction = 0.5;
   private static final boolean useTwoCMPs = true;

   private static final double maxDurationForSmoothingEntryToExitCMPSwitch = 1.0;
   private static final double timeSpentOnExitCMPInPercentOfStepTime = 0.5;

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

   private final YoBoolean sendFootsteps = new YoBoolean("sendFootsteps", registry);
   private final YoBoolean hasFootsteps = new YoBoolean("hasFootsteps", registry);
   private final YoInteger numberOfFootsteps = new YoInteger("numberOfFootsteps", registry);

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final ReferenceFrame centerOfMassFrame;

   private final RigidBodyBasics elevator;
   private final FullRobotModel fullRobotModel;

   private final TwistCalculator twistCalculator;
   private final CenterOfMassJacobian centerOfMassJacobian;

   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> currentFootPoses = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private BipedSupportPolygons bipedSupportPolygons;
   private ICPControlPolygons icpControlPolygons;

   private final ArrayList<Updatable> updatables = new ArrayList<>();
   private final ArrayList<Footstep> footsteps = new ArrayList<>();

   private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2D yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextNextFootstep", "", worldFrame, 4, registry);

   private final double controlDT;
   private final double desiredHeight;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   private ReferenceFrame midFeetZUpFrame;

   private FootstepTestHelper footstepTestHelper;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private SmoothCMPPlannerParameters smoothICPPlannerParameters;
   private ICPOptimizationParameters icpOptimizationParameters;

   private YoDouble yoTime;

   public SphereControlToolbox(FullRobotModel sphereRobotModel, double controlDT, double desiredHeight, double gravity, YoDouble yoTime,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.fullRobotModel = sphereRobotModel;
      this.controlDT = controlDT;
      this.desiredHeight = desiredHeight;
      this.yoTime = yoTime;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      elevator = sphereRobotModel.getElevator();

      double omega = Math.sqrt(gravity / desiredHeight);
      omega0.set(omega);

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);
      setupFeetFrames(gravity, yoGraphicsListRegistry);

      capturePointVelocityAlpha.set(0.5);
      icpVelocity = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector("capturePointVelocity", "", capturePointVelocityAlpha, controlDT, registry, icp);

      String graphicListName = getClass().getSimpleName();

      YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", desiredCMP, 0.012, YoAppearance.Purple(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition desiredICPViz = new YoGraphicPosition("Desired Capture Point", desiredICP, 0.01, YoAppearance.Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition icpViz = new YoGraphicPosition("Capture Point", icp, 0.01, YoAppearance.Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("Center of Mass", yoCenterOfMass, 0.01, YoAppearance.Grey(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

      YoArtifactPolygon nextFootstepArtifact =  new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false);
      YoArtifactPolygon nextNextFootstepArtifact =  new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false);
      YoArtifactPolygon nextNextNextFootstepArtifact =  new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false);

      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCMPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, desiredICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(graphicListName, comViz.createArtifact());

      yoGraphicsListRegistry.registerArtifact(graphicListName, nextFootstepArtifact);
      yoGraphicsListRegistry.registerArtifact(graphicListName, nextNextFootstepArtifact);
      yoGraphicsListRegistry.registerArtifact(graphicListName, nextNextNextFootstepArtifact);

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<>();
      for (FramePoint2D point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));

      YoGraphicShape nextFootstepViz = new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0);
      YoGraphicShape nextNextFootstepViz = new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0);
      YoGraphicShape nextNextNextFootstepViz = new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0);

      yoGraphicsListRegistry.registerYoGraphic(graphicListName, nextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, nextNextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic(graphicListName, nextNextNextFootstepViz);

      updatables.add(new Updatable()
      {
         @Override
         public void update(double time)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               soleFrames.get(robotSide).update();
               soleZUpFrames.get(robotSide).update();
            }
            midFeetZUpFrame.update();
         }
      });

      twistCalculator = new TwistCalculator(worldFrame, sphereRobotModel.getRootJoint().getSuccessor());
      centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);

      smoothICPPlannerParameters = createNewICPPlannerParameters();
      icpOptimizationParameters = createICPOptimizationParameters();

      parentRegistry.addChild(registry);
   }

   private void setupFeetFrames(double gravityZ, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.0;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         currentFootPoses.put(robotSide, new YoFramePoseUsingYawPitchRoll(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         soleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, contactableFoot.getSoleFrame(), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, soleFrames, registry, yoGraphicsListRegistry);

      ICPControlPlane icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);
      omega0.addVariableChangedListener(var -> icpControlPlane.setOmega0(omega0.getValue()));
      omega0.notifyVariableChangedListeners();
      icpControlPolygons = new ICPControlPolygons(icpControlPlane, midFeetZUpFrame, registry, yoGraphicsListRegistry);

      footstepTestHelper = new FootstepTestHelper(contactableFeet);

   }

   public SideDependentList<ReferenceFrame> getSoleZUpFrames()
   {
      return soleZUpFrames;
   }

   public ReferenceFrame getMidFeetZUpFrame()
   {
      return midFeetZUpFrame;
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
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

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public YoFramePoint3D getCenterOfMass()
   {
      return yoCenterOfMass;
   }

   public YoFrameVector3D getCenterOfMassVelocity()
   {
      return yoCenterOfMassVelocity;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public ICPControlPolygons getICPControlPolygons()
   {
      return icpControlPolygons;
   }

   public SideDependentList<FootSpoof> getContactableFeet()
   {
      return contactableFeet;
   }

   public SideDependentList<YoPlaneContactState> getContactStates()
   {
      return contactStates;
   }

   public SideDependentList<FramePose3D> getFootPosesAtTouchdown()
   {
      return footPosesAtTouchdown;
   }

   public SmoothCMPPlannerParameters getNewCapturePointPlannerParameters()
   {
      return smoothICPPlannerParameters;
   }

   public ICPOptimizationParameters getICPOptimizationParameters()
   {
      return icpOptimizationParameters;
   }

   public double getDoubleSupportDuration()
   {
      return doubleSupportDuration;
   }

   public double getSingleSupportDuration()
   {
      return singleSupportDuration;
   }

   public YoDouble getYoTime()
   {
      return yoTime;
   }

   public YoFramePoint3D getDesiredCMP()
   {
      return desiredCMP;
   }

   public YoFramePoint3D getDesiredICP()
   {
      return desiredICP;
   }

   public YoFrameVector3D getDesiredICPVelocity()
   {
      return desiredICPVelocity;
   }

   public YoFramePoint3D getICP()
   {
      return icp;
   }

   public YoFrameVector3D getICPVelocity()
   {
      return icpVelocity;
   }

   public FramePoint2D getCapturePoint2d()
   {
      return capturePoint2d;
   }

   public void update()
   {
      centerOfMassFrame.update();

      twistCalculator.compute();
      centerOfMassJacobian.reset();
      bipedSupportPolygons.updateUsingContactStates(contactStates);
      icpControlPolygons.updateUsingContactStates(contactStates);

      callUpdatables();
      updateFootViz();
      computeCenterOfMass();
      computeCapturePoint();

      if (sendFootsteps.getBooleanValue())
      {
         for (Footstep footstep : footstepTestHelper.createFootsteps(0.4, 0.6, 10))
            footsteps.add(footstep);

         sendFootsteps.set(false);
      }

      hasFootsteps.set(footsteps.size() > 0);
      numberOfFootsteps.set(footsteps.size());
   }

   private void updateFootViz()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (contactStates.get(robotSide).inContact())
         {
            FramePose3D footPose = new FramePose3D(contactableFeet.get(robotSide).getSoleFrame());
            footPose.changeFrame(worldFrame);
            currentFootPoses.get(robotSide).set(footPose);
         }
         else
         {
            currentFootPoses.get(robotSide).setToNaN();
         }
      }
   }

   public boolean hasFootsteps()
   {
      return hasFootsteps.getBooleanValue();
   }

   public int getNumberOfSteps()
   {
      return numberOfFootsteps.getIntegerValue();
   }

   public Footstep getFootstep(int i)
   {
      return footsteps.remove(i);
   }

   public Footstep peekAtFootstep(int i)
   {
      if (i >= footsteps.size())
         return null;
      else
         return footsteps.get(i);
   }

   private final FrameConvexPolygon2D footstepPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D tempFootstepPolygonForShrinking = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   public void updateUpcomingFootstepsViz(Footstep nextFootstep, Footstep nextNextFootstep, Footstep nextNextNextFootstep)
   {
      if (nextFootstep == null)
      {
         yoNextFootstepPose.setToNaN();
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextFootstepPolygon.clear();
         yoNextNextFootstepPolygon.clear();
         yoNextNextNextFootstepPolygon.clear();
         return;
      }

      if (nextFootstep.getPredictedContactPoints() == null)
         nextFootstep.setPredictedContactPoints(contactableFeet.get(nextFootstep.getRobotSide()).getContactPoints2d());

      double polygonShrinkAmount = 0.005;

      tempFootstepPolygonForShrinking.setIncludingFrame(nextFootstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(nextFootstep.getPredictedContactPoints()));
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextFootstepPolygon.set(footstepPolygon);

      FramePose3D nextFootstepPose = new FramePose3D(nextFootstep.getSoleReferenceFrame());
      yoNextFootstepPose.setMatchingFrame(nextFootstepPose);

      if (nextNextFootstep == null)
      {
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextFootstepPolygon.clear();
         yoNextNextNextFootstepPolygon.clear();
         return;
      }

      if (nextNextFootstep.getPredictedContactPoints() == null)
         nextNextFootstep.setPredictedContactPoints(contactableFeet.get(nextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrame(nextNextFootstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(nextNextFootstep.getPredictedContactPoints()));
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextFootstepPolygon.set(footstepPolygon);

      FramePose3D nextNextFootstepPose = new FramePose3D(nextNextFootstep.getSoleReferenceFrame());
      yoNextNextFootstepPose.setMatchingFrame(nextNextFootstepPose);

      if (nextNextNextFootstep == null)
      {
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPolygon.clear();
         return;
      }

      if (nextNextNextFootstep.getPredictedContactPoints() == null)
         nextNextNextFootstep.setPredictedContactPoints(contactableFeet.get(nextNextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrame(nextNextNextFootstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(nextNextNextFootstep.getPredictedContactPoints()));
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextNextFootstepPolygon.set(footstepPolygon);

      FramePose3D nextNextNextFootstepPose = new FramePose3D(nextNextNextFootstep.getSoleReferenceFrame());
      yoNextNextNextFootstepPose.setMatchingFrame(nextNextNextFootstepPose);
   }

   private void callUpdatables()
   {
      for (Updatable updatable : updatables)
         updatable.update(yoTime.getDoubleValue());
   }


   private final FramePoint3D centerOfMass = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FramePoint2D centerOfMass2d = new FramePoint2D();
   private final FrameVector2D centerOfMassVelocity2d = new FrameVector2D();
   public void computeCenterOfMass()
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

   private SmoothCMPPlannerParameters createNewICPPlannerParameters()
   {
      return new SphereSmoothCMPPlannerParameters();
   }

   public static class SphereSmoothCMPPlannerParameters extends SmoothCMPPlannerParameters
   {
      public SphereSmoothCMPPlannerParameters()
      {
         super();
         endCoPName = CoPPointName.MIDFEET_COP;
         entryCoPName = CoPPointName.ENTRY_COP;
         exitCoPName = CoPPointName.EXIT_COP;
         swingCopPointsToPlan = new CoPPointName[]{CoPPointName.MIDFOOT_COP, CoPPointName.EXIT_COP};
         transferCoPPointsToPlan = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.ENTRY_COP};

         stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFEET_COP, 0.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.ENTRY_COP, 1.0 / 3.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFOOT_COP, 1.0 / 8.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.EXIT_COP, 1.0 / 3.0);

         copOffsetsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(0.0, 0.0));
         copOffsetsInFootFrame.put(CoPPointName.ENTRY_COP, new Vector2D(0.0, -0.005));
         copOffsetsInFootFrame.put(CoPPointName.MIDFOOT_COP, new Vector2D(0.0, 0.01));
         copOffsetsInFootFrame.put(CoPPointName.EXIT_COP, new Vector2D(0.0, 0.025));

         copOffsetBoundsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
         copOffsetBoundsInFootFrame.put(CoPPointName.ENTRY_COP, new Vector2D(-0.04, 0.03));
         copOffsetBoundsInFootFrame.put(CoPPointName.MIDFOOT_COP, new Vector2D(0.0, 0.055));
         copOffsetBoundsInFootFrame.put(CoPPointName.EXIT_COP, new Vector2D(0.0, 0.08));
      }

      @Override
      public boolean planSwingAngularMomentum()
      {
         return true;
      }

      @Override
      public boolean planTransferAngularMomentum()
      {
         return true;
      }
   }

   public ICPOptimizationParameters createICPOptimizationParameters()
   {
      return new ICPOptimizationParameters()
      {
         @Override
         public double getForwardFootstepWeight()
         {
            return 20.0;
         }

         @Override
         public double getLateralFootstepWeight()
         {
            return 20.0;
         }

         @Override
         public double getFootstepRateWeight()
         {
            return 0.001;
         }

         @Override
         public double getFeedbackForwardWeight()
         {
            return 0.5;
         }

         @Override
         public double getFeedbackLateralWeight()
         {
            return 0.5;
         }

         @Override
         public double getFeedbackRateWeight()
         {
            return 0.0001;
         }

         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpOrthogonalToMotion(2.5);
            gains.setKpParallelToMotion(3.0);
            return gains;
         }

         @Override
         public double getDynamicsObjectiveWeight()
         {
            return 500.0;
         }

         @Override
         public double getDynamicsObjectiveDoubleSupportWeightModifier()
         {
            return 1.0;
         }

         @Override
         public double getAngularMomentumMinimizationWeight()
         {
            return 50.0;
         }

         @Override
         public boolean scaleStepRateWeightWithTime()
         {
            return false;
         }

         @Override
         public boolean scaleFeedbackWeightWithGain()
         {
            return true;
         }

         @Override
         public boolean useFeedbackRate()
         {
            return true;
         }

         @Override
         public boolean allowStepAdjustment()
         {
            return true;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return true;
         }

        @Override
         public boolean useFootstepRate()
         {
            return true;
         }

         @Override
         public double getMinimumFootstepWeight()
         {
            return 0.0001;
         }

         @Override
         public double getMinimumFeedbackWeight()
         {
            return 0.0001;
         }

         @Override
         public double getMinimumTimeRemaining()
         {
            return 0.0001;
         }

         @Override
         public double getAdjustmentDeadband()
         {
            return 0.03;
         }
      };
   }
}
