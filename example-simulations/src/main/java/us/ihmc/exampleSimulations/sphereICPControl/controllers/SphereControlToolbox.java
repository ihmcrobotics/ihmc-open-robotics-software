package us.ihmc.exampleSimulations.sphereICPControl.controllers;

import java.awt.Color;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSupportPolygonNames;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
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
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
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

   private final YoFramePoint eCMP = new YoFramePoint("eCMP", worldFrame, registry);
   private final YoFramePoint desiredICP = new YoFramePoint("desiredICP", worldFrame, registry);
   private final YoFrameVector desiredICPVelocity = new YoFrameVector("desiredICPVelocity", worldFrame, registry);
   private final YoFramePoint desiredCMP = new YoFramePoint("desiredCMP", worldFrame, registry);

   private final YoFramePoint icp = new YoFramePoint("icp", worldFrame, registry);

   private final YoFramePoint yoCenterOfMass = new YoFramePoint("centerOfMass", worldFrame, registry);
   private final YoFrameVector yoCenterOfMassVelocity = new YoFrameVector("centerOfMassVelocity", worldFrame, registry);
   private final YoFramePoint2d yoCenterOfMass2d = new YoFramePoint2d("centerOfMass2d", worldFrame, registry);
   private final YoFrameVector2d yoCenterOfMassVelocity2d = new YoFrameVector2d("centerOfMassVelocity2d", worldFrame, registry);

   private final YoBoolean sendFootsteps = new YoBoolean("sendFootsteps", registry);
   private final YoBoolean hasFootsteps = new YoBoolean("hasFootsteps", registry);
   private final YoInteger numberOfFootsteps = new YoInteger("numberOfFootsteps", registry);

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final ReferenceFrame centerOfMassFrame;

   private final RigidBody elevator;
   private final FullRobotModel fullRobotModel;

   private final TwistCalculator twistCalculator;
   private final CenterOfMassJacobian centerOfMassJacobian;

   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<YoFramePose> currentFootPoses = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private BipedSupportPolygons bipedSupportPolygons;
   private ICPControlPolygons icpControlPolygons;

   private final ArrayList<Updatable> updatables = new ArrayList<>();
   private final ArrayList<Footstep> footsteps = new ArrayList<>();

   private final YoFramePose yoNextFootstepPose = new YoFramePose("nextFootstepPose", worldFrame, registry);
   private final YoFramePose yoNextNextFootstepPose = new YoFramePose("nextNextFootstepPose", worldFrame, registry);
   private final YoFramePose yoNextNextNextFootstepPose = new YoFramePose("nextNextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoNextFootstepPolygon = new YoFrameConvexPolygon2d("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2d yoNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2d yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextNextFootstep", "", worldFrame, 4, registry);

   private final double controlDT;
   private final double desiredHeight;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   private ReferenceFrame midFeetZUpFrame;

   private FootstepTestHelper footstepTestHelper;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private ContinuousCMPICPPlannerParameters capturePointPlannerParameters;
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

      setupFeetFrames(gravity, yoGraphicsListRegistry);

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
               ankleZUpFrames.get(robotSide).update();
            }
            midFeetZUpFrame.update();
         }
      });

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, elevator);

      twistCalculator = new TwistCalculator(worldFrame, sphereRobotModel.getRootJoint().getSuccessor());
      centerOfMassJacobian = new CenterOfMassJacobian(elevator);

      capturePointPlannerParameters = createICPPlannerParameters();
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

         currentFootPoses.put(robotSide, new YoFramePose(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBody foot = contactableFoot.getRigidBody();
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
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, yoGraphicsListRegistry);

      ICPControlPlane icpControlPlane = new ICPControlPlane(omega0, centerOfMassFrame, gravityZ, registry);
      icpControlPolygons = new ICPControlPolygons(icpControlPlane, midFeetZUpFrame, registry, yoGraphicsListRegistry);

      footstepTestHelper = new FootstepTestHelper(contactableFeet);

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

   public YoFramePoint getCenterOfMass()
   {
      return yoCenterOfMass;
   }

   public YoFrameVector getCenterOfMassVelocity()
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

   public ContinuousCMPICPPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
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

   public YoFramePoint getDesiredCMP()
   {
      return desiredCMP;
   }

   public YoFramePoint getDesiredICP()
   {
      return desiredICP;
   }

   public YoFrameVector getDesiredICPVelocity()
   {
      return desiredICPVelocity;
   }

   public YoFramePoint getICP()
   {
      return icp;
   }

   public FramePoint2D getCapturePoint2d()
   {
      return capturePoint2d;
   }

   public void update()
   {
      centerOfMassFrame.update();

      twistCalculator.compute();
      centerOfMassJacobian.compute();
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

   private final FrameConvexPolygon2d footstepPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempFootstepPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   public void updateUpcomingFootstepsViz(Footstep nextFootstep, Footstep nextNextFootstep, Footstep nextNextNextFootstep)
   {
      if (nextFootstep == null)
      {
         yoNextFootstepPose.setToNaN();
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextFootstepPolygon.hide();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextFootstep.getPredictedContactPoints() == null)
         nextFootstep.setPredictedContactPoints(contactableFeet.get(nextFootstep.getRobotSide()).getContactPoints2d());

      double polygonShrinkAmount = 0.005;

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextFootstep.getSoleReferenceFrame(), nextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose3D nextFootstepPose = new FramePose3D(nextFootstep.getSoleReferenceFrame());
      yoNextFootstepPose.setAndMatchFrame(nextFootstepPose);

      if (nextNextFootstep == null)
      {
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextNextFootstep.getPredictedContactPoints() == null)
         nextNextFootstep.setPredictedContactPoints(contactableFeet.get(nextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextNextFootstep.getSoleReferenceFrame(), nextNextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose3D nextNextFootstepPose = new FramePose3D(nextNextFootstep.getSoleReferenceFrame());
      yoNextNextFootstepPose.setAndMatchFrame(nextNextFootstepPose);

      if (nextNextNextFootstep == null)
      {
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (nextNextNextFootstep.getPredictedContactPoints() == null)
         nextNextNextFootstep.setPredictedContactPoints(contactableFeet.get(nextNextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(nextNextNextFootstep.getSoleReferenceFrame(), nextNextNextFootstep.getPredictedContactPoints());
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose3D nextNextNextFootstepPose = new FramePose3D(nextNextNextFootstep.getSoleReferenceFrame());
      yoNextNextNextFootstepPose.setAndMatchFrame(nextNextNextFootstepPose);
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
      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);

      CapturePointCalculator.computeCapturePoint(capturePoint2d, centerOfMass2d, centerOfMassVelocity2d, omega0.getDoubleValue());

      capturePoint2d.changeFrame(icp.getReferenceFrame());
      icp.set(capturePoint2d, 0.0);
   }

   private ContinuousCMPICPPlannerParameters createICPPlannerParameters()
   {
      return new ContinuousCMPICPPlannerParameters()
      {
         @Override
         public int getNumberOfCoPWayPointsPerFoot()
         {
            return 2;
         }

         @Override
         public EnumMap<CoPPointName, Vector2D> getCoPForwardOffsetBoundsInFoot()
         {
            EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds;

            Vector2D entryBounds = new Vector2D(0.0, 0.03);
            Vector2D exitBounds = new Vector2D(-0.04, 0.08);

            copForwardOffsetBounds = new EnumMap<>(CoPPointName.class);
            copForwardOffsetBounds.put(entryCoPName, entryBounds);
            copForwardOffsetBounds.put(exitCoPName, exitBounds);

            return copForwardOffsetBounds;
         }

         /**{@inheritDoc} */
         @Override
         public CoPPointName getExitCoPName()
         {
            return exitCoPName;
         }

         /**{@inheritDoc} */
         @Override
         public CoPPointName getEntryCoPName()
         {
            return entryCoPName;
         }

         @Override
         public EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame()
         {
            EnumMap<CoPPointName, Vector2D> copOffsets;

            Vector2D entryOffset = new Vector2D(0.0, -0.005);
            Vector2D exitOffset = new Vector2D(0.0, 0.015); //FIXME 0.025);

            copOffsets = new EnumMap<CoPPointName, Vector2D>(CoPPointName.class);
            copOffsets.put(entryCoPName, entryOffset);
            copOffsets.put(entryCoPName, exitOffset);

            return copOffsets;
         }
      };
   }

   private SmoothCMPPlannerParameters createNewICPPlannerParameters()
   {
      return new SphereSmoothCMPPlannerParameters();
   }

   private class SphereSmoothCMPPlannerParameters extends SmoothCMPPlannerParameters
   {
      public SphereSmoothCMPPlannerParameters()
      {
         super();
         endCoPName = CoPPointName.MIDFEET_COP;
         entryCoPName = CoPPointName.HEEL_COP;
         exitCoPName = CoPPointName.TOE_COP;
         swingCopPointsToPlan = new CoPPointName[]{CoPPointName.BALL_COP, CoPPointName.TOE_COP};
         transferCoPPointsToPlan = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.HEEL_COP};
         copOffsetFrameNames.put(CoPPointName.HEEL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
         copOffsetFrameNames.put(CoPPointName.BALL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
         copOffsetFrameNames.put(CoPPointName.TOE_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
         copOffsetFrameNames.put(CoPPointName.MIDFEET_COP, CoPSupportPolygonNames.INITIAL_DOUBLE_SUPPORT_POLYGON);

         stepLengthOffsetPolygon.put(CoPPointName.MIDFEET_COP, CoPSupportPolygonNames.NULL);
         stepLengthOffsetPolygon.put(CoPPointName.HEEL_COP, CoPSupportPolygonNames.INITIAL_SWING_POLYGON);
         stepLengthOffsetPolygon.put(CoPPointName.BALL_COP, CoPSupportPolygonNames.FINAL_SWING_POLYGON);
         stepLengthOffsetPolygon.put(CoPPointName.TOE_COP, CoPSupportPolygonNames.FINAL_SWING_POLYGON);

         constrainToMinMax.put(CoPPointName.MIDFEET_COP, false);
         constrainToMinMax.put(CoPPointName.HEEL_COP, true);
         constrainToMinMax.put(CoPPointName.BALL_COP, true);
         constrainToMinMax.put(CoPPointName.TOE_COP, true);

         constrainToSupportPolygon.put(CoPPointName.MIDFEET_COP, false);
         constrainToSupportPolygon.put(CoPPointName.HEEL_COP, true);
         constrainToSupportPolygon.put(CoPPointName.BALL_COP, true);
         constrainToSupportPolygon.put(CoPPointName.TOE_COP, true);

         stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFEET_COP, 0.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.HEEL_COP, 1.0 / 3.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.BALL_COP, 1.0 / 8.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.TOE_COP, 1.0 / 3.0);

         copOffsetsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(0.0, 0.0));
         copOffsetsInFootFrame.put(CoPPointName.HEEL_COP, new Vector2D(0.0, -0.005));
         copOffsetsInFootFrame.put(CoPPointName.BALL_COP, new Vector2D(0.0, 0.01));
         copOffsetsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.025));

         copOffsetBoundsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
         copOffsetBoundsInFootFrame.put(CoPPointName.HEEL_COP, new Vector2D(-0.04, 0.03));
         copOffsetBoundsInFootFrame.put(CoPPointName.BALL_COP, new Vector2D(0.0, 0.055));
         copOffsetBoundsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.08));
      }

      @Override
      public boolean planWithAngularMomentum()
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
         public double getFeedbackParallelGain()
         {
            return 3.0;
         }

         @Override
         public double getFeedbackOrthogonalGain()
         {
            return 2.5;
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
