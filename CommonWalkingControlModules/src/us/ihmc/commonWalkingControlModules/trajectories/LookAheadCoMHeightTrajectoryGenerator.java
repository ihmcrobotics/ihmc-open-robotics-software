package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.StringStretcher2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.WaypointPositionTrajectoryData;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

/**
 * TODO There is not enough HumanoidReferenceFrames in that class, it is pretty fragile
 *
 */
public class LookAheadCoMHeightTrajectoryGenerator implements CoMHeightTrajectoryGenerator
{
   private static final boolean CONSIDER_NEXT_FOOTSTEP = false;

   private static final boolean DEBUG = false;

   private boolean VISUALIZE = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FourPointSpline1D spline = new FourPointSpline1D(registry);

   private final DesiredComHeightProvider desiredComHeightProvider;

   private final BooleanYoVariable hasBeenInitializedWithNextStep = new BooleanYoVariable("hasBeenInitializedWithNextStep", registry);

   private final DoubleYoVariable offsetHeightAboveGround = new DoubleYoVariable("offsetHeightAboveGround", registry);

   private final DoubleYoVariable offsetHeightAboveGroundPrevValue = new DoubleYoVariable("offsetHeightAboveGroundPrevValue", registry);
   private final DoubleYoVariable offsetHeightAboveGroundChangedTime = new DoubleYoVariable("offsetHeightAboveGroundChangedTime", registry);
   private final YoVariableDoubleProvider offsetHeightAboveGroundInitialPositionProvider = new YoVariableDoubleProvider(offsetHeightAboveGroundPrevValue);
   private final YoVariableDoubleProvider offsetHeightAboveGroundFinalPositionProvider = new YoVariableDoubleProvider(offsetHeightAboveGround);
   private final YoVariableDoubleProvider offsetHeightAboveGroundTrajectoryOutput = new YoVariableDoubleProvider("offsetHeightAboveGroundTrajectoryOutput",
                                                                                       registry);
   private final YoVariableDoubleProvider offsetHeightAboveGroundTrajectoryTimeProvider =
      new YoVariableDoubleProvider("offsetHeightAboveGroundTrajectoryTimeProvider", registry);
   private final CubicPolynomialTrajectoryGenerator offsetHeightAboveGroundTrajectory =
      new CubicPolynomialTrajectoryGenerator("offsetHeightAboveGroundTrajectory", offsetHeightAboveGroundInitialPositionProvider,
         offsetHeightAboveGroundFinalPositionProvider, offsetHeightAboveGroundTrajectoryTimeProvider, registry);

   private final DoubleYoVariable minimumHeightAboveGround = new DoubleYoVariable("minimumHeightAboveGround", registry);
   private final DoubleYoVariable nominalHeightAboveGround = new DoubleYoVariable("nominalHeightAboveGround", registry);
   private final DoubleYoVariable maximumHeightAboveGround = new DoubleYoVariable("maximumHeightAboveGround", registry);

   private final DoubleYoVariable maximumHeightDeltaBetweenWaypoints = new DoubleYoVariable("maximumHeightDeltaBetweenWaypoints", registry);
   private final DoubleYoVariable doubleSupportPercentageIn = new DoubleYoVariable("doubleSupportPercentageIn", registry);

   private final DoubleYoVariable previousZFinalLeft = new DoubleYoVariable("previousZFinalLeft", registry);
   private final DoubleYoVariable previousZFinalRight = new DoubleYoVariable("previousZFinalRight", registry);
   private final SideDependentList<DoubleYoVariable> previousZFinals = new SideDependentList<DoubleYoVariable>(previousZFinalLeft, previousZFinalRight);

   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);

   private LineSegment2d projectionSegment;

   private final YoFramePoint contactFrameZeroPosition = new YoFramePoint("contactFrameZeroPosition", worldFrame, registry);
   private final YoFramePoint contactFrameOnePosition = new YoFramePoint("contactFrameOnePosition", worldFrame, registry);

   private final YoGraphicPosition pointS0Viz, pointSFViz, pointD0Viz, pointDFViz, pointSNextViz;
   private final YoGraphicPosition pointS0MinViz, pointSFMinViz, pointD0MinViz, pointDFMinViz, pointSNextMinViz;
   private final YoGraphicPosition pointS0MaxViz, pointSFMaxViz, pointD0MaxViz, pointDFMaxViz, pointSNextMaxViz;

   private final BooleanYoVariable correctForCoMHeightDrift = new BooleanYoVariable("correctForCoMHeightDrift", registry);
   private final BooleanYoVariable initializeToCurrent = new BooleanYoVariable("initializeCoMHeightToCurrent", registry);

   private final BagOfBalls bagOfBalls;

   private WalkOnTheEdgesManager walkOnTheEdgesManager;
   private double extraCoMMaxHeightWithToes = 0.0;

   private final DoubleYoVariable yoTime;

   private ReferenceFrame frameOfLastFoostep;
   private final ReferenceFrame pelvisFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   public LookAheadCoMHeightTrajectoryGenerator(DesiredComHeightProvider desiredComHeightProvider, double minimumHeightAboveGround,
         double nominalHeightAboveGround, double maximumHeightAboveGround, double defaultOffsetHeightAboveGround, double doubleSupportPercentageIn,
         ReferenceFrame pelvisFrame, SideDependentList<ReferenceFrame> ankleZUpFrames, final DoubleYoVariable yoTime,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.desiredComHeightProvider = desiredComHeightProvider;
      this.pelvisFrame = pelvisFrame;
      this.ankleZUpFrames = ankleZUpFrames;
      frameOfLastFoostep = ankleZUpFrames.get(RobotSide.LEFT);
      this.yoTime = yoTime;
      offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
      offsetHeightAboveGroundTrajectoryTimeProvider.set(0.5);
      offsetHeightAboveGround.set(defaultOffsetHeightAboveGround);
      offsetHeightAboveGroundPrevValue.set(defaultOffsetHeightAboveGround);
      offsetHeightAboveGround.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
            offsetHeightAboveGroundTrajectory.initialize();
         }
      });

      setMinimumHeightAboveGround(minimumHeightAboveGround);
      setNominalHeightAboveGround(nominalHeightAboveGround);
      setMaximumHeightAboveGround(maximumHeightAboveGround);
      previousZFinalLeft.set(nominalHeightAboveGround);
      previousZFinalRight.set(nominalHeightAboveGround);

      hasBeenInitializedWithNextStep.set(false);

      this.doubleSupportPercentageIn.set(doubleSupportPercentageIn);

      this.maximumHeightDeltaBetweenWaypoints.set(0.2);    // 0.04);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry == null)
      {
         VISUALIZE = false;
      }

      if (VISUALIZE)
      {
         double pointSize = 0.03;

         YoGraphicPosition position0 = new YoGraphicPosition("contactFrame0", contactFrameZeroPosition, pointSize, YoAppearance.Purple());
         YoGraphicPosition position1 = new YoGraphicPosition("contactFrame1", contactFrameOnePosition, pointSize, YoAppearance.Orange());

         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", position0);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", position1);

         pointS0Viz = new YoGraphicPosition("pointS0", "", registry, pointSize, YoAppearance.CadetBlue());
         pointSFViz = new YoGraphicPosition("pointSF", "", registry, pointSize, YoAppearance.Chartreuse());
         pointD0Viz = new YoGraphicPosition("pointD0", "", registry, pointSize, YoAppearance.BlueViolet());
         pointDFViz = new YoGraphicPosition("pointDF", "", registry, pointSize, YoAppearance.Azure());
         pointSNextViz = new YoGraphicPosition("pointSNext", "", registry, pointSize, YoAppearance.Pink());

         pointS0MinViz = new YoGraphicPosition("pointS0Min", "", registry, 0.8 * pointSize, YoAppearance.CadetBlue());
         pointSFMinViz = new YoGraphicPosition("pointSFMin", "", registry, 0.8 * pointSize, YoAppearance.Chartreuse());
         pointD0MinViz = new YoGraphicPosition("pointD0Min", "", registry, 0.8 * pointSize, YoAppearance.BlueViolet());
         pointDFMinViz = new YoGraphicPosition("pointDFMin", "", registry, 0.8 * pointSize, YoAppearance.Azure());
         pointSNextMinViz = new YoGraphicPosition("pointSNextMin", "", registry, 0.8 * pointSize, YoAppearance.Pink());

         pointS0MaxViz = new YoGraphicPosition("pointS0Max", "", registry, 0.9 * pointSize, YoAppearance.CadetBlue());
         pointSFMaxViz = new YoGraphicPosition("pointSFMax", "", registry, 0.9 * pointSize, YoAppearance.Chartreuse());
         pointD0MaxViz = new YoGraphicPosition("pointD0Max", "", registry, 0.9 * pointSize, YoAppearance.BlueViolet());
         pointDFMaxViz = new YoGraphicPosition("pointDFMax", "", registry, 0.9 * pointSize, YoAppearance.Azure());
         pointSNextMaxViz = new YoGraphicPosition("pointSNextMax", "", registry, 0.9 * pointSize, YoAppearance.Pink());

         bagOfBalls = new BagOfBalls(registry, yoGraphicsListRegistry);

         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointS0Viz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointSFViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointD0Viz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointDFViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointSNextViz);

         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointS0MinViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointSFMinViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointD0MinViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointDFMinViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointSNextMinViz);

         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointS0MaxViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointSFMaxViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointD0MaxViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointDFMaxViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointSNextMaxViz);

         YoGraphicPosition desiredCoMPositionViz = new YoGraphicPosition("desiredCoMPosition", desiredCoMPosition, 1.1 * pointSize, YoAppearance.Gold());
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", desiredCoMPositionViz);

      }
      else
      {
         pointS0Viz = null;
         pointSFViz = null;
         pointD0Viz = null;
         pointDFViz = null;
         pointSNextViz = null;

         pointS0MinViz = null;
         pointSFMinViz = null;
         pointD0MinViz = null;
         pointDFMinViz = null;
         pointSNextMinViz = null;

         pointS0MaxViz = null;
         pointSFMaxViz = null;
         pointD0MaxViz = null;
         pointDFMaxViz = null;
         pointSNextMaxViz = null;

         bagOfBalls = null;
      }
   }

   @Override
   public void attachWalkOnToesManager(WalkOnTheEdgesManager walkOnTheEdgesManager)
   {
      this.walkOnTheEdgesManager = walkOnTheEdgesManager;
      extraCoMMaxHeightWithToes = walkOnTheEdgesManager.getExtraCoMMaxHeightWithToes();
   }

   public void setMinimumHeightAboveGround(double minimumHeightAboveGround)
   {
      this.minimumHeightAboveGround.set(minimumHeightAboveGround);
   }

   public void setNominalHeightAboveGround(double nominalHeightAboveGround)
   {
      this.nominalHeightAboveGround.set(nominalHeightAboveGround);
   }

   public void setMaximumHeightAboveGround(double maximumHeightAboveGround)
   {
      this.maximumHeightAboveGround.set(maximumHeightAboveGround);
   }

   public void setCoMHeightDriftCompensation(boolean activate)
   {
      correctForCoMHeightDrift.set(activate);
   }

   private final Point2d[] points = new Point2d[4];
   private final double[] endpointSlopes = new double[] {0.0, 0.0};
   private final double[] waypointSlopes = new double[2];
   
   @Override
   public void setSupportLeg(RobotSide supportLeg)
   {
      ReferenceFrame newFrame = ankleZUpFrames.get(supportLeg);

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, s0.getY());
      tempFramePoint.changeFrame(newFrame);
      s0.setY(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, d0.getY());
      tempFramePoint.changeFrame(newFrame);
      d0.setY(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, dF.getY());
      tempFramePoint.changeFrame(newFrame);
      dF.setY(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, sF.getY());
      tempFramePoint.changeFrame(newFrame);
      sF.setY(tempFramePoint.getZ());
      
      points[0] = s0;
      points[1] = d0;
      points[2] = dF;
      points[3] = sF;
      
      endpointSlopes[0] = 0.0;
      endpointSlopes[1] = 0.0;

      waypointSlopes[0] = (points[2].y - points[0].y) / (points[2].x - points[0].x);
      waypointSlopes[1] = (points[3].y - points[1].y) / (points[3].x - points[1].x);

      spline.setPoints(points, endpointSlopes, waypointSlopes);

      for (RobotSide robotSide : RobotSide.values)
      {
         DoubleYoVariable previousZFinal = previousZFinals.get(robotSide);
         tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, previousZFinal.getDoubleValue());
         tempFramePoint.changeFrame(newFrame);
         previousZFinal.set(tempFramePoint.getZ());
      }
      
      frameOfLastFoostep = newFrame;
   }

   @Override
   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide supportLeg, Footstep nextFootstep,
                          List<PlaneContactState> contactStates)
   {
      initialize(transferToAndNextFootstepsData);
   }

   private final Point2d tempPoint2dA = new Point2d();
   private final Point2d tempPoint2dB = new Point2d();

   private final Point2d s0Min = new Point2d();
   private final Point2d d0Min = new Point2d();
   private final Point2d dFMin = new Point2d();
   private final Point2d sFMin = new Point2d();
   private final Point2d sNextMin = new Point2d();

   private final Point2d s0Nom = new Point2d();
   private final Point2d d0Nom = new Point2d();
   private final Point2d dFNom = new Point2d();
   private final Point2d sFNom = new Point2d();
   private final Point2d sNextNom = new Point2d();

   private final Point2d s0Max = new Point2d();
   private final Point2d d0Max = new Point2d();
   private final Point2d dFMax = new Point2d();
   private final Point2d sFMax = new Point2d();
   private final Point2d sNextMax = new Point2d();

   private final Point2d s0 = new Point2d();
   private final Point2d d0 = new Point2d();
   private final Point2d dF = new Point2d();
   private final Point2d sF = new Point2d();
   private final Point2d sNext = new Point2d();

   private final FramePoint framePointS0 = new FramePoint();
   private final FramePoint framePointD0 = new FramePoint();
   private final FramePoint framePointDF = new FramePoint();
   private final FramePoint framePointSF = new FramePoint();
   private final FramePoint framePointSNext = new FramePoint();

   private final FramePoint tempFramePointForViz1 = new FramePoint();
   private final FramePoint tempFramePointForViz2 = new FramePoint();

   private void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      if (DEBUG)
      {
         printOutFootstepGenerationCode(transferToAndNextFootstepsData);
      }

      Footstep transferFromFootstep = transferToAndNextFootstepsData.getTransferFromFootstep();
      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      Footstep transferFromDesiredFootstep = transferToAndNextFootstepsData.getTransferFromDesiredFootstep();

      Footstep nextFootstep = null;

      if (CONSIDER_NEXT_FOOTSTEP)
         nextFootstep = transferToAndNextFootstepsData.getNextFootstep();

      if (nextFootstep != null)
         hasBeenInitializedWithNextStep.set(true);
      else
         hasBeenInitializedWithNextStep.set(false);

      FramePoint transferFromContactFramePosition = new FramePoint(transferFromFootstep.getPoseReferenceFrame());
      FramePoint transferToContactFramePosition = new FramePoint(transferToFootstep.getPoseReferenceFrame());

      FrameVector fromContactFrameDrift = null;

      if (correctForCoMHeightDrift.getBooleanValue() && (transferFromDesiredFootstep != null))
      {
         if (transferFromDesiredFootstep.getRobotSide() != transferFromFootstep.getRobotSide())
         {
            if(DEBUG)
            {
               System.err.println("transferFromDesiredFootstep.getRobotSide() != transferFromFootstep.getRobotSide() in LookAheadCoMHeightTrajectoryGenerator.initialize()");
            }
         }

         else
         {
            FramePoint transferFromDesiredContactFramePosition = new FramePoint(transferFromDesiredFootstep.getPoseReferenceFrame());
            transferFromDesiredContactFramePosition.changeFrame(transferFromContactFramePosition.getReferenceFrame());

            fromContactFrameDrift = new FrameVector(transferFromContactFramePosition.getReferenceFrame());
            fromContactFrameDrift.sub(transferFromContactFramePosition, transferFromDesiredContactFramePosition);
            fromContactFrameDrift.changeFrame(transferToContactFramePosition.getReferenceFrame());
            transferToContactFramePosition.setZ(transferToContactFramePosition.getZ() + fromContactFrameDrift.getZ());
         }
      }

      transferFromContactFramePosition.changeFrame(worldFrame);
      transferToContactFramePosition.changeFrame(worldFrame);

      FramePoint nextContactFramePosition = null;
      if (nextFootstep != null)
      {
         nextContactFramePosition = new FramePoint(nextFootstep.getPoseReferenceFrame());

         if (fromContactFrameDrift != null)
         {
            fromContactFrameDrift.changeFrame(nextContactFramePosition.getReferenceFrame());
            nextContactFramePosition.setZ(nextContactFramePosition.getZ() + fromContactFrameDrift.getZ());
         }

         nextContactFramePosition.changeFrame(worldFrame);
      }

      contactFrameZeroPosition.set(transferFromContactFramePosition);
      contactFrameOnePosition.set(transferToContactFramePosition);

      getPoint2d(tempPoint2dA, transferFromContactFramePosition);
      getPoint2d(tempPoint2dB, transferToContactFramePosition);

      projectionSegment = new LineSegment2d(tempPoint2dA, tempPoint2dB);
      setPointXValues(nextContactFramePosition);

      transferFromContactFramePosition.changeFrame(frameOfLastFoostep);
      transferToContactFramePosition.changeFrame(frameOfLastFoostep);

      double footHeight0 = transferFromContactFramePosition.getZ();
      double footHeight1 = transferToContactFramePosition.getZ();

      double nextFootHeight = Double.NaN;
      if (nextContactFramePosition != null)
      {
         nextContactFramePosition.changeFrame(frameOfLastFoostep);
         nextFootHeight = nextContactFramePosition.getZ();
      }

      s0Min.setY(footHeight0 + minimumHeightAboveGround.getDoubleValue());
      s0Nom.setY(footHeight0 + nominalHeightAboveGround.getDoubleValue());
      s0Max.setY(footHeight0 + maximumHeightAboveGround.getDoubleValue());

      sFMin.setY(footHeight1 + minimumHeightAboveGround.getDoubleValue());
      sFNom.setY(footHeight1 + nominalHeightAboveGround.getDoubleValue());
      sFMax.setY(footHeight1 + maximumHeightAboveGround.getDoubleValue());

      d0Min.setY(findMinimumDoubleSupportHeight(s0.getX(), sF.getX(), d0.getX(), footHeight0, footHeight1));
      d0Nom.setY(findNominalDoubleSupportHeight(s0.getX(), sF.getX(), d0.getX(), footHeight0, footHeight1));

      if ((walkOnTheEdgesManager != null) && walkOnTheEdgesManager.willLandOnToes())
      {
         d0Max.setY(findMaximumDoubleSupportHeight(s0.getX(), sF.getX(), d0.getX(), footHeight0, footHeight1 + extraCoMMaxHeightWithToes));
      }
      else
      {
         d0Max.setY(findMaximumDoubleSupportHeight(s0.getX(), sF.getX(), d0.getX(), footHeight0, footHeight1));
      }

      dFMin.setY(findMinimumDoubleSupportHeight(s0.getX(), sF.getX(), dF.getX(), footHeight0, footHeight1));
      dFNom.setY(findNominalDoubleSupportHeight(s0.getX(), sF.getX(), dF.getX(), footHeight0, footHeight1));

      if ((walkOnTheEdgesManager != null)
              && walkOnTheEdgesManager.willDoToeOff(transferToAndNextFootstepsData))
      {
         dFMax.setY(findMaximumDoubleSupportHeight(s0.getX(), sF.getX(), dF.getX(), footHeight0 + extraCoMMaxHeightWithToes, footHeight1));
      }
      else
      {
         dFMax.setY(findMaximumDoubleSupportHeight(s0.getX(), sF.getX(), dF.getX(), footHeight0, footHeight1));
      }

      sNextMin.setY(nextFootHeight + minimumHeightAboveGround.getDoubleValue());
      sNextNom.setY(nextFootHeight + nominalHeightAboveGround.getDoubleValue());
      sNextMax.setY(nextFootHeight + maximumHeightAboveGround.getDoubleValue());

      computeHeightsToUseByStretchingString(transferFromFootstep.getRobotSide());
      previousZFinals.get(transferToFootstep.getRobotSide()).set(sF.getY());

      Point2d[] points = new Point2d[] {s0, d0, dF, sF};
      double[] endpointSlopes = new double[] {0.0, 0.0};

      double[] waypointSlopes = new double[2];
      waypointSlopes[0] = (points[2].y - points[0].y) / (points[2].x - points[0].x);
      waypointSlopes[1] = (points[3].y - points[1].y) / (points[3].x - points[1].x);

      spline.setPoints(points, endpointSlopes, waypointSlopes);

      if (VISUALIZE)
      {
         framePointS0.setIncludingFrame(transferFromContactFramePosition);
         framePointS0.setZ(s0.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointS0.changeFrame(worldFrame);
         pointS0Viz.setPosition(framePointS0);

         framePointS0.changeFrame(frameOfLastFoostep);
         framePointS0.setZ(s0Min.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointS0.changeFrame(worldFrame);
         pointS0MinViz.setPosition(framePointS0);

         framePointS0.changeFrame(frameOfLastFoostep);
         framePointS0.setZ(s0Max.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointS0.changeFrame(worldFrame);
         pointS0MaxViz.setPosition(framePointS0);

         framePointD0.setToZero(transferFromContactFramePosition.getReferenceFrame());
         framePointD0.interpolate(transferFromContactFramePosition, transferToContactFramePosition, d0.getX() / sF.getX());
         framePointD0.setZ(d0.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointD0.changeFrame(worldFrame);
         pointD0Viz.setPosition(framePointD0);

         framePointD0.changeFrame(frameOfLastFoostep);
         framePointD0.setZ(d0Min.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointD0.changeFrame(worldFrame);
         pointD0MinViz.setPosition(framePointD0);

         framePointD0.changeFrame(frameOfLastFoostep);
         framePointD0.setZ(d0Max.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointD0.changeFrame(worldFrame);
         pointD0MaxViz.setPosition(framePointD0);

         framePointDF.setToZero(transferFromContactFramePosition.getReferenceFrame());
         framePointDF.interpolate(transferFromContactFramePosition, transferToContactFramePosition, dF.getX() / sF.getX());
         framePointDF.setZ(dF.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointDF.changeFrame(worldFrame);
         pointDFViz.setPosition(framePointDF);

         framePointDF.changeFrame(frameOfLastFoostep);
         framePointDF.setZ(dFMin.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointDF.changeFrame(worldFrame);
         pointDFMinViz.setPosition(framePointDF);

         framePointDF.changeFrame(frameOfLastFoostep);
         framePointDF.setZ(dFMax.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointDF.changeFrame(worldFrame);
         pointDFMaxViz.setPosition(framePointDF);

         framePointSF.setIncludingFrame(transferToContactFramePosition);
         framePointSF.setZ(sF.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointSF.changeFrame(worldFrame);
         pointSFViz.setPosition(framePointSF);

         framePointSF.changeFrame(frameOfLastFoostep);
         framePointSF.setZ(sFMin.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointSF.changeFrame(worldFrame);
         pointSFMinViz.setPosition(framePointSF);

         framePointSF.changeFrame(frameOfLastFoostep);
         framePointSF.setZ(sFMax.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointSF.changeFrame(worldFrame);
         pointSFMaxViz.setPosition(framePointSF);

         if (nextContactFramePosition != null)
         {
            framePointSNext.setIncludingFrame(nextContactFramePosition);
            framePointSNext.setZ(sNext.getY() + offsetHeightAboveGround.getDoubleValue());
            framePointSNext.changeFrame(worldFrame);
            pointSNextViz.setPosition(framePointSNext);

            framePointSNext.changeFrame(frameOfLastFoostep);
            framePointSNext.setZ(sNextMin.getY() + offsetHeightAboveGround.getDoubleValue());
            framePointSNext.changeFrame(worldFrame);
            pointSNextMinViz.setPosition(framePointSNext);

            framePointSNext.changeFrame(frameOfLastFoostep);
            framePointSNext.setZ(sNextMax.getY() + offsetHeightAboveGround.getDoubleValue());
            framePointSNext.changeFrame(worldFrame);
            pointSNextMaxViz.setPosition(framePointSNext);
         }
         else
         {
            pointSNextViz.setPositionToNaN();
            pointSNextMinViz.setPositionToNaN();
            pointSNextMaxViz.setPositionToNaN();
         }

         bagOfBalls.reset();
         int numberOfPoints = 30;
         CoMHeightPartialDerivativesData coMHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
         for (int i = 0; i < numberOfPoints; i++)
         {
            tempFramePointForViz1.setToZero(transferFromContactFramePosition.getReferenceFrame());
            tempFramePointForViz1.interpolate(transferFromContactFramePosition, transferToContactFramePosition, ((double) i) / ((double) numberOfPoints));
            tempFramePointForViz1.changeFrame(worldFrame);
            Point2d queryPoint = new Point2d(tempFramePointForViz1.getX(), tempFramePointForViz1.getY());
            this.solve(coMHeightPartialDerivativesData, queryPoint, false);
            coMHeightPartialDerivativesData.getCoMHeight(tempFramePointForViz2);
            tempFramePointForViz2.setX(tempFramePointForViz1.getX());
            tempFramePointForViz2.setY(tempFramePointForViz1.getY());

            bagOfBalls.setBallLoop(tempFramePointForViz2);
         }
      }
   }

   private Point2d tempPoint2dForStringStretching = new Point2d();

   private void computeHeightsToUseByStretchingString(RobotSide transferFromSide)
   {
      // s0 is at previous
      double z0 = MathTools.clipToMinMax(previousZFinals.get(transferFromSide).getDoubleValue(), s0Min.getY(), s0Max.getY());
      s0.setY(z0);

      StringStretcher2d stringStretcher2d = new StringStretcher2d();
      stringStretcher2d.setStartPoint(s0);

      if (!Double.isNaN(sNext.getX()))
      {
         if (sNext.getX() < sF.getX() + 0.01)
         {
            sNext.setX(sF.getX() + 0.01);
         }

         double zNext;
         if (sNextMin.getY() > sFNom.getY())
         {
            zNext = sNextMin.getY();
         }
         else if (sNextMax.getY() < sFNom.getY())
         {
            zNext = sNextMax.getY();
         }
         else
         {
            zNext = sFNom.getY();
         }

         sNext.setY(zNext);

         stringStretcher2d.setEndPoint(sNext);

         stringStretcher2d.addMinMaxPoints(d0Min, d0Max);
         stringStretcher2d.addMinMaxPoints(dFMin, dFMax);
         stringStretcher2d.addMinMaxPoints(sFMin, sFMax);
      }
      else
      {
         double sFNomHeight = sFNom.getY();

         // If the final single support nominal position is higher than the final double support max position, decrease it
         // but don't decrease it below the final single support minimum height.
         if (sFNomHeight > dFMax.getY())
         {
            sFNomHeight = Math.max(dFMax.getY(), sFMin.getY());
            tempPoint2dForStringStretching.set(sFNom);
            tempPoint2dForStringStretching.setY(sFNomHeight);
            stringStretcher2d.setEndPoint(tempPoint2dForStringStretching);
         }
         else
         {
            stringStretcher2d.setEndPoint(sFNom);
         }

         stringStretcher2d.addMinMaxPoints(d0Min, d0Max);
         stringStretcher2d.addMinMaxPoints(dFMin, dFMax);
      }

      List<Point2d> stretchedString = stringStretcher2d.stretchString();

      d0.set(stretchedString.get(1));
      dF.set(stretchedString.get(2));
      sF.set(stretchedString.get(3));
   }

   private void setPointXValues(FramePoint nextContactFramePosition)
   {
      double length = projectionSegment.length();

      double xS0 = 0.0;
      double xD0 = doubleSupportPercentageIn.getDoubleValue() * length;
      double xDF = (1.0 - doubleSupportPercentageIn.getDoubleValue()) * length;
      double xSF = length;

      double xSNext = Double.NaN;
      if (nextContactFramePosition != null)
      {
         Line2d line2d = new Line2d(projectionSegment.getFirstEndPointCopy(), projectionSegment.getSecondEndPointCopy());
         Point2d nextPoint2d = new Point2d(nextContactFramePosition.getX(), nextContactFramePosition.getY());
         line2d.orthogonalProjectionCopy(nextPoint2d);
         xSNext = projectionSegment.percentageAlongLineSegment(nextPoint2d) * projectionSegment.length();
      }

      s0.setX(xS0);
      d0.setX(xD0);
      dF.setX(xDF);
      sF.setX(xSF);
      sNext.setX(xSNext);

      s0Min.setX(xS0);
      d0Min.setX(xD0);
      dFMin.setX(xDF);
      sFMin.setX(xSF);
      sNextMin.setX(xSNext);

      s0Nom.setX(xS0);
      d0Nom.setX(xD0);
      dFNom.setX(xDF);
      sFNom.setX(xSF);
      sNextNom.setX(xSNext);

      s0Max.setX(xS0);
      d0Max.setX(xD0);
      dFMax.setX(xDF);
      sFMax.setX(xSF);
      sNextMax.setX(xSNext);
   }

   public double findMinimumDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      return findDoubleSupportHeight(minimumHeightAboveGround.getDoubleValue(), s0, sF, s_d0, foot0Height, foot1Height);
   }

   public double findNominalDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      return findDoubleSupportHeight(nominalHeightAboveGround.getDoubleValue(), s0, sF, s_d0, foot0Height, foot1Height);
   }

   public double findMaximumDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      return findDoubleSupportHeight(maximumHeightAboveGround.getDoubleValue(), s0, sF, s_d0, foot0Height, foot1Height);
   }

   private double findDoubleSupportHeight(double distanceFromFoot, double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      double z_d0_A = foot0Height + Math.sqrt(MathTools.square(distanceFromFoot) - MathTools.square(s_d0 - s0));
      double z_d0_B = foot1Height + Math.sqrt(MathTools.square(distanceFromFoot) - MathTools.square((sF - s_d0)));
      double z_d0 = Math.min(z_d0_A, z_d0_B);

      return z_d0;
   }

   private void getPoint2d(Point2d point2dToPack, FramePoint point)
   {
      point2dToPack.set(point.getX(), point.getY());
   }

   private final FramePoint tempFramePoint = new FramePoint();
   private final Point2d queryPoint = new Point2d();
   private final Point2d solutionPoint = new Point2d();

   @Override
   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData)
   {
      getCenterOfMass2d(queryPoint, centerOfMassHeightInputData.getCenterOfMassFrame());
      solutionPoint.set(queryPoint);
      boolean isInDoubleSupport = centerOfMassHeightInputData.getSupportLeg() == null;
      solve(coMHeightPartialDerivativesDataToPack, solutionPoint, isInDoubleSupport);

      coMHeightPartialDerivativesDataToPack.getCoMHeight(tempFramePoint);
      desiredCoMPosition.set(queryPoint.getX(), queryPoint.getY(), tempFramePoint.getZ());
   }

   private final FramePoint height = new FramePoint();
   private final FramePoint desiredPosition = new FramePoint();
   private final double[] splineOutput = new double[3];
   private final double[] partialDerivativesWithRespectToS = new double[2];
   
   private void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, Point2d queryPoint, boolean isInDoubleSupport)
   {
      projectionSegment.orthogonalProjection(queryPoint);
      double splineQuery = projectionSegment.percentageAlongLineSegment(queryPoint) * projectionSegment.length();

      // Happens when the robot gets stuck in double support but the ICP is still being dragged in the front support foot.
      if (isInDoubleSupport)
         splineQuery = Math.min(splineQuery, dF.getX());

      spline.getZSlopeAndSecondDerivative(splineQuery, splineOutput);

      if (initializeToCurrent.getBooleanValue())
      {
         initializeToCurrent.set(false);

         desiredPosition.setToZero(pelvisFrame);
         desiredPosition.changeFrame(frameOfLastFoostep);

         double heightOffset = desiredPosition.getZ() - splineOutput[0];

         offsetHeightAboveGround.set(heightOffset);
         offsetHeightAboveGroundTrajectoryTimeProvider.set(0.0);
         offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
         offsetHeightAboveGroundTrajectory.initialize();
      }
      else if (desiredComHeightProvider != null)
      {
         if (desiredComHeightProvider.isNewComHeightInformationAvailable())
         {
            offsetHeightAboveGround.set(desiredComHeightProvider.getComHeightOffset());
            offsetHeightAboveGroundTrajectoryTimeProvider.set(desiredComHeightProvider.getComHeightTrajectoryTime());
            offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
            offsetHeightAboveGroundTrajectory.initialize();
         }
         else if (desiredComHeightProvider.isNewComHeightMultipointAvailable())
         {
            WaypointPositionTrajectoryData pelvisTrajectory = desiredComHeightProvider.getComHeightMultipointWorldPosition();

            int lastIndex = pelvisTrajectory.getTimeAtWaypoints().length - 1;

            // TODO (Sylvain) Check if that's the right way to do it
            desiredPosition.setIncludingFrame(worldFrame, pelvisTrajectory.getPositions()[lastIndex]);
            desiredPosition.changeFrame(frameOfLastFoostep);

            double heightOffset = desiredPosition.getZ() - splineOutput[0];

            // it is not really the last time, since we have the "settling"
            double totalTime = pelvisTrajectory.getTimeAtWaypoints()[lastIndex];

            offsetHeightAboveGround.set(heightOffset);
            offsetHeightAboveGroundTrajectoryTimeProvider.set(totalTime);
            offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
            offsetHeightAboveGroundTrajectory.initialize();
         }
      }

      offsetHeightAboveGroundTrajectory.compute(yoTime.getDoubleValue() - offsetHeightAboveGroundChangedTime.getDoubleValue());
      offsetHeightAboveGroundTrajectoryOutput.set(offsetHeightAboveGroundTrajectory.getValue());

      offsetHeightAboveGroundPrevValue.set(offsetHeightAboveGroundTrajectory.getValue());

      double z = splineOutput[0] + offsetHeightAboveGroundTrajectoryOutput.getValue();
      double dzds = splineOutput[1];
      double ddzdds = splineOutput[2];

      getPartialDerivativesWithRespectToS(projectionSegment, partialDerivativesWithRespectToS);
      double dsdx = partialDerivativesWithRespectToS[0];
      double dsdy = partialDerivativesWithRespectToS[1];
      double ddsddx = 0;
      double ddsddy = 0;
      double ddsdxdy = 0;

      double dzdx = dsdx * dzds;
      double dzdy = dsdy * dzds;
      double ddzddx = dzds * ddsddx + ddzdds * dsdx * dsdx;
      double ddzddy = dzds * ddsddy + ddzdds * dsdy * dsdy;
      double ddzdxdy = ddzdds * dsdx * dsdy + dzds * ddsdxdy;

      height.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, z);
      height.changeFrame(worldFrame);
      coMHeightPartialDerivativesDataToPack.setCoMHeight(worldFrame, height.getZ());
      coMHeightPartialDerivativesDataToPack.setPartialDzDx(dzdx);
      coMHeightPartialDerivativesDataToPack.setPartialDzDy(dzdy);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDxDy(ddzdxdy);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDx2(ddzddx);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDy2(ddzddy);

      desiredCoMHeight.set(z);
   }

   public void initializeDesiredHeightToCurrent()
   {
      initializeToCurrent.set(true);
   }

   private void getPartialDerivativesWithRespectToS(LineSegment2d segment, double[] partialDerivativesToPack)
   {
      double dsdx = (segment.getX1() - segment.getX0()) / segment.length();
      double dsdy = (segment.getY1() - segment.getY0()) / segment.length();

      partialDerivativesToPack[0] = dsdx;
      partialDerivativesToPack[1] = dsdy;
   }

   private final FramePoint coM = new FramePoint();

   private void getCenterOfMass2d(Point2d point2dToPack, ReferenceFrame centerOfMassFrame)
   {
      coM.setToZero(centerOfMassFrame);
      coM.changeFrame(worldFrame);

      getPoint2d(point2dToPack, coM);
   }

   @Override
   public boolean hasBeenInitializedWithNextStep()
   {
      return hasBeenInitializedWithNextStep.getBooleanValue();
   }

   public void setOffsetHeightAboveGround(double value)
   {
      offsetHeightAboveGround.set(value);
   }

   public double getOffsetHeightAboveGround()
   {
      return offsetHeightAboveGround.getDoubleValue();
   }

   private void printOutFootstepGenerationCode(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      Footstep transferFromFootstep = transferToAndNextFootstepsData.getTransferFromFootstep();
      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
      Footstep transferFromDesiredFootstep = transferToAndNextFootstepsData.getTransferFromDesiredFootstep();
      RobotSide transferToSide = transferToAndNextFootstepsData.getTransferToSide();

      System.out.println("\nLookAheadCoMHeightTrajectoryGenerator.initialize()");
      System.out.println("transferFromFootstep:");
      printFootstepConstructor(transferFromFootstep);

      System.out.println("transferToFootstep:");
      printFootstepConstructor(transferToFootstep);

      if (nextFootstep != null)
      {
         System.out.println("nextFootstep:");
         printFootstepConstructor(nextFootstep);
      }

      if (transferFromDesiredFootstep != null)
      {
         System.out.println("transferFromDesiredFootstep:");
         printFootstepConstructor(transferFromDesiredFootstep);
      }

      System.out.println("transferToSide = " + transferToSide);
   }

   private void printFootstepConstructor(Footstep footstep)
   {
      RobotSide robotSide = footstep.getRobotSide();
      Point3d position = new Point3d();
      footstep.getPositionInWorldFrame(position);
      Quat4d orientation = new Quat4d();
      footstep.getOrientationInWorldFrame(orientation);

      System.out.println("footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide." + robotSide + ", new Point3d(" + position.getX() + ", "
                         + position.getY() + ", " + position.getZ() + "), new Quat4d(" + orientation.getW() + ", " + orientation.getX() + ", "
                         + orientation.getY() + ", " + orientation.getZ() + ")));");
   }
}
