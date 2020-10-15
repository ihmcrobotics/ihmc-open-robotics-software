package us.ihmc.commonWalkingControlModules.heightPlanning;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.trajectories.YoFourPointCubicSpline1D;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.StringStretcher2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * TODO There is not enough HumanoidReferenceFrames in that class, it is pretty fragile
 */
public class LookAheadCoMHeightTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean CONSIDER_NEXT_FOOTSTEP = false;

   private static final boolean DEBUG = false;

   private boolean visualize = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoFourPointCubicSpline1D spline = new YoFourPointCubicSpline1D("height", registry);

   private final YoBoolean hasBeenInitializedWithNextStep = new YoBoolean("hasBeenInitializedWithNextStep", registry);


   private final HeightOffsetHandler heightOffsetHandler;

   private final YoDouble minimumHeightAboveGround = new YoDouble("minimumHeightAboveGround", registry);
   private final YoDouble nominalHeightAboveGround = new YoDouble("nominalHeightAboveGround", registry);
   private final YoDouble maximumHeightAboveGround = new YoDouble("maximumHeightAboveGround", registry);

   private final YoDouble doubleSupportPercentageIn = new YoDouble("doubleSupportPercentageIn", registry);

   private final YoDouble previousZFinalLeft = new YoDouble("previousZFinalLeft", registry);
   private final YoDouble previousZFinalRight = new YoDouble("previousZFinalRight", registry);
   private final SideDependentList<YoDouble> previousZFinals = new SideDependentList<YoDouble>(previousZFinalLeft, previousZFinalRight);

   private final YoDouble desiredCoMHeight = new YoDouble("desiredCoMHeight", registry);
   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);

   private final LineSegment2D projectionSegment = new LineSegment2D();

   private final YoFramePoint3D contactFrameZeroPosition = new YoFramePoint3D("contactFrameZeroPosition", worldFrame, registry);
   private final YoFramePoint3D contactFrameOnePosition = new YoFramePoint3D("contactFrameOnePosition", worldFrame, registry);

   private final YoGraphicPosition pointS0Viz, pointSFViz, pointD0Viz, pointDFViz, pointSNextViz;
   private final YoGraphicPosition pointS0MinViz, pointSFMinViz, pointD0MinViz, pointDFMinViz, pointSNextMinViz;
   private final YoGraphicPosition pointS0MaxViz, pointSFMaxViz, pointD0MaxViz, pointDFMaxViz, pointSNextMaxViz;

   private final YoBoolean correctForCoMHeightDrift = new YoBoolean("correctForCoMHeightDrift", registry);
   private final YoBoolean initializeToCurrent = new YoBoolean("initializeCoMHeightToCurrent", registry);

   private final BagOfBalls bagOfBalls;

   private final DoubleProvider yoTime;

   private ReferenceFrame frameOfLastFoostep;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<? extends ReferenceFrame> ankleZUpFrames;

   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole;


   private final BooleanProvider processGoHome = new BooleanParameter("ProcessGoHome", registry, false);

   public LookAheadCoMHeightTrajectoryGenerator(double minimumHeightAboveGround, double nominalHeightAboveGround, double maximumHeightAboveGround,
                                                double defaultOffsetHeightAboveGround, double doubleSupportPercentageIn, ReferenceFrame centerOfMassFrame,
                                                ReferenceFrame pelvisFrame, SideDependentList<? extends ReferenceFrame> ankleZUpFrames,
                                                SideDependentList<RigidBodyTransform> transformsFromAnkleToSole, DoubleProvider yoTime,
                                                YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this.pelvisFrame = pelvisFrame;
      this.centerOfMassFrame = centerOfMassFrame;
      this.ankleZUpFrames = ankleZUpFrames;
      frameOfLastFoostep = ankleZUpFrames.get(RobotSide.LEFT);
      this.yoTime = yoTime;
      this.transformsFromAnkleToSole = transformsFromAnkleToSole;

      heightOffsetHandler = new HeightOffsetHandler(yoTime, defaultOffsetHeightAboveGround, registry);

      setMinimumHeightAboveGround(minimumHeightAboveGround);
      setNominalHeightAboveGround(nominalHeightAboveGround);
      setMaximumHeightAboveGround(maximumHeightAboveGround);
      previousZFinalLeft.set(nominalHeightAboveGround);
      previousZFinalRight.set(nominalHeightAboveGround);

      hasBeenInitializedWithNextStep.set(false);

      this.doubleSupportPercentageIn.set(doubleSupportPercentageIn);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry == null)
         visualize = false;

      if (visualize)
      {
         double pointSize = 0.03;

         YoGraphicPosition position0 = new YoGraphicPosition("contactFrame0", contactFrameZeroPosition, pointSize, YoAppearance.Purple());
         YoGraphicPosition position1 = new YoGraphicPosition("contactFrame1", contactFrameOnePosition, pointSize, YoAppearance.Orange());

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

         bagOfBalls = new BagOfBalls(15, registry, yoGraphicsListRegistry);

         YoGraphicPosition desiredCoMPositionViz = new YoGraphicPosition("desiredCoMPosition", desiredCoMPosition, 1.1 * pointSize, YoAppearance.Gold());

         String graphicListName = "CoMHeightTrajectoryGenerator";
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, position0);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, position1);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0Viz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0Viz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSNextViz);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0MinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFMinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFMinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSNextMinViz);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0MaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFMaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFMaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSNextMaxViz);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, desiredCoMPositionViz);

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

   public void reset()
   {
      hasBeenInitializedWithNextStep.set(false);
      heightOffsetHandler.reset();
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

   public void setSupportLeg(RobotSide supportLeg)
   {
      ReferenceFrame newFrame = ankleZUpFrames.get(supportLeg);

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, startWaypoint.getY());
      tempFramePoint.changeFrame(newFrame);
      startWaypoint.setY(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, firstMidpointPosition.getY());
      tempFramePoint.changeFrame(newFrame);
      firstMidpointPosition.setY(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, secondMidpointPosition.getY());
      tempFramePoint.changeFrame(newFrame);
      secondMidpointPosition.setY(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, endWaypoint.getY());
      tempFramePoint.changeFrame(newFrame);
      endWaypoint.setY(tempFramePoint.getZ());

      spline.initialize(startWaypoint, firstMidpointPosition, secondMidpointPosition, endWaypoint);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoDouble previousZFinal = previousZFinals.get(robotSide);
         tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, previousZFinal.getDoubleValue());
         tempFramePoint.changeFrame(newFrame);
         previousZFinal.set(tempFramePoint.getZ());
      }

      frameOfLastFoostep = newFrame;
      heightOffsetHandler.setReferenceFrame(frameOfLastFoostep);
   }

   private final Point2D tempPoint2dA = new Point2D();
   private final Point2D tempPoint2dB = new Point2D();

   private final Point2D startWaypointMinimum = new Point2D();
   private final Point2D firstMidpointMinimum = new Point2D();
   private final Point2D secondMidpointMinimum = new Point2D();
   private final Point2D endWaypointMinimum = new Point2D();
   private final Point2D sNextMin = new Point2D();

   private final Point2D startWaypointNominal = new Point2D();
   private final Point2D firstMidpointNominal = new Point2D();
   private final Point2D secondMidpointNominal = new Point2D();
   private final Point2D endWaypointNominal = new Point2D();
   private final Point2D sNextNom = new Point2D();

   private final Point2D startWaypointMaximum = new Point2D();
   private final Point2D firstMidpointMaximum = new Point2D();
   private final Point2D secondMidpointMaximum = new Point2D();
   private final Point2D endWaypointMaximum = new Point2D();
   private final Point2D sNextMax = new Point2D();

   private final Point2D startWaypoint = new Point2D();
   private final Point2D firstMidpointPosition = new Point2D();
   private final Point2D secondMidpointPosition = new Point2D();
   private final Point2D endWaypoint = new Point2D();
   private final Point2D sNext = new Point2D();

   private final FramePoint3D framePointS0 = new FramePoint3D();
   private final FramePoint3D framePointD0 = new FramePoint3D();
   private final FramePoint3D framePointDF = new FramePoint3D();
   private final FramePoint3D framePointSF = new FramePoint3D();
   private final FramePoint3D framePointSNext = new FramePoint3D();

   private final FramePoint3D tempFramePointForViz1 = new FramePoint3D();
   private final FramePoint3D tempFramePointForViz2 = new FramePoint3D();

   private final FramePoint3D transferFromAnklePosition = new FramePoint3D();
   private final FramePoint3D transferToAnklePosition = new FramePoint3D();
   private final FramePoint3D transferFromDesiredContactFramePosition = new FramePoint3D();

   private final FrameVector3D fromContactFrameDrift = new FrameVector3D();
   private final CoMHeightPartialDerivativesData coMHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
   private final Point2D queryPoint = new Point2D();

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
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
      {
         nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
         if (nextFootstep != null)
            hasBeenInitializedWithNextStep.set(true);
         else
            hasBeenInitializedWithNextStep.set(false);
      }
      else
      {
         hasBeenInitializedWithNextStep.set(false);
      }

      transferFromFootstep.getAnklePosition(transferFromAnklePosition, transformsFromAnkleToSole.get(transferFromFootstep.getRobotSide()));
      transferToFootstep.getAnklePosition(transferToAnklePosition, transformsFromAnkleToSole.get(transferToFootstep.getRobotSide()));

      boolean frameDrifted = false;
      if (correctForCoMHeightDrift.getBooleanValue() && transferToFootstep.getTrustHeight() && (transferFromDesiredFootstep != null))
      {
         if (transferFromDesiredFootstep.getRobotSide() != transferFromFootstep.getRobotSide())
         {
            if (DEBUG)
            {
               System.err.println("transferFromDesiredFootstep.getRobotSide() != transferFromFootstep.getRobotSide() in LookAheadCoMHeightTrajectoryGenerator.initialize()");
            }
         }
         else
         {
            transferFromDesiredFootstep.getAnklePosition(transferFromDesiredContactFramePosition,
                                                         transformsFromAnkleToSole.get(transferFromDesiredFootstep.getRobotSide()));
            transferFromDesiredContactFramePosition.changeFrame(transferFromAnklePosition.getReferenceFrame());

            fromContactFrameDrift.setToZero(transferFromAnklePosition.getReferenceFrame());
            fromContactFrameDrift.sub(transferFromAnklePosition, transferFromDesiredContactFramePosition);
            fromContactFrameDrift.changeFrame(transferToAnklePosition.getReferenceFrame());
            transferToAnklePosition.setZ(transferToAnklePosition.getZ() + fromContactFrameDrift.getZ());
            frameDrifted = true;
         }
      }

      transferFromAnklePosition.changeFrame(worldFrame);
      transferToAnklePosition.changeFrame(worldFrame);

      FramePoint3D nextContactFramePosition = null;

      if (CONSIDER_NEXT_FOOTSTEP)
      {
         if (nextFootstep != null)
         {
            nextContactFramePosition = new FramePoint3D();
            nextFootstep.getAnklePosition(nextContactFramePosition, transformsFromAnkleToSole.get(nextFootstep.getRobotSide()));

            if (frameDrifted)
            {
               fromContactFrameDrift.changeFrame(nextContactFramePosition.getReferenceFrame());
               nextContactFramePosition.setZ(nextContactFramePosition.getZ() + fromContactFrameDrift.getZ());
            }

            nextContactFramePosition.changeFrame(worldFrame);
         }
      }

      contactFrameZeroPosition.set(transferFromAnklePosition);
      contactFrameOnePosition.set(transferToAnklePosition);

      tempPoint2dA.set(transferFromAnklePosition);
      tempPoint2dB.set(transferToAnklePosition);

      projectionSegment.set(tempPoint2dA, tempPoint2dB);
      setPointXValues(nextContactFramePosition);

      transferFromAnklePosition.changeFrame(frameOfLastFoostep);
      transferToAnklePosition.changeFrame(frameOfLastFoostep);

      double footHeight0 = transferFromAnklePosition.getZ();
      double footHeight1 = transferToAnklePosition.getZ();

      double nextFootHeight = Double.NaN;

      if (CONSIDER_NEXT_FOOTSTEP)
      {
         if (nextContactFramePosition != null)
         {
            nextContactFramePosition.changeFrame(frameOfLastFoostep);
            nextFootHeight = nextContactFramePosition.getZ();
         }
      }

      startWaypointMinimum.setY(footHeight0 + minimumHeightAboveGround.getDoubleValue());
      startWaypointNominal.setY(footHeight0 + nominalHeightAboveGround.getDoubleValue());
      startWaypointMaximum.setY(footHeight0 + maximumHeightAboveGround.getDoubleValue());

      endWaypointMinimum.setY(footHeight1 + minimumHeightAboveGround.getDoubleValue());
      endWaypointNominal.setY(footHeight1 + nominalHeightAboveGround.getDoubleValue());
      endWaypointMaximum.setY(footHeight1 + maximumHeightAboveGround.getDoubleValue());

      firstMidpointMinimum.setY(findMinimumDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), firstMidpointPosition.getX(), footHeight0, footHeight1));
      firstMidpointNominal.setY(findNominalDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), firstMidpointPosition.getX(), footHeight0, footHeight1));
      firstMidpointMaximum.setY(findMaximumDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), firstMidpointPosition.getX(), footHeight0, footHeight1));

      secondMidpointMinimum.setY(findMinimumDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), secondMidpointPosition.getX(), footHeight0, footHeight1));
      secondMidpointNominal.setY(findNominalDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), secondMidpointPosition.getX(), footHeight0, footHeight1));
      secondMidpointMaximum.setY(findMaximumDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), secondMidpointPosition.getX(), footHeight0 + extraToeOffHeight, footHeight1));

      sNextMin.setY(nextFootHeight + minimumHeightAboveGround.getDoubleValue());
      sNextNom.setY(nextFootHeight + nominalHeightAboveGround.getDoubleValue());
      sNextMax.setY(nextFootHeight + maximumHeightAboveGround.getDoubleValue());

      computeHeightsToUseByStretchingString(transferFromFootstep.getRobotSide());
      previousZFinals.get(transferToFootstep.getRobotSide()).set(endWaypoint.getY());

      spline.initialize(startWaypoint, firstMidpointPosition, secondMidpointPosition, endWaypoint);

      if (visualize)
      {
         framePointS0.setIncludingFrame(transferFromAnklePosition);
         framePointS0.setZ(startWaypoint.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointS0.changeFrame(worldFrame);
         pointS0Viz.setPosition(framePointS0);

         framePointS0.changeFrame(frameOfLastFoostep);
         framePointS0.setZ(startWaypointMinimum.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointS0.changeFrame(worldFrame);
         pointS0MinViz.setPosition(framePointS0);

         framePointS0.changeFrame(frameOfLastFoostep);
         framePointS0.setZ(startWaypointMaximum.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointS0.changeFrame(worldFrame);
         pointS0MaxViz.setPosition(framePointS0);

         framePointD0.setToZero(transferFromAnklePosition.getReferenceFrame());
         framePointD0.interpolate(transferFromAnklePosition, transferToAnklePosition, firstMidpointPosition.getX() / endWaypoint.getX());
         framePointD0.setZ(firstMidpointPosition.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointD0.changeFrame(worldFrame);
         pointD0Viz.setPosition(framePointD0);

         framePointD0.changeFrame(frameOfLastFoostep);
         framePointD0.setZ(firstMidpointMinimum.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointD0.changeFrame(worldFrame);
         pointD0MinViz.setPosition(framePointD0);

         framePointD0.changeFrame(frameOfLastFoostep);
         framePointD0.setZ(firstMidpointMaximum.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointD0.changeFrame(worldFrame);
         pointD0MaxViz.setPosition(framePointD0);

         framePointDF.setToZero(transferFromAnklePosition.getReferenceFrame());
         framePointDF.interpolate(transferFromAnklePosition, transferToAnklePosition, secondMidpointPosition.getX() / endWaypoint.getX());
         framePointDF.setZ(secondMidpointPosition.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointDF.changeFrame(worldFrame);
         pointDFViz.setPosition(framePointDF);

         framePointDF.changeFrame(frameOfLastFoostep);
         framePointDF.setZ(secondMidpointMinimum.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointDF.changeFrame(worldFrame);
         pointDFMinViz.setPosition(framePointDF);

         framePointDF.changeFrame(frameOfLastFoostep);
         framePointDF.setZ(secondMidpointMaximum.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointDF.changeFrame(worldFrame);
         pointDFMaxViz.setPosition(framePointDF);

         framePointSF.setIncludingFrame(transferToAnklePosition);
         framePointSF.setZ(endWaypoint.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointSF.changeFrame(worldFrame);
         pointSFViz.setPosition(framePointSF);

         framePointSF.changeFrame(frameOfLastFoostep);
         framePointSF.setZ(endWaypointMinimum.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointSF.changeFrame(worldFrame);
         pointSFMinViz.setPosition(framePointSF);

         framePointSF.changeFrame(frameOfLastFoostep);
         framePointSF.setZ(endWaypointMaximum.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
         framePointSF.changeFrame(worldFrame);
         pointSFMaxViz.setPosition(framePointSF);

         if (CONSIDER_NEXT_FOOTSTEP)
         {
            if (nextContactFramePosition != null)
            {
               framePointSNext.setIncludingFrame(nextContactFramePosition);
               framePointSNext.setZ(sNext.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
               framePointSNext.changeFrame(worldFrame);
               pointSNextViz.setPosition(framePointSNext);

               framePointSNext.changeFrame(frameOfLastFoostep);
               framePointSNext.setZ(sNextMin.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
               framePointSNext.changeFrame(worldFrame);
               pointSNextMinViz.setPosition(framePointSNext);

               framePointSNext.changeFrame(frameOfLastFoostep);
               framePointSNext.setZ(sNextMax.getY() + heightOffsetHandler.getOffsetHeightAboveGround());
               framePointSNext.changeFrame(worldFrame);
               pointSNextMaxViz.setPosition(framePointSNext);
            }
            else
            {
               pointSNextViz.setPositionToNaN();
               pointSNextMinViz.setPositionToNaN();
               pointSNextMaxViz.setPositionToNaN();
            }
         }
         else
         {
            pointSNextViz.setPositionToNaN();
            pointSNextMinViz.setPositionToNaN();
            pointSNextMaxViz.setPositionToNaN();
         }

         bagOfBalls.reset();
         int numberOfPoints = bagOfBalls.getNumberOfBalls();

         for (int i = 0; i < numberOfPoints; i++)
         {
            tempFramePointForViz1.setToZero(transferFromAnklePosition.getReferenceFrame());
            tempFramePointForViz1.interpolate(transferFromAnklePosition, transferToAnklePosition, ((double) i) / ((double) numberOfPoints));
            tempFramePointForViz1.changeFrame(worldFrame);
            queryPoint.set(tempFramePointForViz1);
            this.solve(coMHeightPartialDerivativesData, queryPoint, false);
            tempFramePointForViz2.setToZero(coMHeightPartialDerivativesData.getFrameOfCoMHeight());
            tempFramePointForViz2.setZ(coMHeightPartialDerivativesData.getComHeight());
            tempFramePointForViz2.setX(tempFramePointForViz1.getX());
            tempFramePointForViz2.setY(tempFramePointForViz1.getY());

            bagOfBalls.setBallLoop(tempFramePointForViz2);
         }
      }
   }


   private Point2D tempPoint2dForStringStretching = new Point2D();

   private final StringStretcher2d stringStretcher2d = new StringStretcher2d();
   private final List<Point2DBasics> stretchedStringWaypoints = new ArrayList<>();

   private void computeHeightsToUseByStretchingString(RobotSide transferFromSide)
   {
      stringStretcher2d.reset();
      // startWaypoint is at previous
      double z0 = MathTools.clamp(previousZFinals.get(transferFromSide).getDoubleValue(), startWaypointMinimum.getY(), startWaypointMaximum.getY());
      startWaypoint.setY(z0);

      stringStretcher2d.setStartPoint(startWaypoint);

      if (!Double.isNaN(sNext.getX()))
      {
         if (sNext.getX() < endWaypoint.getX() + 0.01)
         {
            sNext.setX(endWaypoint.getX() + 0.01);
         }

         double zNext;
         if (sNextMin.getY() > endWaypointNominal.getY())
         {
            zNext = sNextMin.getY();
         }
         else if (sNextMax.getY() < endWaypointNominal.getY())
         {
            zNext = sNextMax.getY();
         }
         else
         {
            zNext = endWaypointNominal.getY();
         }

         sNext.setY(zNext);

         stringStretcher2d.setEndPoint(sNext);

         stringStretcher2d.addMinMaxPoints(firstMidpointMinimum, firstMidpointMaximum);
         stringStretcher2d.addMinMaxPoints(secondMidpointMinimum, secondMidpointMaximum);
         stringStretcher2d.addMinMaxPoints(endWaypointMinimum, endWaypointMaximum);
      }
      else
      {
         double sFNomHeight = endWaypointNominal.getY();

         // If the final single support nominal position is higher than the final double support max position, decrease it
         // but don't decrease it below the final single support minimum height.
         if (sFNomHeight > secondMidpointMaximum.getY())
         {
            sFNomHeight = Math.max(secondMidpointMaximum.getY(), endWaypointMinimum.getY());
            tempPoint2dForStringStretching.set(endWaypointNominal);
            tempPoint2dForStringStretching.setY(sFNomHeight);
            stringStretcher2d.setEndPoint(tempPoint2dForStringStretching);
         }
         else
         {
            stringStretcher2d.setEndPoint(endWaypointNominal);
         }

         stringStretcher2d.addMinMaxPoints(firstMidpointMinimum, firstMidpointMaximum);
         stringStretcher2d.addMinMaxPoints(secondMidpointMinimum, secondMidpointMaximum);
      }

      stringStretcher2d.stretchString(stretchedStringWaypoints);

      firstMidpointPosition.set(stretchedStringWaypoints.get(1));
      secondMidpointPosition.set(stretchedStringWaypoints.get(2));
      endWaypoint.set(stretchedStringWaypoints.get(3));
   }

   private final Point2D nextPoint2d = new Point2D();
   private final Point2D projectedPoint = new Point2D();
   private final Line2D line2d = new Line2D();

   private void setPointXValues(FramePoint3D nextContactFramePosition)
   {
      double length = projectionSegment.length();

      double xS0 = 0.0;
      double xD0 = doubleSupportPercentageIn.getDoubleValue() * length;
      double xDF = (1.0 - doubleSupportPercentageIn.getDoubleValue()) * length;
      double xSF = length;

      double xSNext = Double.NaN;
      if (nextContactFramePosition != null)
      {
         //need to double check this
         line2d.set(projectionSegment.getFirstEndpoint(), projectionSegment.getSecondEndpoint());
         nextPoint2d.set(nextContactFramePosition.getX(), nextContactFramePosition.getY());
         line2d.orthogonalProjection(nextPoint2d, projectedPoint);
         xSNext = projectionSegment.percentageAlongLineSegment(projectedPoint) * projectionSegment.length();
      }

      startWaypoint.setX(xS0);
      firstMidpointPosition.setX(xD0);
      secondMidpointPosition.setX(xDF);
      endWaypoint.setX(xSF);
      sNext.setX(xSNext);

      startWaypointMinimum.setX(xS0);
      firstMidpointMinimum.setX(xD0);
      secondMidpointMinimum.setX(xDF);
      endWaypointMinimum.setX(xSF);
      sNextMin.setX(xSNext);

      startWaypointNominal.setX(xS0);
      firstMidpointNominal.setX(xD0);
      secondMidpointNominal.setX(xDF);
      endWaypointNominal.setX(xSF);
      sNextNom.setX(xSNext);

      startWaypointMaximum.setX(xS0);
      firstMidpointMaximum.setX(xD0);
      secondMidpointMaximum.setX(xDF);
      endWaypointMaximum.setX(xSF);
      sNextMax.setX(xSNext);
   }

   private double findMinimumDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      return findDoubleSupportHeight(minimumHeightAboveGround.getDoubleValue(), s0, sF, s_d0, foot0Height, foot1Height);
   }

   private double findNominalDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      return findDoubleSupportHeight(nominalHeightAboveGround.getDoubleValue(), s0, sF, s_d0, foot0Height, foot1Height);
   }

   private double findMaximumDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
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

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final Point2D solutionPoint = new Point2D();

   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, boolean isInDoubleSupport)
   {
      getCenterOfMass2d(solutionPoint, centerOfMassFrame);
      solve(coMHeightPartialDerivativesDataToPack, solutionPoint, isInDoubleSupport);

      desiredCoMPosition.set(solutionPoint.getX(), solutionPoint.getY(), coMHeightPartialDerivativesDataToPack.getComHeight());
   }

   private final FramePoint3D height = new FramePoint3D();
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final double[] partialDerivativesWithRespectToS = new double[2];

   private void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, Point2DBasics queryPoint, boolean isInDoubleSupport)
   {
      projectionSegment.orthogonalProjection(queryPoint);
      double splineQuery = projectionSegment.percentageAlongLineSegment(queryPoint) * projectionSegment.length();

      // Happens when the robot gets stuck in double support but the ICP is still being dragged in the front support foot.
      if (isInDoubleSupport)
         splineQuery = Math.min(splineQuery, secondMidpointPosition.getX());

      spline.compute(splineQuery);

      handleInitializeToCurrent();

      heightOffsetHandler.update(spline.getY());

      double z = spline.getY() + heightOffsetHandler.getOffsetHeightAboveGroundTrajectoryOutput();
      double dzds = spline.getYDot();
      double ddzdds = spline.getYDDot();

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

   private void handleInitializeToCurrent()
   {
      if (initializeToCurrent.getBooleanValue())
      {
         initializeToCurrent.set(false);

         desiredPosition.setToZero(pelvisFrame);
         desiredPosition.changeFrame(frameOfLastFoostep);

         double heightOffset = desiredPosition.getZ() - spline.getY();

         heightOffsetHandler.initializeToCurrent(heightOffset);
      }
   }

   private final PelvisHeightTrajectoryCommand tempPelvisHeightTrajectoryCommand = new PelvisHeightTrajectoryCommand();

   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();

      if (!se3Trajectory.getSelectionMatrix().isLinearZSelected())
         return false; // The user does not want to control the height, do nothing.

      se3Trajectory.changeFrame(worldFrame);
      tempPelvisHeightTrajectoryCommand.set(command);
      handlePelvisHeightTrajectoryCommand(tempPelvisHeightTrajectoryCommand);
      return true;
   }

   public boolean handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      return heightOffsetHandler.handlePelvisHeightTrajectoryCommand(command, spline.getY());
   }

   public void goHome(double trajectoryTime)
   {
      if (!processGoHome.getValue())
         return;

      heightOffsetHandler.goHome(trajectoryTime);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      heightOffsetHandler.handleStopAllTrajectoryCommand(command);
   }

   public void initializeDesiredHeightToCurrent()
   {
      initializeToCurrent.set(true);
   }

   private void getPartialDerivativesWithRespectToS(LineSegment2D segment, double[] partialDerivativesToPack)
   {
      double dsdx = (segment.getSecondEndpointX() - segment.getFirstEndpointX()) / segment.length();
      double dsdy = (segment.getSecondEndpointY() - segment.getFirstEndpointY()) / segment.length();

      partialDerivativesToPack[0] = dsdx;
      partialDerivativesToPack[1] = dsdy;
   }

   private final FramePoint3D coM = new FramePoint3D();

   private void getCenterOfMass2d(Point2D point2dToPack, ReferenceFrame centerOfMassFrame)
   {
      coM.setToZero(centerOfMassFrame);
      coM.changeFrame(worldFrame);

      point2dToPack.set(coM);
   }

   public boolean hasBeenInitializedWithNextStep()
   {
      return hasBeenInitializedWithNextStep.getBooleanValue();
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
      FramePoint3D position = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(position, orientation);
      position.changeFrame(worldFrame);
      orientation.changeFrame(worldFrame);

      System.out.println("footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide." + robotSide + ", new Point3D(" + position.getX() + ", "
            + position.getY() + ", " + position.getZ() + "), new Quat4d(" + orientation.getS() + ", " + orientation.getX() + ", " + orientation.getY() + ", "
            + orientation.getZ() + ")));");
   }

   public void getCurrentDesiredHeight(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(desiredCoMPosition);
   }

   public double getOffsetHeightTimeInTrajectory()
   {
      return yoTime.getValue() - heightOffsetHandler.getOffsetHeightAboveGroundChangedTime();
   }
}
