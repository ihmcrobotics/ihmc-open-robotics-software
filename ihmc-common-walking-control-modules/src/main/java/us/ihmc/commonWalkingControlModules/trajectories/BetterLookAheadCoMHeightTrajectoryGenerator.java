package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.StringStretcher2d;
import us.ihmc.robotics.geometry.StringStretcher3D;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.math.trajectories.YoSpline3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

/**
 * TODO There is not enough HumanoidReferenceFrames in that class, it is pretty fragile
 */
public class BetterLookAheadCoMHeightTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean DEBUG = false;

   private boolean visualize = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoPolynomial spline = new YoPolynomial("height", 4 , registry);

   private final YoBoolean isTrajectoryOffsetStopped = new YoBoolean("isPelvisOffsetHeightTrajectoryStopped", registry);

   private final YoDouble offsetHeightAboveGround = new YoDouble("offsetHeightAboveGround", registry);

   private final YoDouble offsetHeightAboveGroundPrevValue = new YoDouble("offsetHeightAboveGroundPrevValue", registry);
   private final YoDouble offsetHeightAboveGroundChangedTime = new YoDouble("offsetHeightAboveGroundChangedTime", registry);
   private final YoVariableDoubleProvider offsetHeightAboveGroundTrajectoryOutput = new YoVariableDoubleProvider("offsetHeightAboveGroundTrajectoryOutput",
                                                                                                                 registry);
   private final YoVariableDoubleProvider offsetHeightAboveGroundTrajectoryTimeProvider = new YoVariableDoubleProvider("offsetHeightAboveGroundTrajectoryTimeProvider",
                                                                                                                       registry);
   private final MultipleWaypointsTrajectoryGenerator offsetHeightTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator("pelvisHeightOffset",
                                                                                                                                 registry);

   private final YoDouble minimumHeightAboveGround = new YoDouble("minimumHeightAboveGround", registry);
   private final YoDouble nominalHeightAboveGround = new YoDouble("nominalHeightAboveGround", registry);
   private final YoDouble maximumHeightAboveGround = new YoDouble("maximumHeightAboveGround", registry);

   private final YoDouble doubleSupportPercentageIn = new YoDouble("doubleSupportPercentageIn", registry);

   private final YoDouble previousZFinalLeft = new YoDouble("previousZFinalLeft", registry);
   private final YoDouble previousZFinalRight = new YoDouble("previousZFinalRight", registry);
   private final SideDependentList<YoDouble> previousZFinals = new SideDependentList<YoDouble>(previousZFinalLeft, previousZFinalRight);

   private final YoDouble desiredCoMHeight = new YoDouble("desiredCoMHeight", registry);
   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);

   private final LineSegment2D projectionSegmentInWorld = new LineSegment2D();

   private final YoFramePoint3D contactFrameZeroPosition = new YoFramePoint3D("contactFrameZeroPosition", worldFrame, registry);
   private final YoFramePoint3D contactFrameOnePosition = new YoFramePoint3D("contactFrameOnePosition", worldFrame, registry);

   private final YoGraphicPosition pointS0Viz, pointSFViz, pointD0Viz, pointDFViz;
   private final YoGraphicPosition pointS0MinViz, pointSFMinViz, pointD0MinViz, pointDFMinViz;
   private final YoGraphicPosition pointS0MaxViz, pointSFMaxViz, pointD0MaxViz, pointDFMaxViz;

   private final YoBoolean initializeToCurrent = new YoBoolean("initializeCoMHeightToCurrent", registry);

   private final BagOfBalls bagOfBalls;

   private final DoubleProvider yoTime;

   private ReferenceFrame frameOfLastFoostep;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final YoLong lastCommandId;

   private final YoBoolean isReadyToHandleQueuedCommands;
   private final YoLong numberOfQueuedCommands;
   private final RecyclingArrayDeque<PelvisHeightTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(PelvisHeightTrajectoryCommand.class,
                                                                                                             PelvisHeightTrajectoryCommand::set);

   private final BooleanProvider processGoHome = new BooleanParameter("ProcessGoHome", registry, false);

   public BetterLookAheadCoMHeightTrajectoryGenerator(double minimumHeightAboveGround, double nominalHeightAboveGround, double maximumHeightAboveGround,
                                                      double defaultOffsetHeightAboveGround, double doubleSupportPercentageIn, ReferenceFrame centerOfMassFrame,
                                                      ReferenceFrame pelvisFrame,
                                                      SideDependentList<MovingReferenceFrame> soleFrames, DoubleProvider yoTime,
                                                      YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.pelvisFrame = pelvisFrame;
      this.centerOfMassFrame = centerOfMassFrame;
      this.soleFrames = soleFrames;
      frameOfLastFoostep = soleFrames.get(RobotSide.LEFT);
      this.yoTime = yoTime;
      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightAboveGroundTrajectoryTimeProvider.set(0.5);
      offsetHeightAboveGround.set(defaultOffsetHeightAboveGround);
      offsetHeightAboveGroundPrevValue.set(defaultOffsetHeightAboveGround);
      offsetHeightAboveGround.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
            double previous = offsetHeightTrajectoryGenerator.getValue();
            offsetHeightTrajectoryGenerator.clear();
            offsetHeightTrajectoryGenerator.appendWaypoint(0.0, previous, 0.0);
            offsetHeightTrajectoryGenerator.appendWaypoint(offsetHeightAboveGroundTrajectoryTimeProvider.getValue(),
                                                           offsetHeightAboveGround.getDoubleValue(),
                                                           0.0);
            offsetHeightTrajectoryGenerator.initialize();
         }
      });

      setMinimumHeightAboveGround(minimumHeightAboveGround);
      setNominalHeightAboveGround(nominalHeightAboveGround);
      setMaximumHeightAboveGround(maximumHeightAboveGround);
      previousZFinalLeft.set(nominalHeightAboveGround);
      previousZFinalRight.set(nominalHeightAboveGround);

      this.doubleSupportPercentageIn.set(doubleSupportPercentageIn);

      String namePrefix = "pelvisHeight";
      lastCommandId = new YoLong(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new YoBoolean(namePrefix + "IsReadyToHandleQueuedPelvisHeightTrajectoryCommands", registry);
      numberOfQueuedCommands = new YoLong(namePrefix + "NumberOfQueuedCommands", registry);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry == null)
         visualize = false;

      if (visualize)
      {
         double pointSize = 0.03;

         String prefix = "better_";

         YoGraphicPosition position0 = new YoGraphicPosition(prefix + "contactFrame0", contactFrameZeroPosition, pointSize, YoAppearance.Purple());
         YoGraphicPosition position1 = new YoGraphicPosition(prefix + "contactFrame1", contactFrameOnePosition, pointSize, YoAppearance.Orange());

         pointS0Viz = new YoGraphicPosition(prefix + "pointS0", "", registry, pointSize, YoAppearance.CadetBlue());
         pointSFViz = new YoGraphicPosition(prefix + "pointSF", "", registry, pointSize, YoAppearance.Chartreuse());
         pointD0Viz = new YoGraphicPosition(prefix + "pointD0", "", registry, pointSize, YoAppearance.BlueViolet());
         pointDFViz = new YoGraphicPosition(prefix + "pointDF", "", registry, pointSize, YoAppearance.Azure());

         pointS0MinViz = new YoGraphicPosition(prefix + "pointS0Min", "", registry, 0.8 * pointSize, YoAppearance.CadetBlue());
         pointSFMinViz = new YoGraphicPosition(prefix + "pointSFMin", "", registry, 0.8 * pointSize, YoAppearance.Chartreuse());
         pointD0MinViz = new YoGraphicPosition(prefix + "pointD0Min", "", registry, 0.8 * pointSize, YoAppearance.BlueViolet());
         pointDFMinViz = new YoGraphicPosition(prefix + "pointDFMin", "", registry, 0.8 * pointSize, YoAppearance.Azure());

         pointS0MaxViz = new YoGraphicPosition(prefix + "pointS0Max", "", registry, 0.9 * pointSize, YoAppearance.CadetBlue());
         pointSFMaxViz = new YoGraphicPosition(prefix + "pointSFMax", "", registry, 0.9 * pointSize, YoAppearance.Chartreuse());
         pointD0MaxViz = new YoGraphicPosition(prefix + "pointD0Max", "", registry, 0.9 * pointSize, YoAppearance.BlueViolet());
         pointDFMaxViz = new YoGraphicPosition(prefix + "pointDFMax", "", registry, 0.9 * pointSize, YoAppearance.Azure());

         bagOfBalls = new BagOfBalls(15, 0.01, "height", registry, yoGraphicsListRegistry);

         YoGraphicPosition desiredCoMPositionViz = new YoGraphicPosition(prefix + "desiredCoMPosition", desiredCoMPosition, 1.1 * pointSize, YoAppearance.Gold());

         String graphicListName = "BetterCoMHeightTrajectoryGenerator";
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, position0);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, position1);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0Viz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0Viz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFViz);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0MinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFMinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFMinViz);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0MaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFMaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFMaxViz);
//
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, desiredCoMPositionViz);
      }
      else
      {
         pointS0Viz = null;
         pointSFViz = null;
         pointD0Viz = null;
         pointDFViz = null;

         pointS0MinViz = null;
         pointSFMinViz = null;
         pointD0MinViz = null;
         pointDFMinViz = null;

         pointS0MaxViz = null;
         pointSFMaxViz = null;
         pointD0MaxViz = null;
         pointDFMaxViz = null;

         bagOfBalls = null;
      }
   }

   public void reset()
   {
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);
      isReadyToHandleQueuedCommands.set(false);
      numberOfQueuedCommands.set(0);

      offsetHeightAboveGround.set(0.0, false);
      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightTrajectoryGenerator.clear();
      offsetHeightTrajectoryGenerator.appendWaypoint(0.0, 0.0, 0.0);
      offsetHeightTrajectoryGenerator.initialize();
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

   public void setSupportLeg(RobotSide supportLeg)
   {
      ReferenceFrame newFrame = soleFrames.get(supportLeg);

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, startWaypoint.getZ());
      tempFramePoint.changeFrame(newFrame);
      startWaypoint.setZ(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, firstMidpoint.getZ());
      tempFramePoint.changeFrame(newFrame);
      firstMidpoint.setZ(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, secondMidpoint.getZ());
      tempFramePoint.changeFrame(newFrame);
      secondMidpoint.setZ(tempFramePoint.getZ());

      tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, endWaypoint.getZ());
      tempFramePoint.changeFrame(newFrame);
      endWaypoint.setZ(tempFramePoint.getZ());

      spline.setCubicUsingIntermediatePoints(startWaypoint.getX(), firstMidpoint.getX(), secondMidpoint.getX(), endWaypoint.getX(),
                                             startWaypoint.getZ(), firstMidpoint.getZ(), secondMidpoint.getZ(), endWaypoint.getZ());

      for (RobotSide robotSide : RobotSide.values)
      {
         YoDouble previousZFinal = previousZFinals.get(robotSide);
         tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, previousZFinal.getDoubleValue());
         tempFramePoint.changeFrame(newFrame);
         previousZFinal.set(tempFramePoint.getZ());
      }

      frameOfLastFoostep = newFrame;
   }

   private final FramePoint3D startWaypointMinimum = new FramePoint3D();
   private final FramePoint3D firstMidpointMinimum = new FramePoint3D();
   private final FramePoint3D secondMidpointMinimum = new FramePoint3D();
   private final FramePoint3D endWaypointMinimum = new FramePoint3D();

   private final FramePoint3D startWaypointNominal = new FramePoint3D();
   private final FramePoint3D firstMidpointNominal = new FramePoint3D();
   private final FramePoint3D secondMidpointNominal = new FramePoint3D();
   private final FramePoint3D endWaypointNominal = new FramePoint3D();

   private final FramePoint3D startWaypointMaximum = new FramePoint3D();
   private final FramePoint3D firstMidpointMaximum = new FramePoint3D();
   private final FramePoint3D secondMidpointMaximum = new FramePoint3D();
   private final FramePoint3D endWaypointMaximum = new FramePoint3D();

   private final FramePoint3D startWaypoint = new FramePoint3D();
   private final FramePoint3D firstMidpoint = new FramePoint3D();
   private final FramePoint3D secondMidpoint = new FramePoint3D();
   private final FramePoint3D endWaypoint = new FramePoint3D();

   private final FramePoint3D framePointS0 = new FramePoint3D();
   private final FramePoint3D framePointD0 = new FramePoint3D();
   private final FramePoint3D framePointDF = new FramePoint3D();
   private final FramePoint3D framePointSF = new FramePoint3D();

   private final FramePoint3D tempFramePointForViz1 = new FramePoint3D();
   private final FramePoint3D tempFramePointForViz2 = new FramePoint3D();

   private final FramePoint3D transferFromSolePosition = new FramePoint3D();
   private final FramePoint3D transferToSolePosition = new FramePoint3D();

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

      transferFromFootstep.getPosition(transferFromSolePosition);
      transferToFootstep.getPosition(transferToSolePosition);

      transferFromSolePosition.changeFrame(worldFrame);
      transferToSolePosition.changeFrame(worldFrame);

      contactFrameZeroPosition.setMatchingFrame(transferFromSolePosition);
      contactFrameOnePosition.setMatchingFrame(transferToSolePosition);

      projectionSegmentInWorld.set(transferFromSolePosition, transferToSolePosition);

      transferFromSolePosition.changeFrame(frameOfLastFoostep);
      transferToSolePosition.changeFrame(frameOfLastFoostep);

      zeroFrames();

      double length = projectionSegmentInWorld.length();

      double xS0 = 0.0;
      double xD0 = doubleSupportPercentageIn.getDoubleValue() * length;
      double xDF = (1.0 - doubleSupportPercentageIn.getDoubleValue()) * length;
      double xSF = length;

      startWaypoint.setX(xS0);
      firstMidpoint.setX(xD0);
      secondMidpoint.setX(xDF);
      endWaypoint.setX(xSF);

      startWaypointMinimum.setX(xS0);
      firstMidpointMinimum.setX(xD0);
      secondMidpointMinimum.setX(xDF);
      endWaypointMinimum.setX(xSF);

      startWaypointNominal.setX(xS0);
      firstMidpointNominal.setX(xD0);
      secondMidpointNominal.setX(xDF);
      endWaypointNominal.setX(xSF);

      startWaypointMaximum.setX(xS0);
      firstMidpointMaximum.setX(xD0);
      secondMidpointMaximum.setX(xDF);
      endWaypointMaximum.setX(xSF);

      // we always start at 0.0
      startWaypointMinimum.setZ(minimumHeightAboveGround.getDoubleValue());
      startWaypointNominal.setZ(nominalHeightAboveGround.getDoubleValue());
      startWaypointMaximum.setZ(maximumHeightAboveGround.getDoubleValue());

      endWaypointMinimum.setZ(minimumHeightAboveGround.getDoubleValue());
      endWaypointNominal.setZ(nominalHeightAboveGround.getDoubleValue());
      endWaypointMaximum.setZ(maximumHeightAboveGround.getDoubleValue());

      firstMidpointMinimum.setZ(findMinimumDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), firstMidpoint.getX(), 0.0));
      firstMidpointNominal.setZ(findNominalDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), firstMidpoint.getX(), 0.0));
      firstMidpointMaximum.setZ(findMaximumDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), firstMidpoint.getX(), 0.0));

      secondMidpointMinimum.setZ(findMinimumDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), secondMidpoint.getX(), 0.0));
      secondMidpointNominal.setZ(findNominalDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), secondMidpoint.getX(), 0.0));
      secondMidpointMaximum.setZ(findMaximumDoubleSupportHeight(startWaypoint.getX(), endWaypoint.getX(), secondMidpoint.getX(), extraToeOffHeight));

      startWaypointMinimum.setY(0.5 * transferToSolePosition.getY());
      startWaypointNominal.setY(0.5 * transferToSolePosition.getY());
      startWaypointMaximum.setY(0.5 * transferToSolePosition.getY());

      endWaypointMinimum.setY(0.5 * transferToSolePosition.getY());
      endWaypointNominal.setY(0.5 * transferToSolePosition.getY());
      endWaypointMaximum.setY(0.5 * transferToSolePosition.getY());

      firstMidpointMinimum.setY(0.5 * transferToSolePosition.getY());
      firstMidpointNominal.setY(0.5 * transferToSolePosition.getY());
      firstMidpointMaximum.setY(0.5 * transferToSolePosition.getY());

      secondMidpointMinimum.setY(0.5 * transferToSolePosition.getY());
      secondMidpointNominal.setY(0.5 * transferToSolePosition.getY());
      secondMidpointMaximum.setY(0.5 * transferToSolePosition.getY());

      computeHeightsToUseByStretchingString(transferFromFootstep.getRobotSide());
      previousZFinals.get(transferToFootstep.getRobotSide()).set(endWaypoint.getY());

      spline.setCubicUsingIntermediatePoints(startWaypoint.getX(), firstMidpoint.getX(), secondMidpoint.getX(), endWaypoint.getX(),
                                             startWaypoint.getZ(), firstMidpoint.getZ(), secondMidpoint.getZ(), endWaypoint.getZ());

      if (visualize)
      {
         framePointS0.setIncludingFrame(transferFromSolePosition);
         framePointS0.setZ(startWaypoint.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointS0.changeFrame(worldFrame);
         pointS0Viz.setPosition(framePointS0);

         framePointS0.changeFrame(frameOfLastFoostep);
         framePointS0.setZ(startWaypointMinimum.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointS0.changeFrame(worldFrame);
         pointS0MinViz.setPosition(framePointS0);

         framePointS0.changeFrame(frameOfLastFoostep);
         framePointS0.setZ(startWaypointMaximum.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointS0.changeFrame(worldFrame);
         pointS0MaxViz.setPosition(framePointS0);

         framePointD0.setToZero(transferFromSolePosition.getReferenceFrame());
         framePointD0.interpolate(transferFromSolePosition, transferToSolePosition, firstMidpoint.getX() / endWaypoint.getX());
         framePointD0.setZ(firstMidpoint.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointD0.changeFrame(worldFrame);
         pointD0Viz.setPosition(framePointD0);

         framePointD0.changeFrame(frameOfLastFoostep);
         framePointD0.setZ(firstMidpointMinimum.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointD0.changeFrame(worldFrame);
         pointD0MinViz.setPosition(framePointD0);

         framePointD0.changeFrame(frameOfLastFoostep);
         framePointD0.setZ(firstMidpointMaximum.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointD0.changeFrame(worldFrame);
         pointD0MaxViz.setPosition(framePointD0);

         framePointDF.setToZero(transferFromSolePosition.getReferenceFrame());
         framePointDF.interpolate(transferFromSolePosition, transferToSolePosition, secondMidpoint.getX() / endWaypoint.getX());
         framePointDF.setZ(secondMidpoint.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointDF.changeFrame(worldFrame);
         pointDFViz.setPosition(framePointDF);

         framePointDF.changeFrame(frameOfLastFoostep);
         framePointDF.setZ(secondMidpointMinimum.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointDF.changeFrame(worldFrame);
         pointDFMinViz.setPosition(framePointDF);

         framePointDF.changeFrame(frameOfLastFoostep);
         framePointDF.setZ(secondMidpointMaximum.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointDF.changeFrame(worldFrame);
         pointDFMaxViz.setPosition(framePointDF);

         framePointSF.setIncludingFrame(transferToSolePosition);
         framePointSF.setZ(endWaypoint.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointSF.changeFrame(worldFrame);
         pointSFViz.setPosition(framePointSF);

         framePointSF.changeFrame(frameOfLastFoostep);
         framePointSF.setZ(endWaypointMinimum.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointSF.changeFrame(worldFrame);
         pointSFMinViz.setPosition(framePointSF);

         framePointSF.changeFrame(frameOfLastFoostep);
         framePointSF.setZ(endWaypointMaximum.getY() + offsetHeightAboveGround.getDoubleValue());
         framePointSF.changeFrame(worldFrame);
         pointSFMaxViz.setPosition(framePointSF);

         bagOfBalls.reset();
         int numberOfPoints = bagOfBalls.getNumberOfBalls();

         for (int i = 0; i < numberOfPoints; i++)
         {
            tempFramePointForViz1.setToZero(transferFromSolePosition.getReferenceFrame());
            tempFramePointForViz1.interpolate(transferFromSolePosition, transferToSolePosition, ((double) i) / ((double) numberOfPoints));
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

   private void zeroFrames()
   {
      startWaypoint.setToZero(frameOfLastFoostep);
      firstMidpoint.setToZero(frameOfLastFoostep);
      secondMidpoint.setToZero(frameOfLastFoostep);
      endWaypoint.setToZero(frameOfLastFoostep);

      startWaypointMinimum.setToZero(frameOfLastFoostep);
      firstMidpointMinimum.setToZero(frameOfLastFoostep);
      secondMidpointMinimum.setToZero(frameOfLastFoostep);
      endWaypointMinimum.setToZero(frameOfLastFoostep);

      startWaypointNominal.setToZero(frameOfLastFoostep);
      firstMidpointNominal.setToZero(frameOfLastFoostep);
      secondMidpointNominal.setToZero(frameOfLastFoostep);
      endWaypointNominal.setToZero(frameOfLastFoostep);

      startWaypointMaximum.setToZero(frameOfLastFoostep);
      firstMidpointMaximum.setToZero(frameOfLastFoostep);
      secondMidpointMaximum.setToZero(frameOfLastFoostep);
      endWaypointMaximum.setToZero(frameOfLastFoostep);
   }

   private void setPointViz()
   {

   }


   private Point3D tempPoint2dForStringStretching = new Point3D();

   private final StringStretcher3D stringStretcher = new StringStretcher3D();
   private final List<Point3DBasics> stretchedStringWaypoints = new ArrayList<>();

   private void computeHeightsToUseByStretchingString(RobotSide transferFromSide)
   {
      stringStretcher.reset();
      // startWaypoint is at previous
      double z0 = MathTools.clamp(previousZFinals.get(transferFromSide).getDoubleValue(), startWaypointMinimum.getZ(), startWaypointMaximum.getZ());
      startWaypoint.setZ(z0);

      stringStretcher.setStartPoint(startWaypoint);

         double sFNomHeight = endWaypointNominal.getZ();

         // If the final single support nominal position is higher than the final double support max position, decrease it
         // but don't decrease it below the final single support minimum height.
         if (sFNomHeight > secondMidpointMaximum.getZ())
         {
            sFNomHeight = Math.max(secondMidpointMaximum.getZ(), endWaypointMinimum.getZ());
            tempPoint2dForStringStretching.set(endWaypointNominal);
            tempPoint2dForStringStretching.setZ(sFNomHeight);
            stringStretcher.setEndPoint(tempPoint2dForStringStretching);
         }
         else
         {
            stringStretcher.setEndPoint(endWaypointNominal);
         }

         stringStretcher.addMinMaxPoints(firstMidpointMinimum, firstMidpointMaximum);
         stringStretcher.addMinMaxPoints(secondMidpointMinimum, secondMidpointMaximum);

      stringStretcher.stretchString(stretchedStringWaypoints);

      firstMidpoint.set(stretchedStringWaypoints.get(1));
      secondMidpoint.set(stretchedStringWaypoints.get(2));
      endWaypoint.set(stretchedStringWaypoints.get(3));
   }

   private double findMinimumDoubleSupportHeight(double startXRelativeToAnkle, double endXRelativeToAnkle, double queryRelativeToAnkle, double extraToeOffHeight)
   {
      return findDoubleSupportHeight(minimumHeightAboveGround.getDoubleValue(), startXRelativeToAnkle, endXRelativeToAnkle, queryRelativeToAnkle, extraToeOffHeight);
   }

   private double findNominalDoubleSupportHeight(double startXRelativeToAnkle, double endXRelativeToAnkle, double queryRelativeToAnkle, double extraToeOffHeight)
   {
      return findDoubleSupportHeight(nominalHeightAboveGround.getDoubleValue(), startXRelativeToAnkle, endXRelativeToAnkle, queryRelativeToAnkle, extraToeOffHeight);
   }

   private double findMaximumDoubleSupportHeight(double startXRelativeToAnkle, double endXRelativeToAnkle, double queryRelativeToAnkle, double extraToeOffHeight)
   {
      return findDoubleSupportHeight(maximumHeightAboveGround.getDoubleValue(), startXRelativeToAnkle, endXRelativeToAnkle, queryRelativeToAnkle, extraToeOffHeight);
   }

   private double findDoubleSupportHeight(double desiredDistanceFromFoot, double startXRelativeToAnkle, double endXRelativeToAnkle, double queryRelativeToAnkle, double extraToeOffHeight)
   {
      double z_d0_A = extraToeOffHeight + Math.sqrt(MathTools.square(desiredDistanceFromFoot) - MathTools.square(queryRelativeToAnkle - startXRelativeToAnkle));
      double z_d0_B = Math.sqrt(MathTools.square(desiredDistanceFromFoot) - MathTools.square((endXRelativeToAnkle - queryRelativeToAnkle)));

      return Math.min(z_d0_A, z_d0_B);
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
      // TODO this is not how we want to do this, we want to be using the actual center of mass point
      projectionSegmentInWorld.orthogonalProjection(queryPoint);
      double splineQuery = projectionSegmentInWorld.percentageAlongLineSegment(queryPoint) * projectionSegmentInWorld.length();

      // Happens when the robot gets stuck in double support but the ICP is still being dragged in the front support foot.
      if (isInDoubleSupport)
         splineQuery = Math.min(splineQuery, secondMidpoint.getX());

      spline.compute(splineQuery);

      handleInitializeToCurrent();

      if (!isTrajectoryOffsetStopped.getBooleanValue())
      {
         double deltaTime = yoTime.getValue() - offsetHeightAboveGroundChangedTime.getDoubleValue();

         if (!offsetHeightTrajectoryGenerator.isEmpty())
         {
            offsetHeightTrajectoryGenerator.compute(deltaTime);
         }

         if (offsetHeightTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
         {
            double firstTrajectoryPointTime = offsetHeightTrajectoryGenerator.getLastWaypointTime();
            PelvisHeightTrajectoryCommand command = commandQueue.poll();
            numberOfQueuedCommands.decrement();
            initializeOffsetTrajectoryGenerator(command, firstTrajectoryPointTime);
            offsetHeightTrajectoryGenerator.compute(deltaTime);
         }

         offsetHeightAboveGround.set(offsetHeightTrajectoryGenerator.getValue(), false);
      }
      offsetHeightAboveGroundTrajectoryOutput.set(offsetHeightTrajectoryGenerator.getValue());

      offsetHeightAboveGroundPrevValue.set(offsetHeightTrajectoryGenerator.getValue());

      double z = spline.getPosition() + offsetHeightAboveGroundTrajectoryOutput.getValue();
      double dzds = spline.getVelocity();
      double ddzdds = spline.getAcceleration();

      getPartialDerivativesWithRespectToS(projectionSegmentInWorld, partialDerivativesWithRespectToS);
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

         double heightOffset = desiredPosition.getZ() - spline.getPosition();

         offsetHeightAboveGround.set(heightOffset);
         offsetHeightAboveGroundChangedTime.set(yoTime.getValue());

         offsetHeightTrajectoryGenerator.clear();
         offsetHeightTrajectoryGenerator.appendWaypoint(0.0, heightOffset, 0.0);
         offsetHeightTrajectoryGenerator.initialize();
         isTrajectoryOffsetStopped.set(false);
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
      EuclideanTrajectoryControllerCommand euclideanTrajectory = command.getEuclideanTrajectory();

      if (euclideanTrajectory.getExecutionMode() == ExecutionMode.OVERRIDE)
      {
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(euclideanTrajectory.getCommandId());
         offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
         initializeOffsetTrajectoryGenerator(command, 0.0);
         return true;
      }
      else if (euclideanTrajectory.getExecutionMode() == ExecutionMode.QUEUE)
      {
         boolean success = queuePelvisHeightTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            offsetHeightTrajectoryGenerator.clear();
            offsetHeightTrajectoryGenerator.appendWaypoint(0.0, offsetHeightAboveGroundPrevValue.getDoubleValue(), 0.0);
            offsetHeightTrajectoryGenerator.initialize();
         }
         return success;
      }
      else if (euclideanTrajectory.getExecutionMode() == ExecutionMode.STREAM)
      {
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(euclideanTrajectory.getCommandId());
         offsetHeightAboveGroundChangedTime.set(yoTime.getValue());

         if (euclideanTrajectory.getNumberOfTrajectoryPoints() != 1)
         {
            LogTools.warn("When streaming, trajectories should contain only 1 trajectory point, was: " + euclideanTrajectory.getNumberOfTrajectoryPoints());
            return false;
         }

         FrameEuclideanTrajectoryPoint trajectoryPoint = euclideanTrajectory.getTrajectoryPoint(0);

         if (trajectoryPoint.getTime() != 0.0)
         {
            LogTools.warn("When streaming, the trajectory point should have a time of zero, was: " + trajectoryPoint.getTime());
            return false;
         }

         offsetHeightTrajectoryGenerator.clear();

         double time = trajectoryPoint.getTime();
         double z = fromAbsoluteToOffset(trajectoryPoint.getPositionZ());
         double zDot = trajectoryPoint.getLinearVelocityZ();
         offsetHeightTrajectoryGenerator.appendWaypoint(time, z, zDot);

         time = euclideanTrajectory.getStreamIntegrationDuration();
         z += time * zDot;
         offsetHeightTrajectoryGenerator.appendWaypoint(time, z, zDot);

         offsetHeightTrajectoryGenerator.initialize();
         isTrajectoryOffsetStopped.set(false);
         return true;
      }
      else
      {
         LogTools.warn("Unknown {} value: {}. Command ignored.", ExecutionMode.class.getSimpleName(), euclideanTrajectory.getExecutionMode());
         return false;
      }
   }

   private boolean queuePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      if (!isReadyToHandleQueuedCommands.getBooleanValue())
      {
         LogTools.warn("The very first {} of a series must be {}. Aborting motion.", command.getClass().getSimpleName(), ExecutionMode.OVERRIDE);
         return false;
      }

      long previousCommandId = command.getEuclideanTrajectory().getPreviousCommandId();

      if (previousCommandId != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != previousCommandId)
      {
         LogTools.warn("Previous command ID mismatch: previous ID from command = {}, last message ID received by the controller = {}. Aborting motion.",
                       previousCommandId,
                       lastCommandId.getLongValue());
         return false;
      }

      if (command.getEuclideanTrajectory().getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         LogTools.warn("Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(command.getEuclideanTrajectory().getCommandId());

      return true;
   }

   private void initializeOffsetTrajectoryGenerator(PelvisHeightTrajectoryCommand command, double firstTrajectoryPointTime)
   {
      command.getEuclideanTrajectory().addTimeOffset(firstTrajectoryPointTime);

      offsetHeightTrajectoryGenerator.clear();

      if (command.getEuclideanTrajectory().getTrajectoryPoint(0).getTime() > firstTrajectoryPointTime + 1.0e-5)
      {
         offsetHeightTrajectoryGenerator.appendWaypoint(0.0, offsetHeightAboveGroundPrevValue.getDoubleValue(), 0.0);
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         appendTrajectoryPoint(command.getEuclideanTrajectory().getTrajectoryPoint(trajectoryPointIndex));
      }

      offsetHeightTrajectoryGenerator.initialize();
      isTrajectoryOffsetStopped.set(false);
   }

   /**
    * Appends the given trajectory point to the trajectory generator. This method handles the
    * conversion of absolute coordinate to offset.
    */
   private void appendTrajectoryPoint(FrameEuclideanTrajectoryPoint trajectoryPoint)
   {
      double time = trajectoryPoint.getTime();
      double z = fromAbsoluteToOffset(trajectoryPoint.getPositionZ());
      double zDot = trajectoryPoint.getLinearVelocityZ();
      offsetHeightTrajectoryGenerator.appendWaypoint(time, z, zDot);
   }

   private double fromAbsoluteToOffset(double zInWorld)
   {
      // TODO (Sylvain) Check if that's the right way to do it
      // FIXME the given coordinate is always assumed to be in world which doesn't seem right.
      desiredPosition.setIncludingFrame(worldFrame, 0.0, 0.0, zInWorld);
      desiredPosition.changeFrame(frameOfLastFoostep);

      double zOffset = desiredPosition.getZ() - spline.getPosition();
      return zOffset;
   }

   private int queueExceedingTrajectoryPointsIfNeeded(PelvisHeightTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getEuclideanTrajectory().getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = offsetHeightTrajectoryGenerator.getMaximumNumberOfWaypoints()
            - offsetHeightTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      PelvisHeightTrajectoryCommand commandForExcedent = commandQueue.addFirst();
      numberOfQueuedCommands.increment();
      commandForExcedent.clear();
      commandForExcedent.getEuclideanTrajectory().setPropertiesOnly(command.getEuclideanTrajectory());

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.getEuclideanTrajectory().addTrajectoryPoint(command.getEuclideanTrajectory().getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getEuclideanTrajectory().getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.getEuclideanTrajectory().subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   public void goHome(double trajectoryTime)
   {
      if (!processGoHome.getValue())
         return;

      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightTrajectoryGenerator.clear();
      offsetHeightTrajectoryGenerator.appendWaypoint(0.0, offsetHeightAboveGroundPrevValue.getDoubleValue(), 0.0);
      offsetHeightTrajectoryGenerator.appendWaypoint(trajectoryTime, 0.0, 0.0);
      offsetHeightTrajectoryGenerator.initialize();
      isTrajectoryOffsetStopped.set(false);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryOffsetStopped.set(command.isStopAllTrajectory());
      offsetHeightAboveGround.set(offsetHeightAboveGroundPrevValue.getDoubleValue());
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
      return yoTime.getValue() - offsetHeightAboveGroundChangedTime.getValue();
   }
}
