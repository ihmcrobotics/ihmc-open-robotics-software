package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.communication.packets.Packet.*;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.StringStretcher2d;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * TODO There is not enough HumanoidReferenceFrames in that class, it is pretty fragile
 *
 */
public class LookAheadCoMHeightTrajectoryGenerator
{
   private static final boolean CONSIDER_NEXT_FOOTSTEP = false;

   private static final boolean DEBUG = false;

   private boolean visualize = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFourPointCubicSpline1D spline = new YoFourPointCubicSpline1D("height", registry);

   private final YoBoolean isTrajectoryOffsetStopped = new YoBoolean("isPelvisOffsetHeightTrajectoryStopped", registry);

   private final YoBoolean hasBeenInitializedWithNextStep = new YoBoolean("hasBeenInitializedWithNextStep", registry);

   private final YoDouble offsetHeightAboveGround = new YoDouble("offsetHeightAboveGround", registry);

   private final YoDouble offsetHeightAboveGroundPrevValue = new YoDouble("offsetHeightAboveGroundPrevValue", registry);
   private final YoDouble offsetHeightAboveGroundChangedTime = new YoDouble("offsetHeightAboveGroundChangedTime", registry);
   private final YoVariableDoubleProvider offsetHeightAboveGroundTrajectoryOutput = new YoVariableDoubleProvider("offsetHeightAboveGroundTrajectoryOutput",
         registry);
   private final YoVariableDoubleProvider offsetHeightAboveGroundTrajectoryTimeProvider = new YoVariableDoubleProvider(
         "offsetHeightAboveGroundTrajectoryTimeProvider", registry);
   private final MultipleWaypointsTrajectoryGenerator offsetHeightTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(
         "pelvisHeightOffset", registry);

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

   private final YoDouble yoTime;

   private ReferenceFrame frameOfLastFoostep;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<? extends ReferenceFrame> ankleZUpFrames;

   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole;

   private final YoLong lastCommandId;

   private final YoBoolean isReadyToHandleQueuedCommands;
   private final YoLong numberOfQueuedCommands;
   private final RecyclingArrayDeque<PelvisHeightTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(PelvisHeightTrajectoryCommand.class, PelvisHeightTrajectoryCommand::set);

   private final BooleanProvider processGoHome = new BooleanParameter("ProcessGoHome", registry, false);

   public LookAheadCoMHeightTrajectoryGenerator(double minimumHeightAboveGround, double nominalHeightAboveGround, double maximumHeightAboveGround,
         double defaultOffsetHeightAboveGround, double doubleSupportPercentageIn, ReferenceFrame centerOfMassFrame, ReferenceFrame pelvisFrame,
         SideDependentList<? extends ReferenceFrame> ankleZUpFrames, SideDependentList<RigidBodyTransform> transformsFromAnkleToSole, final YoDouble yoTime,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.pelvisFrame = pelvisFrame;
      this.centerOfMassFrame = centerOfMassFrame;
      this.ankleZUpFrames = ankleZUpFrames;
      frameOfLastFoostep = ankleZUpFrames.get(RobotSide.LEFT);
      this.yoTime = yoTime;
      this.transformsFromAnkleToSole = transformsFromAnkleToSole;
      offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
      offsetHeightAboveGroundTrajectoryTimeProvider.set(0.5);
      offsetHeightAboveGround.set(defaultOffsetHeightAboveGround);
      offsetHeightAboveGroundPrevValue.set(defaultOffsetHeightAboveGround);
      offsetHeightAboveGround.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
            double previous = offsetHeightTrajectoryGenerator.getValue();
            offsetHeightTrajectoryGenerator.clear();
            offsetHeightTrajectoryGenerator.appendWaypoint(0.0, previous, 0.0);
            offsetHeightTrajectoryGenerator.appendWaypoint(offsetHeightAboveGroundTrajectoryTimeProvider.getValue(),
                  offsetHeightAboveGround.getDoubleValue(), 0.0);
            offsetHeightTrajectoryGenerator.initialize();
         }
      });

      setMinimumHeightAboveGround(minimumHeightAboveGround);
      setNominalHeightAboveGround(nominalHeightAboveGround);
      setMaximumHeightAboveGround(maximumHeightAboveGround);
      previousZFinalLeft.set(nominalHeightAboveGround);
      previousZFinalRight.set(nominalHeightAboveGround);

      hasBeenInitializedWithNextStep.set(false);

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

   public void setCoMHeightDriftCompensation(boolean activate)
   {
      correctForCoMHeightDrift.set(activate);
   }

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

      spline.initialize(s0, d0, dF, sF);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoDouble previousZFinal = previousZFinals.get(robotSide);
         tempFramePoint.setIncludingFrame(frameOfLastFoostep, 0.0, 0.0, previousZFinal.getDoubleValue());
         tempFramePoint.changeFrame(newFrame);
         previousZFinal.set(tempFramePoint.getZ());
      }

      frameOfLastFoostep = newFrame;
   }

   private final Point2D tempPoint2dA = new Point2D();
   private final Point2D tempPoint2dB = new Point2D();

   private final Point2D s0Min = new Point2D();
   private final Point2D d0Min = new Point2D();
   private final Point2D dFMin = new Point2D();
   private final Point2D sFMin = new Point2D();
   private final Point2D sNextMin = new Point2D();

   private final Point2D s0Nom = new Point2D();
   private final Point2D d0Nom = new Point2D();
   private final Point2D dFNom = new Point2D();
   private final Point2D sFNom = new Point2D();
   private final Point2D sNextNom = new Point2D();

   private final Point2D s0Max = new Point2D();
   private final Point2D d0Max = new Point2D();
   private final Point2D dFMax = new Point2D();
   private final Point2D sFMax = new Point2D();
   private final Point2D sNextMax = new Point2D();

   private final Point2D s0 = new Point2D();
   private final Point2D d0 = new Point2D();
   private final Point2D dF = new Point2D();
   private final Point2D sF = new Point2D();
   private final Point2D sNext = new Point2D();

   private final FramePoint3D framePointS0 = new FramePoint3D();
   private final FramePoint3D framePointD0 = new FramePoint3D();
   private final FramePoint3D framePointDF = new FramePoint3D();
   private final FramePoint3D framePointSF = new FramePoint3D();
   private final FramePoint3D framePointSNext = new FramePoint3D();

   private final FramePoint3D tempFramePointForViz1 = new FramePoint3D();
   private final FramePoint3D tempFramePointForViz2 = new FramePoint3D();

   private final FramePoint3D transferFromContactFramePosition = new FramePoint3D();
   private final FramePoint3D transferToContactFramePosition = new FramePoint3D();
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


      transferFromFootstep.getAnklePosition(transferFromContactFramePosition, transformsFromAnkleToSole.get(transferFromFootstep.getRobotSide()));
      transferToFootstep.getAnklePosition(transferToContactFramePosition, transformsFromAnkleToSole.get(transferToFootstep.getRobotSide()));

      boolean frameDrifted = false;
      if (correctForCoMHeightDrift.getBooleanValue() && transferToFootstep.getTrustHeight() && (transferFromDesiredFootstep != null))
      {
         if (transferFromDesiredFootstep.getRobotSide() != transferFromFootstep.getRobotSide())
         {
            if (DEBUG)
            {
               System.err.println(
                     "transferFromDesiredFootstep.getRobotSide() != transferFromFootstep.getRobotSide() in LookAheadCoMHeightTrajectoryGenerator.initialize()");
            }
         }
         else
         {
            transferFromDesiredFootstep.getAnklePosition(transferFromDesiredContactFramePosition, transformsFromAnkleToSole.get(transferFromDesiredFootstep.getRobotSide()));
            transferFromDesiredContactFramePosition.changeFrame(transferFromContactFramePosition.getReferenceFrame());

            fromContactFrameDrift.setToZero(transferFromContactFramePosition.getReferenceFrame());
            fromContactFrameDrift.sub(transferFromContactFramePosition, transferFromDesiredContactFramePosition);
            fromContactFrameDrift.changeFrame(transferToContactFramePosition.getReferenceFrame());
            transferToContactFramePosition.setZ(transferToContactFramePosition.getZ() + fromContactFrameDrift.getZ());
            frameDrifted = true;
         }
      }

      transferFromContactFramePosition.changeFrame(worldFrame);
      transferToContactFramePosition.changeFrame(worldFrame);

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

      contactFrameZeroPosition.set(transferFromContactFramePosition);
      contactFrameOnePosition.set(transferToContactFramePosition);

      getPoint2d(tempPoint2dA, transferFromContactFramePosition);
      getPoint2d(tempPoint2dB, transferToContactFramePosition);

      projectionSegment.set(tempPoint2dA, tempPoint2dB);
      setPointXValues(nextContactFramePosition);

      transferFromContactFramePosition.changeFrame(frameOfLastFoostep);
      transferToContactFramePosition.changeFrame(frameOfLastFoostep);

      double footHeight0 = transferFromContactFramePosition.getZ();
      double footHeight1 = transferToContactFramePosition.getZ();

      double nextFootHeight = Double.NaN;

      if (CONSIDER_NEXT_FOOTSTEP)
      {
         if (nextContactFramePosition != null)
         {
            nextContactFramePosition.changeFrame(frameOfLastFoostep);
            nextFootHeight = nextContactFramePosition.getZ();
         }
      }

      s0Min.setY(footHeight0 + minimumHeightAboveGround.getDoubleValue());
      s0Nom.setY(footHeight0 + nominalHeightAboveGround.getDoubleValue());
      s0Max.setY(footHeight0 + maximumHeightAboveGround.getDoubleValue());

      sFMin.setY(footHeight1 + minimumHeightAboveGround.getDoubleValue());
      sFNom.setY(footHeight1 + nominalHeightAboveGround.getDoubleValue());
      sFMax.setY(footHeight1 + maximumHeightAboveGround.getDoubleValue());

      d0Min.setY(findMinimumDoubleSupportHeight(s0.getX(), sF.getX(), d0.getX(), footHeight0, footHeight1));
      d0Nom.setY(findNominalDoubleSupportHeight(s0.getX(), sF.getX(), d0.getX(), footHeight0, footHeight1));
      d0Max.setY(findMaximumDoubleSupportHeight(s0.getX(), sF.getX(), d0.getX(), footHeight0, footHeight1));

      dFMin.setY(findMinimumDoubleSupportHeight(s0.getX(), sF.getX(), dF.getX(), footHeight0, footHeight1));
      dFNom.setY(findNominalDoubleSupportHeight(s0.getX(), sF.getX(), dF.getX(), footHeight0, footHeight1));
      dFMax.setY(findMaximumDoubleSupportHeight(s0.getX(), sF.getX(), dF.getX(), footHeight0 + extraToeOffHeight, footHeight1));

      sNextMin.setY(nextFootHeight + minimumHeightAboveGround.getDoubleValue());
      sNextNom.setY(nextFootHeight + nominalHeightAboveGround.getDoubleValue());
      sNextMax.setY(nextFootHeight + maximumHeightAboveGround.getDoubleValue());

      computeHeightsToUseByStretchingString(transferFromFootstep.getRobotSide());
      previousZFinals.get(transferToFootstep.getRobotSide()).set(sF.getY());

      spline.initialize(s0, d0, dF, sF);

      if (visualize)
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

         if (CONSIDER_NEXT_FOOTSTEP)
         {
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
            tempFramePointForViz1.setToZero(transferFromContactFramePosition.getReferenceFrame());
            tempFramePointForViz1.interpolate(transferFromContactFramePosition, transferToContactFramePosition, ((double) i) / ((double) numberOfPoints));
            tempFramePointForViz1.changeFrame(worldFrame);
            queryPoint.set(tempFramePointForViz1);
            this.solve(coMHeightPartialDerivativesData, queryPoint, false);
            coMHeightPartialDerivativesData.getCoMHeight(tempFramePointForViz2);
            tempFramePointForViz2.setX(tempFramePointForViz1.getX());
            tempFramePointForViz2.setY(tempFramePointForViz1.getY());

            bagOfBalls.setBallLoop(tempFramePointForViz2);
         }
      }
   }

   private Point2D tempPoint2dForStringStretching = new Point2D();

   private final StringStretcher2d stringStretcher2d = new StringStretcher2d();
   private final List<Point2D> stretchedStringWaypoints = new ArrayList<>();
   private void computeHeightsToUseByStretchingString(RobotSide transferFromSide)
   {
      stringStretcher2d.reset();
      // s0 is at previous
      double z0 = MathTools.clamp(previousZFinals.get(transferFromSide).getDoubleValue(), s0Min.getY(), s0Max.getY());
      s0.setY(z0);

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

      stringStretcher2d.stretchString(stretchedStringWaypoints);

      d0.set(stretchedStringWaypoints.get(1));
      dF.set(stretchedStringWaypoints.get(2));
      sF.set(stretchedStringWaypoints.get(3));
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

   private void getPoint2d(Point2D point2dToPack, FramePoint3D point)
   {
      point2dToPack.set(point.getX(), point.getY());
   }

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final Point2D solutionPoint = new Point2D();

   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, boolean isInDoubleSupport)
   {
      getCenterOfMass2d(solutionPoint, centerOfMassFrame);
      solve(coMHeightPartialDerivativesDataToPack, solutionPoint, isInDoubleSupport);

      coMHeightPartialDerivativesDataToPack.getCoMHeight(tempFramePoint);
      desiredCoMPosition.set(solutionPoint.getX(), solutionPoint.getY(), tempFramePoint.getZ());
   }

   private final FramePoint3D height = new FramePoint3D();
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final double[] partialDerivativesWithRespectToS = new double[2];

   private void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, Point2D queryPoint, boolean isInDoubleSupport)
   {
      projectionSegment.orthogonalProjection(queryPoint);
      double splineQuery = projectionSegment.percentageAlongLineSegment(queryPoint) * projectionSegment.length();

      // Happens when the robot gets stuck in double support but the ICP is still being dragged in the front support foot.
      if (isInDoubleSupport)
         splineQuery = Math.min(splineQuery, dF.getX());

      spline.compute(splineQuery);

      handleInitializeToCurrent();

      if (!isTrajectoryOffsetStopped.getBooleanValue())
      {
         double deltaTime = yoTime.getDoubleValue() - offsetHeightAboveGroundChangedTime.getDoubleValue();

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

      double z = spline.getY() + offsetHeightAboveGroundTrajectoryOutput.getValue();
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

         offsetHeightAboveGround.set(heightOffset);
         offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());

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
         offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
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
                       previousCommandId, lastCommandId.getLongValue());
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
         FrameEuclideanTrajectoryPoint waypoint = command.getEuclideanTrajectory().getTrajectoryPoint(trajectoryPointIndex);
         double time = waypoint.getTime();
         double z = waypoint.getPositionZ();
         double zDot = waypoint.getLinearVelocityZ();

         // TODO (Sylvain) Check if that's the right way to do it
         desiredPosition.setIncludingFrame(worldFrame, 0.0, 0.0, z);
         desiredPosition.changeFrame(frameOfLastFoostep);

         double zOffset = desiredPosition.getZ() - spline.getY();

         offsetHeightTrajectoryGenerator.appendWaypoint(time, zOffset, zDot);
      }

      offsetHeightTrajectoryGenerator.initialize();
      isTrajectoryOffsetStopped.set(false);
   }

   private int queueExceedingTrajectoryPointsIfNeeded(PelvisHeightTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getEuclideanTrajectory().getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = offsetHeightTrajectoryGenerator.getMaximumNumberOfWaypoints() - offsetHeightTrajectoryGenerator.getCurrentNumberOfWaypoints();

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

      offsetHeightAboveGroundChangedTime.set(yoTime.getDoubleValue());
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

      getPoint2d(point2dToPack, coM);
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
            + position.getY() + ", " + position.getZ() + "), new Quat4d(" + orientation.getS() + ", " + orientation.getX()
            + ", " + orientation.getY() + ", " + orientation.getZ() + ")));");
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
