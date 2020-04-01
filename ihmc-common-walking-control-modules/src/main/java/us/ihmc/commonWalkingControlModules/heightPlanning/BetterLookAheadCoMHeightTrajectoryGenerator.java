package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class BetterLookAheadCoMHeightTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private boolean visualize = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble minimumHeightAboveGround = new YoDouble("minimumHeightAboveGround", registry);
   private final YoDouble nominalHeightAboveGround = new YoDouble("nominalHeightAboveGround", registry);
   private final YoDouble maximumHeightAboveGround = new YoDouble("maximumHeightAboveGround", registry);

   private final YoDouble doubleSupportPercentageIn = new YoDouble("doubleSupportPercentageIn", registry);
   private final YoDouble percentageThroughSegment = new YoDouble("percentageThroughSegment", registry);
   private final YoDouble splineQuery = new YoDouble("splineQuery", registry);

   private final YoFramePoint3D transferFromPosition = new YoFramePoint3D("transferFromPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D transferToPosition = new YoFramePoint3D("transferToPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D desiredCoMPositionAtStart = new YoFramePoint3D("desiredCoMPositionAtStart", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble desiredCoMHeight = new YoDouble("desiredCoMHeight", registry);
   private final YoFramePoint3D queryPosition = new YoFramePoint3D("queryPosition", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleProvider yoTime;

   private ReferenceFrame frameOfLastFootstep;
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final FramePoint3D com = new FramePoint3D();

   private final FramePoint3D tempFramePoint = new FramePoint3D();

   private final FramePoint3D startCoMPosition = new FramePoint3D();
   private final FramePoint3D middleCoMPosition = new FramePoint3D();

   private final SplinedHeightTrajectory splinedHeightTrajectory;

   public BetterLookAheadCoMHeightTrajectoryGenerator(double minimumHeightAboveGround,
                                                      double nominalHeightAboveGround,
                                                      double maximumHeightAboveGround,
                                                      double defaultOffsetHeightAboveGround,
                                                      double doubleSupportPercentageIn,
                                                      ReferenceFrame centerOfMassFrame,
                                                      SideDependentList<? extends ReferenceFrame> soleFrames,
                                                      DoubleProvider yoTime,
                                                      YoGraphicsListRegistry yoGraphicsListRegistry,
                                                      YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.soleFrames = soleFrames;
      this.yoTime = yoTime;

      setMinimumHeightAboveGround(minimumHeightAboveGround);
      setNominalHeightAboveGround(nominalHeightAboveGround);
      setMaximumHeightAboveGround(maximumHeightAboveGround);

      splinedHeightTrajectory = new SplinedHeightTrajectory(this.doubleSupportPercentageIn, registry, yoGraphicsListRegistry);

      setSupportLeg(RobotSide.LEFT);

      this.doubleSupportPercentageIn.set(doubleSupportPercentageIn);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry == null)
         visualize = false;

      if (visualize && yoGraphicsListRegistry != null)
      {
         double pointSize = 0.03;

         String prefix = "better_";


         YoGraphicPosition desiredCoMPositionViz = new YoGraphicPosition(prefix + "desiredCoMPosition",
                                                                         desiredCoMPosition,
                                                                         1.1 * pointSize,
                                                                         YoAppearance.Gold());

         String graphicListName = "BetterCoMHeightTrajectoryGenerator";

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, desiredCoMPositionViz);
      }
   }

   public void reset()
   {
      tempFramePoint.setToZero(centerOfMassFrame);
      tempFramePoint.changeFrame(worldFrame);

      desiredCoMPosition.set(tempFramePoint);
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
      frameOfLastFootstep = soleFrames.get(supportLeg);
      splinedHeightTrajectory.setReferenceFrame(frameOfLastFootstep);
   }

   public void initialize(NewTransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      FramePoint3DReadOnly transferToFootstepPosition = transferToAndNextFootstepsData.getTransferToPosition();
      FramePoint3DReadOnly nextFootstepPosition = transferToAndNextFootstepsData.getNextFootstepPosition();

      tempFramePoint.setToZero(frameOfLastFootstep);
      transferFromPosition.setMatchingFrame(tempFramePoint);
      transferToPosition.setMatchingFrame(transferToFootstepPosition);
      desiredCoMPositionAtStart.set(desiredCoMPosition);

      startCoMPosition.setIncludingFrame(desiredCoMPosition);
      startCoMPosition.changeFrame(frameOfLastFootstep);

      middleCoMPosition.setIncludingFrame(transferToFootstepPosition);
      middleCoMPosition.changeFrame(frameOfLastFootstep);
      double middleAnkleZ = middleCoMPosition.getZ();

      middleCoMPosition.addZ(nominalHeightAboveGround.getDoubleValue());

      double midstanceWidth = 0.5 * middleCoMPosition.getY();

      startCoMPosition.setY(midstanceWidth);
      middleCoMPosition.setY(midstanceWidth);

      splinedHeightTrajectory.computeWaypoints(startCoMPosition, middleCoMPosition, 0.0, middleAnkleZ, minimumHeightAboveGround.getDoubleValue(), maximumHeightAboveGround.getDoubleValue());
      splinedHeightTrajectory.computeSpline();
   }

   public void solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, boolean isInDoubleSupport)
   {
      com.setToZero(centerOfMassFrame);
      com.changeFrame(worldFrame);

      solve(comHeightPartialDerivativesDataToPack, com, isInDoubleSupport);

      desiredCoMPosition.set(com.getX(), com.getY(), comHeightPartialDerivativesDataToPack.getComHeight());
   }

   private final Point2D point = new Point2D();

   private void solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, FramePoint3DBasics queryPoint, boolean isInDoubleSupport)
   {
      this.queryPosition.set(queryPoint);

      percentageThroughSegment.set(splinedHeightTrajectory.solve(comHeightPartialDerivativesDataToPack, queryPoint, point));

      this.splineQuery.set(point.getX());
      this.desiredCoMHeight.set(point.getY());
   }

   public void goHome(double trajectoryTime)
   {
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
   }

   public void initializeDesiredHeightToCurrent()
   {
   }

   public void getCurrentDesiredHeight(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(desiredCoMPosition);
   }

   public double getOffsetHeightTimeInTrajectory()
   {
      return yoTime.getValue();
   }
}
