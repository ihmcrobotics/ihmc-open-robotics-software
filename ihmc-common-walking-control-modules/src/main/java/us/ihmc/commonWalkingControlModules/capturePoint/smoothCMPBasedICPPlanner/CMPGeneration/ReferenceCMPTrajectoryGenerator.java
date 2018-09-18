package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.TorqueTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class ReferenceCMPTrajectoryGenerator
{
   private static final int maxNumberOfCoefficients = 10;
   private static final int maxNumberOfSegments = 35;

   private static final double POINT_SIZE =  0.005;
   private static final boolean VISUALIZE = false;

   private static final double trajectoryEpsilon = Epsilons.ONE_HUNDRED_THOUSANDTH;

   private final List<CMPTrajectory> transferCMPTrajectories = new ArrayList<>();
   private final List<CMPTrajectory> swingCMPTrajectories = new ArrayList<>();
   private final YoDouble verticalGroundReaction;
   private final YoInteger numberOfFootstepsToConsider;

   private double initialTime;
   private int numberOfRegisteredSteps;
   private CMPTrajectory activeTrajectory;

   private final FramePoint3D desiredCMP = new FramePoint3D();
   private final FrameVector3D desiredCMPVelocity = new FrameVector3D();
   private final TorqueTrajectory torqueTrajectory;

   private final List<YoFramePoint3D> swingBounds = new ArrayList<>();
   private final List<YoFramePoint3D> transferBounds = new ArrayList<>();

   private final boolean debug;

   public ReferenceCMPTrajectoryGenerator(String namePrefix, int maxNumberOfFootstepsToConsider, YoInteger numberOfFootstepsToConsider,
                                          boolean debug, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
       this.debug = debug && yoGraphicsListRegistry != null && VISUALIZE;
      String fullPrefix = namePrefix + "CMPTrajectoryGenerator";
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         CMPTrajectory transferCMPTrajectory = new CMPTrajectory(maxNumberOfSegments, maxNumberOfCoefficients);
         CMPTrajectory swingCMPTrajectory = new CMPTrajectory(maxNumberOfSegments, maxNumberOfCoefficients);
         transferCMPTrajectories.add(transferCMPTrajectory);
         swingCMPTrajectories.add(swingCMPTrajectory);
      }
      CMPTrajectory transferCMPTrajectory = new CMPTrajectory(maxNumberOfSegments, maxNumberOfCoefficients);
      transferCMPTrajectories.add(transferCMPTrajectory);
      this.torqueTrajectory = new TorqueTrajectory(maxNumberOfSegments, maxNumberOfCoefficients);
      this.verticalGroundReaction = new YoDouble(fullPrefix + "CMPTorqueOffsetScalingFactor", registry);

      if (this.debug)
      {
         ArtifactList cmpWaypointList = new ArtifactList("CMP Waypoints");
         for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
         {
            YoFramePoint3D transferStart = new YoFramePoint3D("TransferCMPWaypoint" + i + 0, ReferenceFrame.getWorldFrame(), registry);
            YoFramePoint3D swingStart = new YoFramePoint3D("SwingCMPWaypoint" + i + 0, ReferenceFrame.getWorldFrame(), registry);

            YoGraphicPosition transferStartViz = new YoGraphicPosition("Transfer CMP Waypoint" + i + 0, transferStart, POINT_SIZE, YoAppearance.Green(), YoGraphicPosition.GraphicType.SQUARE_WITH_CROSS);
            YoGraphicPosition swingStartViz = new YoGraphicPosition("Swing CMP Waypoint" + i + 0, swingStart, POINT_SIZE, YoAppearance.Green(), YoGraphicPosition.GraphicType.SOLID_BALL);

            cmpWaypointList.add(transferStartViz.createArtifact());
            cmpWaypointList.add(swingStartViz.createArtifact());


            transferBounds.add(transferStart);
            swingBounds.add(swingStart);

            for (int j = 0; j < maxNumberOfSegments; j++)
            {
               YoFramePoint3D transferEnd = new YoFramePoint3D("TransferCMPWaypoint" + i + j + 1, ReferenceFrame.getWorldFrame(), registry);
               YoFramePoint3D swingEnd = new YoFramePoint3D("SwingCMPWaypoint" + i + j + 1, ReferenceFrame.getWorldFrame(), registry);

               YoGraphicPosition transferViz = new YoGraphicPosition("Transfer CMP Waypoint" + i + j + 1, transferEnd, POINT_SIZE, YoAppearance.Green(), YoGraphicPosition.GraphicType.SQUARE_WITH_CROSS);
               YoGraphicPosition swingViz = new YoGraphicPosition("Swing CMP Waypoint" + i + j + 1, swingEnd, POINT_SIZE, YoAppearance.Green(), YoGraphicPosition.GraphicType.SQUARE_WITH_CROSS);

               cmpWaypointList.add(transferViz.createArtifact());
               cmpWaypointList.add(swingViz.createArtifact());

               transferBounds.add(transferEnd);
               swingBounds.add(swingEnd);
            }

            YoFramePoint3D transferEnd = new YoFramePoint3D("TransferCMPWaypoint" + i + maxNumberOfSegments + 1, ReferenceFrame.getWorldFrame(), registry);
            YoGraphicPosition transferViz = new YoGraphicPosition("Transfer CMP Waypoint" + i + maxNumberOfSegments + 1, transferEnd, POINT_SIZE, YoAppearance.Green(), YoGraphicPosition.GraphicType.SQUARE_WITH_CROSS);
            cmpWaypointList.add(transferViz.createArtifact());

            transferBounds.add(transferEnd);
         }

         yoGraphicsListRegistry.registerArtifactList(cmpWaypointList);
      }
   }

   public void setGroundReaction(double z)
   {
      this.verticalGroundReaction.set(z);
   }

   public void reset()
   {
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         transferCMPTrajectories.get(i).reset();
         swingCMPTrajectories.get(i).reset();
      }
      transferCMPTrajectories.get(numberOfFootstepsToConsider.getIntegerValue()).reset();
      activeTrajectory = null;
   }

   public void update(double currentTime)
   {
      double timeInState = currentTime - initialTime;

      if (activeTrajectory != null)
         activeTrajectory.update(timeInState, desiredCMP, desiredCMPVelocity);
   }

   public void getPosition(FixedFramePoint3DBasics desiredCMPToPack)
   {
      desiredCMPToPack.set(desiredCMP);
   }

   public void getVelocity(FixedFrameVector3DBasics desiredCMPVelocityToPack)
   {
      desiredCMPVelocityToPack.set(desiredCMPVelocity);
   }

   public void getLinearData(FixedFramePoint3DBasics positionToPack, FixedFrameVector3DBasics velocityToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
   }

   public List<CMPTrajectory> getTransferCMPTrajectories()
   {
      return transferCMPTrajectories;
   }

   public List<CMPTrajectory> getSwingCMPTrajectories()
   {
      return swingCMPTrajectories;
   }

   public void setNumberOfRegisteredSteps(int numberOfRegisteredSteps)
   {
      this.numberOfRegisteredSteps = numberOfRegisteredSteps;
   }

   public void initializeForTransfer(double currentTime, List<? extends CoPTrajectory> transferCoPTrajectories,
                                     List<? extends CoPTrajectory> swingCoPTrajectories,
                                     List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories,
                                     List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories)
   {
      this.initialTime = currentTime;
      setCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories, swingAngularMomentumTrajectories,
                         WalkingTrajectoryType.TRANSFER);
      activeTrajectory = transferCMPTrajectories.get(0);
   }

   public void initializeForSwing(double currentTime, List<? extends CoPTrajectory> transferCoPTrajectories, List<? extends CoPTrajectory> swingCoPTrajectories,
                                  List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories,
                                  List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories)
   {
      this.initialTime = currentTime;
      setCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories, transferAngularMomentumTrajectories, swingAngularMomentumTrajectories,
                         WalkingTrajectoryType.SWING);
      activeTrajectory = swingCMPTrajectories.get(0);
   }

   private void setCMPTrajectories(List<? extends CoPTrajectory> transferCoPTrajectories, List<? extends CoPTrajectory> swingCoPTrajectories,
                                   List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories,
                                   List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories, WalkingTrajectoryType currentPhase)
   {
      if (debug)
      {
         for (int i = 0; i < swingBounds.size(); i++)
         {
            transferBounds.get(i).setToNaN();
            swingBounds.get(i).setToNaN();
         }
      }
      copyCoPTrajectoriesToCMPTrajectories(transferCoPTrajectories, swingCoPTrajectories);

      if (transferAngularMomentumTrajectories == null || swingAngularMomentumTrajectories == null)
      {
         return;
      }

      int numberOfFootstepsToSet = Math.min(numberOfFootstepsToConsider.getIntegerValue(), numberOfRegisteredSteps);
      int phaseIndex = 0;

      int swingWaypointNumber = 0;
      int transferWaypointNumber = 0;

      // handle current swing if that's the current walking phase
      if (currentPhase == WalkingTrajectoryType.SWING)
      {
         torqueTrajectory.setFromAngularMomentumTrajectory(swingAngularMomentumTrajectories.get(phaseIndex), verticalGroundReaction.getDoubleValue());
         if (swingCoPTrajectories.get(phaseIndex).getNumberOfSegments() == 0 || torqueTrajectory.getNumberOfSegments() == 0)
         {
            return;
         }
         TrajectoryMathTools.addSegmentedTrajectories(swingCMPTrajectories.get(phaseIndex), swingCoPTrajectories.get(phaseIndex), torqueTrajectory,
                                                      trajectoryEpsilon);
         phaseIndex++;

         if (debug)
         {
            if (swingCMPTrajectories.get(phaseIndex).getNumberOfSegments() > 1)
            {
               swingCMPTrajectories.get(phaseIndex).getSegment(0).getStartPoint(swingBounds.get(swingWaypointNumber++));
            }
            for (int i = 0; i < swingCMPTrajectories.get(phaseIndex).getNumberOfSegments(); i++)
            {
               swingCMPTrajectories.get(phaseIndex).getSegment(i).getEndPoint(swingBounds.get(swingWaypointNumber++));
            }
         }
      }

      // handle all upcoming state phases
      for (; phaseIndex < numberOfFootstepsToSet; phaseIndex++)
      {
         // transfer
         torqueTrajectory.setFromAngularMomentumTrajectory(transferAngularMomentumTrajectories.get(phaseIndex), verticalGroundReaction.getDoubleValue());
         if (transferCoPTrajectories.get(phaseIndex).getNumberOfSegments() == 0 || torqueTrajectory.getNumberOfSegments() == 0)
         {
            return;
         }
         TrajectoryMathTools.addSegmentedTrajectories(transferCMPTrajectories.get(phaseIndex), transferCoPTrajectories.get(phaseIndex), torqueTrajectory,
                                                      trajectoryEpsilon);

         if (debug)
         {
            if (transferCMPTrajectories.get(phaseIndex).getNumberOfSegments() > 1)
            {
               transferCMPTrajectories.get(phaseIndex).getSegment(0).getStartPoint(transferBounds.get(transferWaypointNumber++));
            }
            for (int i = 0; i < transferCMPTrajectories.get(phaseIndex).getNumberOfSegments(); i++)
            {
               transferCMPTrajectories.get(phaseIndex).getSegment(i).getEndPoint(transferBounds.get(transferWaypointNumber++));
            }
         }

         // swing
         torqueTrajectory.setFromAngularMomentumTrajectory(swingAngularMomentumTrajectories.get(phaseIndex), verticalGroundReaction.getDoubleValue());
         if (swingCoPTrajectories.get(phaseIndex).getNumberOfSegments() == 0 || torqueTrajectory.getNumberOfSegments() == 0)
         {
            return;
         }
         TrajectoryMathTools.addSegmentedTrajectories(swingCMPTrajectories.get(phaseIndex), swingCoPTrajectories.get(phaseIndex), torqueTrajectory,
                                                      trajectoryEpsilon);

         if (debug)
         {
            if (swingCMPTrajectories.get(phaseIndex).getNumberOfSegments() > 1)
            {
               swingCMPTrajectories.get(phaseIndex).getSegment(0).getStartPoint(swingBounds.get(swingWaypointNumber++));
            }
            for (int i = 0; i < swingCMPTrajectories.get(phaseIndex).getNumberOfSegments(); i++)
            {
               swingCMPTrajectories.get(phaseIndex).getSegment(i).getEndPoint(swingBounds.get(swingWaypointNumber++));
            }
         }
      }

      // handle final transfer
      torqueTrajectory
            .setFromAngularMomentumTrajectory(transferAngularMomentumTrajectories.get(numberOfFootstepsToSet), verticalGroundReaction.getDoubleValue());
      if (transferCoPTrajectories.get(numberOfFootstepsToSet).getNumberOfSegments() == 0 || torqueTrajectory.getNumberOfSegments() == 0)
         return;
      TrajectoryMathTools.addSegmentedTrajectories(transferCMPTrajectories.get(numberOfFootstepsToSet), transferCoPTrajectories.get(numberOfFootstepsToSet),
                                                   torqueTrajectory, trajectoryEpsilon);

      if (debug)
      {
         if (transferCMPTrajectories.get(numberOfFootstepsToSet).getNumberOfSegments() > 1)
         {
            transferCMPTrajectories.get(numberOfFootstepsToSet).getSegment(0).getStartPoint(transferBounds.get(transferWaypointNumber++));
         }
         for (int i = 0; i < transferCMPTrajectories.get(phaseIndex).getNumberOfSegments(); i++)
         {
            transferCMPTrajectories.get(numberOfFootstepsToSet).getSegment(i).getEndPoint(transferBounds.get(transferWaypointNumber++));
         }
      }
   }

   private void copyCoPTrajectoriesToCMPTrajectories(List<? extends CoPTrajectory> transferCoPTrajectories, List<? extends CoPTrajectory> swingCoPTrajectories)
   {
      int numberOfFootstepsToCopy = Math.min(numberOfFootstepsToConsider.getIntegerValue(), numberOfRegisteredSteps);
      for (int i = 0; i < numberOfFootstepsToCopy; i++)
      {
         transferCMPTrajectories.get(i).setAll(transferCoPTrajectories.get(i));
         swingCMPTrajectories.get(i).setAll(swingCoPTrajectories.get(i));
      }
      transferCMPTrajectories.get(numberOfFootstepsToCopy).setAll(transferCoPTrajectories.get(numberOfFootstepsToCopy));
   }
}
