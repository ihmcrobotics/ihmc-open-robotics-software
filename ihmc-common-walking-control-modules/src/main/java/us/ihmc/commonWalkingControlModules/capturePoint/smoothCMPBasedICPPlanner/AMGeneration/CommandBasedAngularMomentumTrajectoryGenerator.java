package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPlanningTools;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class CommandBasedAngularMomentumTrajectoryGenerator implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final int waypointsPerWalkingPhase = 12;
   private static final int numberOfTrajectoryCoefficients = 4;
   private static final FramePoint3D zeroPoint = new FramePoint3D();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final MomentumTrajectoryHandler momentumTrajectoryHandler;
   private final YoDouble time;
   private final YoInteger numberOfRegisteredFootsteps;
   private final YoFrameVector3D commandedAngularMomentum = new YoFrameVector3D("commandedAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D commandedCoMTorque = new YoFrameVector3D("commandedCoMTorque", ReferenceFrame.getWorldFrame(), registry);
   private final YoBoolean atAStop = new YoBoolean("atAStop", registry);
   private final YoBoolean updateTrajectories = new YoBoolean("updateTrajectories", registry);

   private final RecyclingArrayList<EuclideanTrajectoryPoint> waypoints = new RecyclingArrayList<>(waypointsPerWalkingPhase,
                                                                                                         EuclideanTrajectoryPoint::new);
   private final List<AngularMomentumTrajectory> transferTrajectories;
   private final List<AngularMomentumTrajectory> swingTrajectories;

   private double initialTime;
   private AngularMomentumTrajectory activeTrajectory;
   private List<CoPPointsInFoot> copLocations;

   private final YoBoolean planSwingAngularMomentum;
   private final YoBoolean planTransferAngularMomentum;
   private int numberOfStepsToConsider;

   private final FrameVector3D desiredAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredTorque = new FrameVector3D();
   private final FrameVector3D desiredRotatum = new FrameVector3D();

   public CommandBasedAngularMomentumTrajectoryGenerator(MomentumTrajectoryHandler momentumTrajectoryHandler, YoDouble time, int maxNumberOfStepsToConsider,
                                                         YoVariableRegistry parentRegistry)
   {
      String namePrefix = "commandedAngMomentum";
      this.momentumTrajectoryHandler = momentumTrajectoryHandler;
      this.time = time;
      this.numberOfRegisteredFootsteps = new YoInteger(namePrefix + "NumberOfRegisteredFootsteps", registry);
      this.planSwingAngularMomentum = new YoBoolean("PlanSwingAngularMomentumWithCommand", registry);
      this.planTransferAngularMomentum = new YoBoolean("PlanTransferAngularMomentumWithCommand", registry);
      this.atAStop.set(true);
      this.updateTrajectories.set(false);

      transferTrajectories = new ArrayList<>(maxNumberOfStepsToConsider + 1);
      swingTrajectories = new ArrayList<>(maxNumberOfStepsToConsider);
      for (int i = 0; i < maxNumberOfStepsToConsider; i++)
      {
         transferTrajectories.add(new AngularMomentumTrajectory(waypointsPerWalkingPhase - 1, numberOfTrajectoryCoefficients));
         swingTrajectories.add(new AngularMomentumTrajectory(waypointsPerWalkingPhase - 1, numberOfTrajectoryCoefficients));
      }
      transferTrajectories.add(new AngularMomentumTrajectory(waypointsPerWalkingPhase - 1, numberOfTrajectoryCoefficients));

      parentRegistry.addChild(registry);
   }

   public void initializeParameters(ICPPlannerParameters icpPlannerParameters)
   {
      numberOfStepsToConsider = icpPlannerParameters.getNumberOfFootstepsToConsider();
      this.planSwingAngularMomentum.set(icpPlannerParameters.planSwingAngularMomentum());
      this.planTransferAngularMomentum.set(icpPlannerParameters.planTransferAngularMomentum());
   }

   @Override
   public void addCopAndComSetpointsToPlan(List<CoPPointsInFoot> copLocations, List<? extends FramePoint3DReadOnly> comInitialPositions,
                                           List<? extends FramePoint3DReadOnly> comFinalPositions, List<? extends FrameVector3DReadOnly> comInitialVelocities,
                                           List<? extends FrameVector3DReadOnly> comFinalVelocities,
                                           List<? extends FrameVector3DReadOnly> comInitialAccelerations,
                                           List<? extends FrameVector3DReadOnly> comFinalAccelerations, int numberOfRegisteredFootsteps)
   {
      this.numberOfRegisteredFootsteps.set(numberOfRegisteredFootsteps);
      this.copLocations = copLocations;
   }

   @Override
   public void clear()
   {
   }

   @Override
   public void update(double currentTime)
   {
      activeTrajectory.update(currentTime - initialTime, desiredAngularMomentum, desiredTorque, desiredRotatum);
   }

   @Override
   public void getDesiredAngularMomentum(FixedFrameVector3DBasics desiredAngMomToPack, FixedFrameVector3DBasics desiredTorqueToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);

      commandedAngularMomentum.set(desiredAngularMomentum);
      commandedCoMTorque.set(desiredTorque);
   }

   @Override
   public void initializeForDoubleSupport(double currentTime, boolean isStanding)
   {
      initialTime = currentTime;
      activeTrajectory = transferTrajectories.get(0);
   }

   @Override
   public void initializeForSingleSupport(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = swingTrajectories.get(0);
   }

   private void computeTrajectories(WalkingTrajectoryType startingTrajectoryType)
   {
      momentumTrajectoryHandler.clearPointsInPast();

      int stepIndex = 0;
      double currentTime = time.getDoubleValue();
      double accumulatedTime = 0.0;

      if (startingTrajectoryType.equals(WalkingTrajectoryType.SWING))
      {
         // at the start of swing, for some reason the copLocation corresponding to the upcoming swing is at index 1
         CoPPointsInFoot pointsInFoot = copLocations.get(1);

         double swingPhaseDuration = getPhaseDuration(WalkingTrajectoryType.SWING, pointsInFoot, CoPPointName.ENTRY_COP);
         momentumTrajectoryHandler.getAngularMomentumTrajectory(currentTime, currentTime + swingPhaseDuration, waypointsPerWalkingPhase, waypoints);
         setSubTrajectoryFromWaypoints(swingPhaseDuration, swingTrajectories.get(0), startingTrajectoryType);
         accumulatedTime += swingPhaseDuration;

         // the first transfer copLocation is at index 2 at the start of swing. otherwise it's at index 0
         stepIndex++;
      }

      int numberOfSteps = Math.min(numberOfRegisteredFootsteps.getIntegerValue(), numberOfStepsToConsider);
      for (; stepIndex < numberOfSteps; stepIndex++)
      {
         CoPPointsInFoot pointsInFoot = copLocations.get(stepIndex + 1);

         double transferPhaseDuration = getPhaseDuration(WalkingTrajectoryType.TRANSFER, pointsInFoot, CoPPointName.ENTRY_COP);
         momentumTrajectoryHandler
               .getAngularMomentumTrajectory(currentTime + accumulatedTime, currentTime + accumulatedTime + transferPhaseDuration, waypointsPerWalkingPhase,
                                             waypoints);
         setSubTrajectoryFromWaypoints(transferPhaseDuration, transferTrajectories.get(stepIndex), WalkingTrajectoryType.TRANSFER);
         accumulatedTime += transferPhaseDuration;

         double swingPhaseDuration = getPhaseDuration(WalkingTrajectoryType.SWING, pointsInFoot, CoPPointName.ENTRY_COP);
         momentumTrajectoryHandler
               .getAngularMomentumTrajectory(currentTime + accumulatedTime, currentTime + accumulatedTime + swingPhaseDuration, waypointsPerWalkingPhase,
                                             waypoints);
         setSubTrajectoryFromWaypoints(swingPhaseDuration, swingTrajectories.get(stepIndex), WalkingTrajectoryType.TRANSFER);
         accumulatedTime += swingPhaseDuration;
      }

      // handle terminal transfer
      CoPPointsInFoot pointsInFoot = copLocations.get(stepIndex + 1);
      double finalTransferDuration = getPhaseDuration(WalkingTrajectoryType.TRANSFER, pointsInFoot, CoPPointName.ENTRY_COP);
      momentumTrajectoryHandler
            .getAngularMomentumTrajectory(currentTime + accumulatedTime, currentTime + accumulatedTime + finalTransferDuration, waypointsPerWalkingPhase,
                                          waypoints);
      setSubTrajectoryFromWaypoints(finalTransferDuration, transferTrajectories.get(numberOfSteps), WalkingTrajectoryType.TRANSFER);
   }

   private boolean isInPhase(WalkingTrajectoryType phase)
   {
      if (activeTrajectory == null)
      {
         return false;
      }
      else if (phase.equals(WalkingTrajectoryType.SWING) && activeTrajectory == swingTrajectories.get(0))
      {
         return true;
      }
      else if (phase.equals(WalkingTrajectoryType.TRANSFER) && activeTrajectory == transferTrajectories.get(0))
      {
         return true;
      }
      return false;
   }

   /**
    * The trajectory points in CoPPointsInFoot go transfer then swing, with "entryCoPName" being the last transfer point.
    */
   private static double getPhaseDuration(WalkingTrajectoryType trajectoryType, CoPPointsInFoot pointsInFoot, CoPPointName entryCoPName)
   {
      double phaseTime = 0.0;
      int entryPointIndex = CoPPlanningTools.getCoPPointIndex(pointsInFoot.getCoPPointList(), entryCoPName);

      if (trajectoryType.equals(WalkingTrajectoryType.TRANSFER))
      {
         int finalTransferIndex = (entryPointIndex == -1) ? pointsInFoot.getNumberOfCoPPoints() - 1 : entryPointIndex;
         for (int i = 0; i <= finalTransferIndex; i++)
         {
            phaseTime += pointsInFoot.get(i).getTime();
         }
      }
      else
      {
         for (int i = entryPointIndex + 1; i < pointsInFoot.getNumberOfCoPPoints(); i++)
         {
            phaseTime += pointsInFoot.get(i).getTime();
         }
      }

      return phaseTime;
   }

   private void setSubTrajectoryFromWaypoints(double subTrajectoryDuration, AngularMomentumTrajectory subTrajectory,
                                              WalkingTrajectoryType walkingTrajectoryType)
   {
      subTrajectory.reset();

      if (waypoints.isEmpty() || !calculateAngularMomentumForPhase(walkingTrajectoryType))
      {
         subTrajectory.add().setConstant(0.0, subTrajectoryDuration, zeroPoint);
         return;
      }

      double startTime = waypoints.getFirst().getTime();
      boolean successfulTrajectory = true;
      int numberOfSegments = waypoints.size() - 1;

      for (int i = 0; i < numberOfSegments; i++)
      {
         EuclideanTrajectoryPoint simpleStart = waypoints.get(i);
         EuclideanTrajectoryPoint simpleEnd = waypoints.get(i + 1);

         EuclideanWaypointBasics startWaypoint = simpleStart;
         EuclideanWaypointBasics endWaypoint = simpleEnd;
         if (startWaypoint.containsNaN() || endWaypoint.containsNaN())
         {
            successfulTrajectory = false;
            break;
         }
         subTrajectory.add().setCubic(simpleStart.getTime() - startTime, simpleEnd.getTime() - startTime, startWaypoint.getPosition(),
                                      startWaypoint.getLinearVelocity(), endWaypoint.getPosition(), endWaypoint.getLinearVelocity());
      }

      if (!successfulTrajectory)
      {
         PrintTools.warn("Something in the waypoint trajectory was invalid. Ignoring for the phase " + walkingTrajectoryType + ".");
         subTrajectory.reset();
         subTrajectory.add().setConstant(0.0, subTrajectoryDuration, zeroPoint);
      }
   }

   private boolean calculateAngularMomentumForPhase(WalkingTrajectoryType walkingTrajectoryType)
   {
      if (walkingTrajectoryType.equals(WalkingTrajectoryType.SWING))
      {
         return planSwingAngularMomentum.getBooleanValue();
      }
      else
      {
         return planTransferAngularMomentum.getBooleanValue();
      }
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean initialTransfer, boolean standing)
   {
      boolean atStop = initialTransfer && standing;
      this.updateTrajectories.set(!isInPhase(WalkingTrajectoryType.TRANSFER) || atStop != this.atAStop.getBooleanValue());
      if (updateTrajectories.getBooleanValue())
      {
         computeTrajectories(WalkingTrajectoryType.TRANSFER);
      }
      this.atAStop.set(atStop);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport()
   {
      this.updateTrajectories.set(!isInPhase(WalkingTrajectoryType.SWING));
      if (updateTrajectories.getBooleanValue())
      {
         computeTrajectories(WalkingTrajectoryType.SWING);
      }
      atAStop.set(false);
   }

   @Override
   public List<AngularMomentumTrajectory> getTransferAngularMomentumTrajectories()
   {
      if (planTransferAngularMomentum.getBooleanValue())
         return transferTrajectories;
      else
         return null;
   }

   @Override
   public List<AngularMomentumTrajectory> getSwingAngularMomentumTrajectories()
   {
      if (planSwingAngularMomentum.getBooleanValue())
         return swingTrajectories;
      else
         return null;
   }

   public boolean hasReferenceTrajectory()
   {
      return !momentumTrajectoryHandler.isEmpty() && momentumTrajectoryHandler.isWithinInterval(time.getDoubleValue());
   }

   public void reset()
   {
      // This is so the next time a trajectory arrives during a standing state the at a stop check will trigger the packing of the trajectory.
      atAStop.set(true);
   }
}
