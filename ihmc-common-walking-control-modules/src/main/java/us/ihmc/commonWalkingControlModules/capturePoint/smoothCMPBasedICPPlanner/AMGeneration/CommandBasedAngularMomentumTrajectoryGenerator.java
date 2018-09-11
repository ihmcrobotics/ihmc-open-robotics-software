package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPlanningTools;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class CommandBasedAngularMomentumTrajectoryGenerator implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final int waypointsPerWalkingPhase = 30;
   private static final int maxNumberOfStepsToConsider = 4;
   private static final int numberOfTrajectoryCoefficients = 4;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final WalkingMessageHandler walkingMessageHandler;
   private final YoDouble time;
   private final YoInteger numberOfStepsToConsider, numberOfRegisteredFootsteps;

   private final RecyclingArrayList<SimpleEuclideanTrajectoryPoint> waypoints = new RecyclingArrayList<>(waypointsPerWalkingPhase, SimpleEuclideanTrajectoryPoint::new);
   private final RecyclingArrayList<AngularMomentumTrajectory> transferTrajectories;
   private final RecyclingArrayList<AngularMomentumTrajectory> swingTrajectories;

   private double initialTime;
   private AngularMomentumTrajectory activeTrajectory;
   private List<CoPPointsInFoot> copLocations;
   private CoPPointName entryCoPName;

   private final FrameVector3D desiredAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredTorque = new FrameVector3D();
   private final FrameVector3D desiredRotatum = new FrameVector3D();

   public CommandBasedAngularMomentumTrajectoryGenerator(WalkingMessageHandler walkingMessageHandler, YoDouble time, YoVariableRegistry parentRegistry)
   {
      String namePrefix = "commandedAngMomentum";
      this.walkingMessageHandler = walkingMessageHandler;
      this.time = time;
      this.numberOfStepsToConsider = new YoInteger(namePrefix + "NumberOfStepsToConsider", registry);
      this.numberOfRegisteredFootsteps = new YoInteger(namePrefix + "NumberOfRegisteredFootsteps", registry);

      transferTrajectories = new RecyclingArrayList<>(maxNumberOfStepsToConsider, () -> new AngularMomentumTrajectory(waypointsPerWalkingPhase - 1, numberOfTrajectoryCoefficients));
      swingTrajectories = new RecyclingArrayList<>(maxNumberOfStepsToConsider, () -> new AngularMomentumTrajectory(waypointsPerWalkingPhase - 1, numberOfTrajectoryCoefficients));

      parentRegistry.addChild(registry);
   }

   @Override
   public void initializeParameters(SmoothCMPPlannerParameters smoothCMPPlannerParameters, double totalMass, double gravityZ)
   {
      entryCoPName = smoothCMPPlannerParameters.getEntryCoPName();
      numberOfStepsToConsider.set(smoothCMPPlannerParameters.getNumberOfFootstepsToConsider());
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
      MomentumTrajectoryHandler momentumTrajectoryHandler = walkingMessageHandler.getMomentumTrajectoryHandler();
      momentumTrajectoryHandler.clearPointsInPast();

      swingTrajectories.clear();
      transferTrajectories.clear();

      int copLocationIndex = 0;
      double currentTime = time.getDoubleValue();
      double accumulatedTime = 0.0;

      if(startingTrajectoryType.equals(WalkingTrajectoryType.SWING))
      {
         // at the start of swing, for some reason the copLocation corresponding to the upcoming swing is at index 1
         copLocationIndex = 1;
         CoPPointsInFoot pointsInFoot = copLocations.get(copLocationIndex);

         double swingPhaseDuration = getPhaseDuration(WalkingTrajectoryType.SWING, pointsInFoot, entryCoPName);
         momentumTrajectoryHandler.getAngularMomentumTrajectory(currentTime, currentTime + swingPhaseDuration, waypointsPerWalkingPhase, waypoints);
         setSubTrajectory(swingPhaseDuration, swingTrajectories.add());
         accumulatedTime += swingPhaseDuration;

         // the first transfer copLocation is at index 2 at the start of swing (i.e. inside this block). otherwise it's at index 0
         copLocationIndex++;
      }

      int numberOfSteps = Math.min(numberOfRegisteredFootsteps.getIntegerValue(), numberOfStepsToConsider.getIntegerValue());
      for (int i = 0; i < numberOfSteps; i++, copLocationIndex++)
      {
         CoPPointsInFoot pointsInFoot = copLocations.get(copLocationIndex);

         double transferPhaseDuration = getPhaseDuration(WalkingTrajectoryType.TRANSFER, pointsInFoot, entryCoPName);
         momentumTrajectoryHandler.getAngularMomentumTrajectory(currentTime + accumulatedTime, currentTime + accumulatedTime + transferPhaseDuration, waypointsPerWalkingPhase, waypoints);
         setSubTrajectory(transferPhaseDuration, transferTrajectories.add());
         accumulatedTime += transferPhaseDuration;

         double swingPhaseDuration = getPhaseDuration(WalkingTrajectoryType.SWING, pointsInFoot, entryCoPName);
         momentumTrajectoryHandler.getAngularMomentumTrajectory(currentTime + accumulatedTime, currentTime + accumulatedTime + swingPhaseDuration, waypointsPerWalkingPhase, waypoints);
         setSubTrajectory(swingPhaseDuration, swingTrajectories.add());
         accumulatedTime += swingPhaseDuration;
      }
   }

   /**
    * The trajectory points in CoPPointsInFoot go transfer then swing, with "entryCoPName" being the last transfer point.
    */
   private static double getPhaseDuration(WalkingTrajectoryType trajectoryType, CoPPointsInFoot pointsInFoot, CoPPointName entryCoPName)
   {
      double phaseTime = 0.0;
      int entryPointIndex = CoPPlanningTools.getCoPPointIndex(pointsInFoot.getCoPPointList(), entryCoPName);

      if(trajectoryType.equals(WalkingTrajectoryType.TRANSFER))
      {
         for (int i = 0; i <= entryPointIndex; i++)
         {
            phaseTime += pointsInFoot.get(i).getTime();
         }
      }
      else
      {
         for(int i = entryPointIndex + 1; i < pointsInFoot.getNumberOfCoPPoints(); i++)
         {
            phaseTime += pointsInFoot.get(i).getTime();
         }
      }

      return phaseTime;
   }

   private void setSubTrajectory(double subTrajectoryDuration, AngularMomentumTrajectory subTrajectory)
   {
      subTrajectory.reset();
      for (int i = 0; i < waypointsPerWalkingPhase - 1; i++)
      {
         double subTrajectoryStartTime = (subTrajectoryDuration * i) / (waypointsPerWalkingPhase - 1);
         double subTrajectoryEndTime = (subTrajectoryDuration * (i + 1)) / (waypointsPerWalkingPhase - 1);
         EuclideanWaypoint startWaypoint = this.waypoints.get(i).getEuclideanWaypoint();
         EuclideanWaypoint endWaypoint = this.waypoints.get(i + 1).getEuclideanWaypoint();
         subTrajectory.add().setCubic(subTrajectoryStartTime, subTrajectoryEndTime, startWaypoint.getPosition(), startWaypoint.getLinearVelocity(),
                                        endWaypoint.getPosition(), endWaypoint.getLinearVelocity());

      }
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop)
   {
      computeTrajectories(WalkingTrajectoryType.TRANSFER);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport()
   {
      computeTrajectories(WalkingTrajectoryType.SWING);
   }

   @Override
   public List<? extends AngularMomentumTrajectoryInterface> getTransferAngularMomentumTrajectories()
   {
      return transferTrajectories;
   }

   @Override
   public List<? extends AngularMomentumTrajectoryInterface> getSwingAngularMomentumTrajectories()
   {
      return swingTrajectories;
   }

   public boolean hasReferenceTrajectory()
   {
      MomentumTrajectoryHandler momentumTrajectoryHandler = walkingMessageHandler.getMomentumTrajectoryHandler();
      return !momentumTrajectoryHandler.isEmpty() && momentumTrajectoryHandler.isWithinInterval(time.getDoubleValue());
   }
}
