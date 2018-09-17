package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPlanningTools;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class CommandBasedAngularMomentumTrajectoryGenerator implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final int waypointsPerWalkingPhase = 12;
   private static final int numberOfTrajectoryCoefficients = 4;
   private static final FramePoint3D zeroPoint = new FramePoint3D();

   static
   {
      zeroPoint.setToZero();
   }

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final MomentumTrajectoryHandler momentumTrajectoryHandler;
   private final YoDouble time;
   private final YoInteger numberOfRegisteredFootsteps;
   private final YoDouble[] commandedAngularMomentum = new YoDouble[3];
   private final YoDouble[] commandedCoMTorque = new YoDouble[3];

   private final RecyclingArrayList<SimpleEuclideanTrajectoryPoint> waypoints = new RecyclingArrayList<>(waypointsPerWalkingPhase, SimpleEuclideanTrajectoryPoint::new);
   private final List<AngularMomentumTrajectory> transferTrajectories;
   private final List<AngularMomentumTrajectory> swingTrajectories;

   private double initialTime;
   private AngularMomentumTrajectory activeTrajectory;
   private List<CoPPointsInFoot> copLocations;
   private SmoothCMPPlannerParameters smoothCMPPlannerParameters;

   private final YoBoolean planSwingAngularMomentum;
   private final YoBoolean planTransferAngularMomentum;

   private final FrameVector3D desiredAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredTorque = new FrameVector3D();
   private final FrameVector3D desiredRotatum = new FrameVector3D();

   public CommandBasedAngularMomentumTrajectoryGenerator(MomentumTrajectoryHandler momentumTrajectoryHandler, YoDouble time, int maxNumberOfStepsToConsider, YoVariableRegistry parentRegistry)
   {
      String namePrefix = "commandedAngMomentum";
      this.momentumTrajectoryHandler = momentumTrajectoryHandler;
      this.time = time;
      this.numberOfRegisteredFootsteps = new YoInteger(namePrefix + "NumberOfRegisteredFootsteps", registry);
      this.planSwingAngularMomentum = new YoBoolean("PlanSwingAngularMomentumWithCommand", registry);
      this.planTransferAngularMomentum = new YoBoolean("PlanTransferAngularMomentumWithCommand", registry);

      transferTrajectories = new ArrayList<>(maxNumberOfStepsToConsider + 1);
      swingTrajectories = new ArrayList<>(maxNumberOfStepsToConsider);
      for (int i = 0; i < maxNumberOfStepsToConsider; i++)
      {
         transferTrajectories.add(new AngularMomentumTrajectory(waypointsPerWalkingPhase - 1, numberOfTrajectoryCoefficients));
         swingTrajectories.add(new AngularMomentumTrajectory(waypointsPerWalkingPhase - 1, numberOfTrajectoryCoefficients));
      }
      transferTrajectories.add(new AngularMomentumTrajectory(waypointsPerWalkingPhase - 1, numberOfTrajectoryCoefficients));

      for (int i = 0; i < Axis.values.length; i++)
      {
         commandedAngularMomentum[i] = new YoDouble("commandedAngularMomentum" + Axis.values[i], registry);
         commandedCoMTorque[i] = new YoDouble("commandedCoMTorque" + Axis.values[i], registry);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void initializeParameters(SmoothCMPPlannerParameters smoothCMPPlannerParameters, double totalMass, double gravityZ)
   {
      this.smoothCMPPlannerParameters = smoothCMPPlannerParameters;
      this.planSwingAngularMomentum.set(smoothCMPPlannerParameters.planSwingAngularMomentum());
      this.planTransferAngularMomentum.set(smoothCMPPlannerParameters.planTransferAngularMomentum());
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

      for (int i = 0; i < Axis.values.length; i++)
      {
         commandedAngularMomentum[i].set(desiredAngularMomentum.getElement(i));
         commandedCoMTorque[i].set(desiredTorque.getElement(i));
      }
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
      // don't recompute if you're already in this phase
      if(isInPhase(startingTrajectoryType))
      {
         return;
      }

      momentumTrajectoryHandler.clearPointsInPast();

      int stepIndex = 0;
      double currentTime = time.getDoubleValue();
      double accumulatedTime = 0.0;
      CoPPointName entryCoPName = smoothCMPPlannerParameters.getEntryCoPName();
      int numberOfStepsToConsider = smoothCMPPlannerParameters.getNumberOfFootstepsToConsider();

      if(startingTrajectoryType.equals(WalkingTrajectoryType.SWING))
      {
         // at the start of swing, for some reason the copLocation corresponding to the upcoming swing is at index 1
         CoPPointsInFoot pointsInFoot = copLocations.get(1);

         double swingPhaseDuration = getPhaseDuration(WalkingTrajectoryType.SWING, pointsInFoot, entryCoPName);
         momentumTrajectoryHandler.getAngularMomentumTrajectory(currentTime, currentTime + swingPhaseDuration, waypointsPerWalkingPhase, waypoints);
         setSubTrajectory(swingPhaseDuration, swingTrajectories.get(0), startingTrajectoryType);
         accumulatedTime += swingPhaseDuration;

         // the first transfer copLocation is at index 2 at the start of swing. otherwise it's at index 0
         stepIndex++;
      }

      int numberOfSteps = Math.min(numberOfRegisteredFootsteps.getIntegerValue(), numberOfStepsToConsider);
      for ( ; stepIndex < numberOfSteps; stepIndex++)
      {
         CoPPointsInFoot pointsInFoot = copLocations.get(stepIndex + 1);

         double transferPhaseDuration = getPhaseDuration(WalkingTrajectoryType.TRANSFER, pointsInFoot, entryCoPName);
         momentumTrajectoryHandler.getAngularMomentumTrajectory(currentTime + accumulatedTime, currentTime + accumulatedTime + transferPhaseDuration, waypointsPerWalkingPhase, waypoints);
         setSubTrajectory(transferPhaseDuration, transferTrajectories.get(stepIndex), WalkingTrajectoryType.TRANSFER);
         accumulatedTime += transferPhaseDuration;

         double swingPhaseDuration = getPhaseDuration(WalkingTrajectoryType.SWING, pointsInFoot, entryCoPName);
         momentumTrajectoryHandler.getAngularMomentumTrajectory(currentTime + accumulatedTime, currentTime + accumulatedTime + swingPhaseDuration, waypointsPerWalkingPhase, waypoints);
         setSubTrajectory(swingPhaseDuration, swingTrajectories.get(stepIndex), WalkingTrajectoryType.TRANSFER);
         accumulatedTime += swingPhaseDuration;
      }

      // handle terminal transfer
      CoPPointsInFoot pointsInFoot = copLocations.get(stepIndex + 1);
      double finalTransferDuration = getPhaseDuration(WalkingTrajectoryType.TRANSFER, pointsInFoot, entryCoPName);
      momentumTrajectoryHandler.getAngularMomentumTrajectory(currentTime + accumulatedTime, currentTime + accumulatedTime + finalTransferDuration, waypointsPerWalkingPhase, waypoints);
      setSubTrajectory(finalTransferDuration, transferTrajectories.get(numberOfSteps), WalkingTrajectoryType.TRANSFER);
   }

   private boolean isInPhase(WalkingTrajectoryType phase)
   {
      if(activeTrajectory == null)
      {
         return false;
      }
      else if(phase.equals(WalkingTrajectoryType.SWING) && activeTrajectory == swingTrajectories.get(0))
      {
         return true;
      }
      else if(phase.equals(WalkingTrajectoryType.TRANSFER) && activeTrajectory == transferTrajectories.get(0))
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

      if(trajectoryType.equals(WalkingTrajectoryType.TRANSFER))
      {
         int finalTransferIndex = (entryPointIndex == -1) ? pointsInFoot.getNumberOfCoPPoints() - 1 : entryPointIndex;
         for (int i = 0; i <= finalTransferIndex; i++)
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

   private void setSubTrajectory(double subTrajectoryDuration, AngularMomentumTrajectory subTrajectory, WalkingTrajectoryType walkingTrajectoryType)
   {
      subTrajectory.reset();

      if(waypoints.isEmpty() || !calculateAngularMomentumForPhase(walkingTrajectoryType))
      {
         subTrajectory.add().setConstant(0.0, subTrajectoryDuration, zeroPoint);
         return;
      }

      boolean successfulTrajectory = true;
      int numberOfSegments = Math.min(waypointsPerWalkingPhase, waypoints.size() - 1);
      for (int i = 0; i < numberOfSegments; i++)
      {
         double subTrajectoryStartTime = (subTrajectoryDuration * i) / (waypointsPerWalkingPhase - 1);
         double subTrajectoryEndTime = (subTrajectoryDuration * (i + 1)) / (waypointsPerWalkingPhase - 1);
         EuclideanWaypoint startWaypoint = this.waypoints.get(i).getEuclideanWaypoint();
         EuclideanWaypoint endWaypoint = this.waypoints.get(i + 1).getEuclideanWaypoint();
         if (startWaypoint.containsNaN() || endWaypoint.containsNaN())
         {
            successfulTrajectory = false;
            break;
         }
         subTrajectory.add().setCubic(subTrajectoryStartTime, subTrajectoryEndTime, startWaypoint.getPosition(), startWaypoint.getLinearVelocity(),
                                      endWaypoint.getPosition(), endWaypoint.getLinearVelocity());
      }

      if (!successfulTrajectory)
      {
         subTrajectory.reset();
         subTrajectory.add().setConstant(0.0, subTrajectoryDuration, zeroPoint);
      }
   }

   private boolean calculateAngularMomentumForPhase(WalkingTrajectoryType walkingTrajectoryType)
   {
      if(walkingTrajectoryType.equals(WalkingTrajectoryType.SWING))
      {
         return planSwingAngularMomentum.getBooleanValue();
      }
      else
      {
         return planTransferAngularMomentum.getBooleanValue();
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
   public List<AngularMomentumTrajectory> getTransferAngularMomentumTrajectories()
   {
      return transferTrajectories;
   }

   @Override
   public List<AngularMomentumTrajectory> getSwingAngularMomentumTrajectories()
   {
      return swingTrajectories;
   }

   public boolean hasReferenceTrajectory()
   {
      return !momentumTrajectoryHandler.isEmpty() && momentumTrajectoryHandler.isWithinInterval(time.getDoubleValue());
   }
}
