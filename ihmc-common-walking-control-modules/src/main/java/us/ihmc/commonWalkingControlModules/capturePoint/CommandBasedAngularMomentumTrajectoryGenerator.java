package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.AngularMomentumEstimationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumSplineType;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumTrajectoryGeneratorInterface;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumTrajectoryInterface;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class CommandBasedAngularMomentumTrajectoryGenerator implements AngularMomentumTrajectoryGeneratorInterface
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final WalkingMessageHandler momentumWaypointSource;
   private final YoDouble time;

   private final YoInteger numberOfWaypointsToUseForTransfer;
   private final YoInteger numberOfWaypointsToUseForSwing;
   private final YoEnum<AngularMomentumSplineType> trajectoryType;
   private final YoInteger numberOfFootstepsToPlan;
   private RecyclingArrayList<TrajectoryPoint3D> waypoints;
   private final List<YoDouble> transferDurations;
   private final List<YoDouble> swingDurations;

   private final List<AngularMomentumTrajectory> transferTrajectories;
   private final List<AngularMomentumTrajectory> swingTrajectories;

   private double initialTime;
   private AngularMomentumTrajectory activeTrajectory;
   private FrameVector3D desiredAngularMomentum;
   private FrameVector3D desiredTorque;
   private FrameVector3D desiredRotatum;
   private FramePoint3D tempFramePoint1 = new FramePoint3D(worldFrame);
   private FramePoint3D tempFramePoint2 = new FramePoint3D(worldFrame);
   private double planTime;

   public CommandBasedAngularMomentumTrajectoryGenerator(String namePrefix, SmoothCMPPlannerParameters smoothCMPPlannerParameters,
                                                         WalkingMessageHandler handler, YoDouble time, YoVariableRegistry registry)
   {
      this.momentumWaypointSource = handler;
      this.time = time;

      AngularMomentumEstimationParameters trajectoryGenerationParameters = smoothCMPPlannerParameters.getAngularMomentumEstimationParameters();
      this.numberOfWaypointsToUseForTransfer = new YoInteger(namePrefix + "NumberOfSampledWaypointsForTransfer", registry);
      this.numberOfWaypointsToUseForTransfer.set(trajectoryGenerationParameters.getNumberOfPointsToSampleForTransfer());
      this.numberOfWaypointsToUseForSwing = new YoInteger(namePrefix + "NumberOfSampledWaypintsForSwing", registry);
      this.numberOfWaypointsToUseForSwing.set(trajectoryGenerationParameters.getNumberOfPointsToSampleForSwing());
      this.trajectoryType = new YoEnum<>(namePrefix + "SplineType", registry, AngularMomentumSplineType.class);
      this.trajectoryType.set(trajectoryGenerationParameters.getSplineType());

      this.numberOfFootstepsToPlan = new YoInteger(namePrefix + "NumberOfFootstepsToPlan", registry);
      this.numberOfFootstepsToPlan.set(smoothCMPPlannerParameters.getNumberOfFootstepsToConsider());

      this.waypoints = new RecyclingArrayList<>(Math.max(numberOfWaypointsToUseForSwing.getIntegerValue(), numberOfWaypointsToUseForTransfer.getIntegerValue()),
                                                TrajectoryPoint3D.class);
      this.waypoints.clear();

      this.transferDurations = new ArrayList<>(numberOfFootstepsToPlan.getIntegerValue() + 1);
      this.swingDurations = new ArrayList<>(numberOfFootstepsToPlan.getIntegerValue());
      this.transferTrajectories = new ArrayList<>(numberOfFootstepsToPlan.getIntegerValue() + 1);
      this.swingTrajectories = new ArrayList<>(numberOfFootstepsToPlan.getIntegerValue());

      for (int i = 0; i < numberOfFootstepsToPlan.getIntegerValue() + 1; i++)
      {
         AngularMomentumTrajectory transferTrajectory = new AngularMomentumTrajectory(worldFrame, numberOfWaypointsToUseForTransfer.getIntegerValue() - 1,
                                                                                      trajectoryType.getEnumValue().getNumberOfCoefficients());
         YoDouble transferDuration = new YoDouble(namePrefix + "TransferDurationStep" + i, registry);
         transferTrajectories.add(transferTrajectory);
         transferDurations.add(transferDuration);
      }
      for (int i = 0; i < numberOfFootstepsToPlan.getIntegerValue(); i++)
      {
         AngularMomentumTrajectory swingTrajectory = new AngularMomentumTrajectory(worldFrame, numberOfWaypointsToUseForSwing.getIntegerValue() - 1,
                                                                                   trajectoryType.getEnumValue().getNumberOfCoefficients());
         YoDouble swingDuration = new YoDouble(namePrefix + "SwingDurationStep" + i, registry);
         swingTrajectories.add(swingTrajectory);
         swingDurations.add(swingDuration);
      }
   }

   public void setStepDurations(List<Double> transferDurations, List<Double> swingDurations)
   {
      int index;
      for (index = 0; index < transferDurations.size(); index++)
         this.transferDurations.get(index).set(transferDurations.get(index));
      for (; index < this.transferDurations.size(); index++)
         this.transferDurations.get(index).setToNaN();

      for (index = 0; index < swingDurations.size(); index++)
         this.swingDurations.get(index).set(swingDurations.get(index));
      for (; index < this.swingDurations.size(); index++)
         this.swingDurations.get(index).setToNaN();
   }

   @Override
   public void updateListeners()
   {
   }

   @Override
   public void createVisualizerForConstantAngularMomentum(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
   }

   @Override
   public void clear()
   {
      for (int i = 0; i < numberOfFootstepsToPlan.getIntegerValue(); i++)
      {
         swingDurations.get(i).setToNaN();
         swingTrajectories.get(i).reset();
      }
      for (int i = 0; i < numberOfFootstepsToPlan.getIntegerValue() + 1; i++)
      {
         transferDurations.get(i).setToNaN();
         transferTrajectories.get(i).reset();
      }
   }

   @Override
   public void update(double currentTime)
   {
      if (activeTrajectory != null)
         activeTrajectory.update(currentTime - initialTime, desiredAngularMomentum, desiredTorque, desiredRotatum);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack, FrameVector3D desiredTorqueToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
      desiredTorqueToPack.setIncludingFrame(desiredTorque);
   }

   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack, FrameVector3D desiredTorqueToPack, FrameVector3D desiredRotatumToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
      desiredTorqueToPack.setIncludingFrame(desiredTorque);
      desiredRotatumToPack.setIncludingFrame(desiredRotatum);
   }

   @Override
   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
   }

   @Override
   public void initializeForDoubleSupport(double currentTime, boolean isStanding)
   {
      initialTime = currentTime;

      if (!isStanding)
         activeTrajectory = transferTrajectories.get(0);
   }

   @Override
   public void initializeForSingleSupport(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = swingTrajectories.get(0);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop)
   {
      computeTrajectories();
   }

   public void computeTrajectories()
   {
      planTime = time.getDoubleValue();
      for (int i = 0; i < numberOfFootstepsToPlan.getIntegerValue() + 1; i++)
      {
         if (i >= transferDurations.size() || transferDurations.get(i).isNaN())
            break;
         planTime = computeTransferTrajectoryForFootstep(i, planTime);
         if (i >= swingDurations.size() || swingDurations.get(i).isNaN())
            break;
         planTime = computeSwingTrajectoryForFootstep(i, planTime);
      }
   }

   private double computeTransferTrajectoryForFootstep(int footstepIndex, double startTime)
   {
      computeAngularMomentumTrajectoryForState(footstepIndex, startTime, transferDurations.get(footstepIndex).getDoubleValue(),
                                               numberOfWaypointsToUseForTransfer.getIntegerValue());
      return startTime + transferDurations.get(footstepIndex).getDoubleValue();
   }

   private double computeSwingTrajectoryForFootstep(int footstepIndex, double startTime)
   {
      computeAngularMomentumTrajectoryForState(footstepIndex, startTime, swingDurations.get(footstepIndex).getDoubleValue(),
                                               numberOfWaypointsToUseForSwing.getIntegerValue());
      return startTime + swingDurations.get(footstepIndex).getDoubleValue();
   }

   private void computeAngularMomentumTrajectoryForState(int footstepIndex, double initialTime, double stateDuration, int numberOfSamples)
   {
      momentumWaypointSource.getAngularMomentumTrajectory(initialTime, initialTime + stateDuration, numberOfSamples, waypoints);
      for (int j = 0; j < numberOfSamples - 1; j++)
      {
         tempFramePoint1.set(waypoints.get(j).getPosition());
         tempFramePoint2.set(waypoints.get(j + 1).getPosition());
         transferTrajectories.get(footstepIndex).set(0.0, stateDuration, tempFramePoint1, tempFramePoint2);
      }
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport()
   {
      computeTrajectories();
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

}
