package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class AngularMomentumTrajectoryGenerator implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final int maxNumberOfTrajectoryCoefficients = 4;

   private final YoEnum<AngularMomentumTrajectoryInput> currentTrajectoryType;
   private final YoInteger numberOfFootstepsToConsider;
   private final YoInteger maxNumberOfWaypoints;
   private final double defaultSwingDuration;
   private final double defaultTransferDuration;

   private final List<CoPPointsInFoot> upcomingCoPsInFootsteps;

   private final List<SwingAngularMomentumTrajectory> swingAngularMomentumTrajectories;
   private final List<TransferAngularMomentumTrajectory> transferAngularMomentumTrajectories;

   private final RecyclingArrayList<TrajectoryPoint3D> upcomingWaypointList; // used for calculating the angular momentum trajectory

   private final FrameVector desiredAngularMomentum = new FrameVector();
   private final FrameVector desiredTorque = new FrameVector();
   private final FrameVector desiredRotatum = new FrameVector();

   private final YoFrameTrajectory3D comTrajectory;
   private final YoFrameTrajectory3D swingFootTrajectory;
   private final YoFrameTrajectory3D swingFootVelocity;
   private final YoFrameTrajectory3D angularMomentumTrajectory;

   private AngularMomentumTrajectoryInterface activeTrajectory;
   private double initialTime;

   public AngularMomentumTrajectoryGenerator(String namePrefix, AngularMomentumEstimationParameters angularMomentumParameters,
                                             YoVariableRegistry parentRegistry)
   {
      this.currentTrajectoryType = new YoEnum<>(namePrefix + "AngularMomentumPlanningMode", parentRegistry, AngularMomentumTrajectoryInput.class);
      this.numberOfFootstepsToConsider = new YoInteger(namePrefix + "AngularMomentumPlanMaxFootsteps", registry);
      this.numberOfFootstepsToConsider.set(angularMomentumParameters.getNumberOfFootstepsToConsider());
      this.maxNumberOfWaypoints = new YoInteger(namePrefix + "", registry);
      this.maxNumberOfWaypoints.set(angularMomentumParameters.getMaximumNumberOfAngularMomentumPointsToPlan());

      this.swingAngularMomentumTrajectories = new ArrayList<>(numberOfFootstepsToConsider.getIntegerValue());
      this.transferAngularMomentumTrajectories = new ArrayList<>(numberOfFootstepsToConsider.getIntegerValue());
      this.upcomingCoPsInFootsteps = new ArrayList<>(numberOfFootstepsToConsider.getIntegerValue() + 2);
      this.upcomingWaypointList = new RecyclingArrayList<>(TrajectoryPoint3D.class);
      this.defaultSwingDuration = CoPPlanningTools.getSwingStateDuration(angularMomentumParameters.getCoPPointList(),
                                                                         angularMomentumParameters.getSegmentTimes(),
                                                                         angularMomentumParameters.getEntryCoPName(),
                                                                         angularMomentumParameters.getExitCoPName());
      this.defaultTransferDuration = CoPPlanningTools.getStepDuration(angularMomentumParameters.getCoPPointList(), angularMomentumParameters.getSegmentTimes())
            - this.defaultSwingDuration;

      ReferenceFrame[] referenceFrames = {worldFrame};
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         SwingAngularMomentumTrajectory swingTrajectory = new SwingAngularMomentumTrajectory(namePrefix + "Footstep", i, registry, worldFrame,
                                                                                             maxNumberOfWaypoints.getIntegerValue());
         this.swingAngularMomentumTrajectories.add(swingTrajectory);
         TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(namePrefix + "Footstep", i, registry, worldFrame,
                                                                                                      maxNumberOfWaypoints.getIntegerValue());
         this.transferAngularMomentumTrajectories.add(transferTrajectory);
         CoPPointsInFoot copLocations = new CoPPointsInFoot(i, referenceFrames, registry);
         upcomingCoPsInFootsteps.add(copLocations);
      }

      this.comTrajectory = new YoFrameTrajectory3D("EstimatedCoMTrajectory", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingFootTrajectory = new YoFrameTrajectory3D("EstimatedCoMTrajectory", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.swingFootVelocity = new YoFrameTrajectory3D("EstimatedCoMTrajectory", maxNumberOfTrajectoryCoefficients, worldFrame, registry);
      this.angularMomentumTrajectory = new YoFrameTrajectory3D("EstiamtedAngularMomentumTrajectory", 7, worldFrame, registry);
   }

   private void setInput(AngularMomentumTrajectoryInput inputType)
   {
      this.currentTrajectoryType.set(inputType);
   }

   public AngularMomentumTrajectoryInput getInputType()
   {
      return currentTrajectoryType.getEnumValue();
   }

   @Override
   public void updateListeners()
   {
      // TODO Auto-generated method stub      
   }

   @Override
   public void createVisualizerForConstantAngularMomentum(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void clear()
   {
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         swingAngularMomentumTrajectories.get(i).reset();
         transferAngularMomentumTrajectories.get(i).reset();
      }
      upcomingWaypointList.clear();
   }

   @Override
   public void addFootstepCoPsToPlan(List<CoPPointsInFoot> copLocations)
   {
      for (int i = 0; i < copLocations.size(); i++)
         upcomingCoPsInFootsteps.get(i).setIncludingFrame(copLocations.get(i));
   }

   @Override
   public void addAngularMomentumWaypointsToPlan(List<AngularMomentumTrajectoryPoint> waypointList)
   {
      //TODO 
      for (int i = 0; i < waypointList.size(); i++)
      {
         //upcomingWaypointList.add().set(waypointList.get(i));
      }
   }

   @Override
   public void addAngularMomentumWaypointToPlan(AngularMomentumTrajectoryPoint waypoint)
   {
      //TODO 
      //upcomingWaypointList.add().set(waypoint);
   }

   @Override
   public void update(double currentTime)
   {
      double timeInState = currentTime - initialTime;

      if (activeTrajectory != null)
         activeTrajectory.update(timeInState, desiredAngularMomentum, desiredTorque);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack, FrameVector desiredTorqueToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
   }

   public void getDesiredAngularMomentum(FrameVector desiredAngMomToPack, FrameVector desiredTorqueToPack, FrameVector desiredRotatumToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
      desiredRotatumToPack.set(desiredRotatum);
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

   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack, YoFrameVector desiredRotatumToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
      desiredRotatumToPack.set(desiredRotatum);
   }

   @Override
   public void initializeForTransfer(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = transferAngularMomentumTrajectories.get(0);
   }

   @Override
   public void initializeForSwing(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = swingAngularMomentumTrajectories.get(0);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      if (currentTrajectoryType.equals(AngularMomentumTrajectoryInput.ANGULAR_MOMENTUM_WAYPOINTS))
      {
         computeForDoubleSupportFromAngularMomentumWaypoints();
      }
      else
      {
         computeForDoubleSupportFromCoPWayPoints();
      }
   }

   private void computeForDoubleSupportFromAngularMomentumWaypoints()
   {
      //TODO complete this method 
      return;
   }

   private void computeForDoubleSupportFromCoPWayPoints()
   {
      int numberOfFootstepToPlan = Math.min(numberOfFootstepsToConsider.getIntegerValue(), upcomingCoPsInFootsteps.size());
      for (int index = 0; index < numberOfFootstepToPlan; index++)
         computeTransferAngularMomentumApproximationForFootstep(index);
   }

   private void computeTransferAngularMomentumApproximationForFootstep(int footIndex)
   {
      TransferAngularMomentumTrajectory transferTrajectory = transferAngularMomentumTrajectories.get(footIndex);
      CoPPointsInFoot previousFootstepCoPPlan = upcomingCoPsInFootsteps.get(footIndex);
      //      p1 = upcomingCoPsInFootsteps.get(footIndex).get(maxNumberOfWaypointsPerFootstep.getIntegerValue() -1).getFrameTuple();
      //      p2 = upcomingCoPsInFootsteps.get(footIndex+1).get(0).getFrameTuple();
      //      p3 = upcomingCoPsInFootsteps.get(footIndex+1).get(maxNumberOfWaypointsPerFootstep.getIntegerValue() -1).getFrameTuple();
      //      p4 = upcomingCoPsInFootsteps.get(footIndex+2).get(0).getFrameTuple();
      //      comTrajectory.setCubicBezier(p1, p2, p3, p4);
      //      swingFootTrajectory.setConstant(z0);
      swingFootTrajectory.subtractByTrimming(comTrajectory);
      swingFootTrajectory.getDerivative(swingFootVelocity);
      TrajectoryMathTools.crossProductByTrimming(angularMomentumTrajectory, swingFootTrajectory, swingFootVelocity);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport(RobotSide supportSide)
   {
      if (currentTrajectoryType.equals(AngularMomentumTrajectoryInput.ANGULAR_MOMENTUM_WAYPOINTS))
      {
         computeForSingleSupportFromAngularMomentumWaypoints(supportSide);
      }
      else
      {
         computeForSingleSupportFromCoPWayPoints();
      }
   }

   private void computeForSingleSupportFromAngularMomentumWaypoints(RobotSide supportSide)
   {
      // TODO Auto-generated method stub
      return;
   }

   private void computeForSingleSupportFromCoPWayPoints()
   {
      int numberOfFootstepToPlan = Math.min(numberOfFootstepsToConsider.getIntegerValue(), upcomingCoPsInFootsteps.size());
      for (int index = 0; index < numberOfFootstepToPlan; index++)
         computeSwingAngularMomentumApproximationForFootstep(index);
   }

   private void computeSwingAngularMomentumApproximationForFootstep(int footIndex)
   {
      TransferAngularMomentumTrajectory transferTrajectory = transferAngularMomentumTrajectories.get(footIndex);
      //      p1 = upcomingExitCoPList.get(footIndex);
      //      p2 = upcomingEntryCoPList.get(footIndex + 1);
      //      p3 = upcomingExitCoPList.get(footIndex + 1);
      //      p4 = upcomingEntryCoPList.get(footIndex + 2);
      //      comTrajectory.setCubicBezier(p1, p2, p3, p4);
      //      swingFootTrajectory.setCubic(z0, zFinal);
      swingFootTrajectory.subtractByTrimming(comTrajectory);
      swingFootTrajectory.getDerivative(swingFootVelocity);
      angularMomentumTrajectory.crossProduct(swingFootTrajectory, swingFootVelocity);
   }

   @Override
   public List<TrajectoryPoint3D> getWaypoints()
   {
      return upcomingWaypointList;
   }

   @Override
   public List<? extends AngularMomentumTrajectoryInterface> getTransferCoPTrajectories()
   {
      return transferAngularMomentumTrajectories;
   }

   @Override
   public List<? extends AngularMomentumTrajectoryInterface> getSwingCoPTrajectories()
   {
      return swingAngularMomentumTrajectories;
   }
}
