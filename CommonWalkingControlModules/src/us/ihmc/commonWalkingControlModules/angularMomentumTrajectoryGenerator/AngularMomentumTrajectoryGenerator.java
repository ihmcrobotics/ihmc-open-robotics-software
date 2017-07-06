package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class AngularMomentumTrajectoryGenerator implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final YoEnum<AngularMomentumTrajectoryInput> trajectoryType;
   private final YoInteger maxNumberOfFootstepsToConsider;
   private final YoInteger maxNumberOfWaypointsPerFootstep;
   private final List<YoDouble> swingDurations;
   private final List<YoDouble> transferDurations;   
   private final List<YoDouble> swingSplitFractions;   
   private final List<YoDouble> swingDurationShiftFractions;
   private final List<YoDouble> transferSplitDurations;
   
   private final List<CoPPointsInFoot> upcomingCoPsInFootsteps;

   private final List<SwingAngularMomentumTrajectory> swingAngularMomentumTrajectories;
   private final List<TransferAngularMomentumTrajectory> transferAngularMomentumTrajectories;
   
   private final RecyclingArrayList<AngularMomentumTrajectoryPoint> upcomingWaypointList; // used for calculating the angular momentum trajectory
   private final RecyclingArrayList<FrameVector> angularMomentumWaypointList; // used to return angular momentum waypoints from the computed trajectories
   
   private final FrameVector desiredAngularMomentum = new FrameVector();
   private final FrameVector desiredTorque = new FrameVector();
   private final FrameVector desiredRotatum = new FrameVector();

   private final YoFrameTrajectory3D comTrajectory;
   private final YoFrameTrajectory3D swingFootTrajectory;
   private final YoFrameTrajectory3D swingFootVelocity;
   private final YoFrameTrajectory3D angularMomentumTrajectory;

   private AngularMomentumTrajectory activeTrajectory;
   private double initialTime;

   public AngularMomentumTrajectoryGenerator(String namePrefix, AngularMomentumTrajectoryInput inputType, YoInteger numberOfFootstepsToConsider, 
                                             YoInteger numberOfWaypointsPerFootstep, YoInteger maxNumberOfCoPPointsPerFoot, List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> swingSplitFractions,
                                             List<YoDouble> swingDurationShiftFractions, List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.maxNumberOfFootstepsToConsider = new YoInteger(namePrefix + "AngularMomentumPlanMaxFootsteps", registry);
      this.maxNumberOfFootstepsToConsider.set(numberOfFootstepsToConsider.getIntegerValue());
      this.maxNumberOfWaypointsPerFootstep = new YoInteger(namePrefix + "AngularMomentumPlanMaxMomentumWaypoints", registry);
      this.maxNumberOfWaypointsPerFootstep.set(numberOfWaypointsPerFootstep.getIntegerValue());
      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.swingDurationShiftFractions = swingDurationShiftFractions;
      this.transferSplitDurations = transferSplitFractions;
            
      
      this.swingAngularMomentumTrajectories = new ArrayList<>(numberOfFootstepsToConsider.getIntegerValue());
      this.transferAngularMomentumTrajectories = new ArrayList<>(numberOfFootstepsToConsider.getIntegerValue());      
      this.upcomingCoPsInFootsteps = new RecyclingArrayList<CoPPointsInFoot>(numberOfFootstepsToConsider.getIntegerValue(), CoPPointsInFoot.class);
      for(int i = 0; i < maxNumberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         SwingAngularMomentumTrajectory swingTrajectory = new SwingAngularMomentumTrajectory(namePrefix + "Footstep" + i + "SwingTrajectory", registry, worldFrame, maxNumberOfWaypointsPerFootstep.getIntegerValue());
         this.swingAngularMomentumTrajectories.add(swingTrajectory);
         TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(namePrefix + "Footstep" + i + "TransferTrajectory", registry, worldFrame, maxNumberOfWaypointsPerFootstep.getIntegerValue());
         this.transferAngularMomentumTrajectories.add(transferTrajectory);
         ReferenceFrame[] referenceFrames = {worldFrame};
         CoPPointsInFoot copLocations = new CoPPointsInFoot(i, maxNumberOfCoPPointsPerFoot.getIntegerValue(), referenceFrames, registry);
         upcomingCoPsInFootsteps.add(copLocations);
      }
      
      this.upcomingWaypointList = new RecyclingArrayList<AngularMomentumTrajectoryPoint>(numberOfFootstepsToConsider.getIntegerValue() * numberOfWaypointsPerFootstep.getIntegerValue(), AngularMomentumTrajectoryPoint.class);
      this.angularMomentumWaypointList = new RecyclingArrayList<FrameVector>(numberOfFootstepsToConsider.getIntegerValue() * numberOfWaypointsPerFootstep.getIntegerValue(), FrameVector.class); // this is for computing the way points from the trajectories 
      
      this.trajectoryType = new YoEnum<>(namePrefix, registry, AngularMomentumTrajectoryInput.class);
      this.trajectoryType.set(inputType);      
      
      this.comTrajectory = new YoFrameTrajectory3D("EstimatedCoMTrajectory", 4, worldFrame, registry);
      this.swingFootTrajectory = new YoFrameTrajectory3D("EstimatedCoMTrajectory", 4, worldFrame, registry);
      this.swingFootVelocity= new YoFrameTrajectory3D("EstimatedCoMTrajectory", 4, worldFrame, registry);
      this.angularMomentumTrajectory = new YoFrameTrajectory3D("EstiamtedAngularMomentumTrajectory", 7, worldFrame, registry);
   }

   public void setInput(AngularMomentumTrajectoryInput inputType)
   {
      this.trajectoryType.set(inputType);
   }

   public AngularMomentumTrajectoryInput getInputType()
   {
      return trajectoryType.getEnumValue();
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
      for(int i = 0;  i < maxNumberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         swingAngularMomentumTrajectories.get(i).reset();
         transferAngularMomentumTrajectories.get(i).reset();
      }
      angularMomentumWaypointList.clear();
      upcomingWaypointList.clear();
   }

   @Override
   public void addFootstepCoPsToPlan(List<CoPPointsInFoot> copLocations)
   {
      for (int i = 0; i < copLocations.size(); i++)
         upcomingCoPsInFootsteps.get(i).set(copLocations.get(i));
   }

   @Override
   public void addAngularMomentumWaypointsToPlan(List<AngularMomentumTrajectoryPoint> waypointList)
   {
      for (int i = 0; i < waypointList.size(); i++)
      {
         upcomingWaypointList.add().set(waypointList.get(i));
      }
   }

   @Override
   public void addAngularMomentumWaypointToPlan(AngularMomentumTrajectoryPoint waypoint)
   {
      upcomingWaypointList.add().set(waypoint);
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
      if (trajectoryType.equals(AngularMomentumTrajectoryInput.ANGULAR_MOMENTUM_WAYPOINTS))
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
      //TODO complete this method post discussion with WDI
      return;
   }

   private void computeForDoubleSupportFromCoPWayPoints()
   {
      int numberOfFootstepToPlan = Math.min(maxNumberOfFootstepsToConsider.getIntegerValue(), upcomingCoPsInFootsteps.size());
      for (int index = 0; index < numberOfFootstepToPlan; index++)
         computeTransferAngularMomentumApproximationForFootstep(index);
   }

   private void computeTransferAngularMomentumApproximationForFootstep(int footIndex)
   {
      TransferAngularMomentumTrajectory transferTrajectory = transferAngularMomentumTrajectories.get(footIndex);
      CoPPointsInFoot previousFootstepCoPPlan = upcomingCoPsInFootsteps.get(footIndex);
      p1 = upcomingCoPsInFootsteps.get(footIndex).get(maxNumberOfWaypointsPerFootstep.getIntegerValue() -1).getFrameTuple();
      p2 = upcomingCoPsInFootsteps.get(footIndex+1).get(0).getFrameTuple();
      p3 = upcomingCoPsInFootsteps.get(footIndex+1).get(maxNumberOfWaypointsPerFootstep.getIntegerValue() -1).getFrameTuple();
      p4 = upcomingCoPsInFootsteps.get(footIndex+2).get(0).getFrameTuple();
      comTrajectory.setCubicBezier(p1, p2, p3, p4);
      swingFootTrajectory.setConstant(z0);
      swingFootTrajectory.subtractByTrimming(comTrajectory);
      swingFootTrajectory.getDerivative(swingFootVelocity);
      TrajectoryMathTools.crossProductByTrimming(angularMomentumTrajectory, swingFootTrajectory, swingFootVelocity);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport(RobotSide supportSide)
   {
      if (trajectoryType.equals(AngularMomentumTrajectoryInput.ANGULAR_MOMENTUM_WAYPOINTS))
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
      int numberOfFootstepToPlan = Math.min(maxNumberOfFootstepsToConsider, upcomingEntryCoPList.size());
      for (int index = 0; index < numberOfFootstepToPlan; index++)
         computeSwingAngularMomentumApproximationForFootstep(index);
   }

   private void computeSwingAngularMomentumApproximationForFootstep(int footIndex)
   {
      TransferAngularMomentumTrajectory transferTrajectory = transferAngularMomentumTrajectories.get(footIndex);
      p1 = upcomingExitCoPList.get(footIndex);
      p2 = upcomingEntryCoPList.get(footIndex + 1);
      p3 = upcomingExitCoPList.get(footIndex + 1);
      p4 = upcomingEntryCoPList.get(footIndex + 2);
      comTrajectory.setCubicBezier(p1, p2, p3, p4);
      swingFootTrajectory.setCubic(z0, zFinal);
      swingFootTrajectory.subtractByTrimming(comTrajectory);
      swingFootTrajectory.getDerivative(swingFootVelocity);
      angularMomentumTrajectory.crossProduct(swingFootTrajectory,swingFootVelocity);
   }

   FramePoint p1 = new FramePoint();
   FramePoint p2 = new FramePoint();
   FramePoint p3 = new FramePoint();
   FramePoint p4 = new FramePoint();

   @Override
   public List<FrameVector> getWaypoints()
   {
      return angularMomentumWaypointList;
   }

   @Override
   public List<? extends AngularMomentumTrajectory> getTransferCoPTrajectories()
   {
      return transferAngularMomentumTrajectories;
   }

   @Override
   public List<? extends AngularMomentumTrajectory> getSwingCoPTrajectories()
   {
      return swingAngularMomentumTrajectories;
   }
}
