package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class AngularMomentumTrajectoryGenerator implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private List<SwingAngularMomentumTrajectory> swingAngularMomentumTrajectories;
   private List<TransferAngularMomentumTrajectory> transferAngularMomentumTrajectories;
   private RecyclingArrayList<FramePoint> upcomingEntryCoPList;
   private RecyclingArrayList<FramePoint> upcomingExitCoPList;
   private RecyclingArrayList<AngularMomentumTrajectoryPoint> upcomingWaypointList; // used for calculating the angular momentum trajectory
   private List<FrameVector> angularMomentumWaypointList; // used to return angular momentum waypoints from the computed trajectories
   private YoEnum<AngularMomentumTrajectoryInput> trajectoryType;
   private int maxNumberOfFootstepsToPlan;
   private int maxNumberOfWaypointsToPlan;

   private FrameVector desiredAngularMomentum = new FrameVector();
   private FrameVector desiredTorque = new FrameVector();
   private FrameVector desiredRotatum = new FrameVector();

   private AngularMomentumTrajectory activeTrajectory;
   private double initialTime;

   public AngularMomentumTrajectoryGenerator(String namePrefix, YoVariableRegistry registry)
   {
      this(namePrefix, registry, AngularMomentumTrajectoryInput.CoP_WAYPOINTS);
   }

   public AngularMomentumTrajectoryGenerator(String namePrefix, YoVariableRegistry registry, AngularMomentumTrajectoryInput inputType)
   {
      this(namePrefix, registry, inputType, 10, 10);
   }

   public AngularMomentumTrajectoryGenerator(String namePrefix, YoVariableRegistry registry, AngularMomentumTrajectoryInput inputType, int maxNumberOfFootsteps,
                                             int maxNumberOfWaypoints)
   {
      swingAngularMomentumTrajectories = new ArrayList<>(maxNumberOfFootsteps);
      transferAngularMomentumTrajectories = new ArrayList<>(maxNumberOfFootsteps);
      upcomingEntryCoPList = new RecyclingArrayList<FramePoint>(maxNumberOfFootsteps, FramePoint.class);
      upcomingExitCoPList = new RecyclingArrayList<FramePoint>(maxNumberOfFootsteps, FramePoint.class);
      upcomingWaypointList = new RecyclingArrayList<AngularMomentumTrajectoryPoint>(maxNumberOfWaypoints, AngularMomentumTrajectoryPoint.class);
      trajectoryType = new YoEnum<>(namePrefix, registry, AngularMomentumTrajectoryInput.class);
      trajectoryType.set(inputType);
      angularMomentumWaypointList = new ArrayList<FrameVector>(Math.max(maxNumberOfFootsteps * 2, maxNumberOfWaypoints));
      this.maxNumberOfFootstepsToPlan = maxNumberOfFootsteps;
   }

   public void setInput(AngularMomentumTrajectoryInput inputType)
   {
      trajectoryType.set(inputType);
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
      upcomingEntryCoPList.clear();
      upcomingExitCoPList.clear();
      upcomingWaypointList.clear();
   }

   @Override
   public void addFootstepCoPsToPlan(List<FramePoint> entryCoPs, List<FramePoint> exitCoPs)
   {
      if (entryCoPs.size() != exitCoPs.size())
         return;

      for (int i = 0; i < entryCoPs.size(); i++)
      {
         upcomingEntryCoPList.add().set(entryCoPs.get(i));
         upcomingExitCoPList.add().set(exitCoPs.get(i));
      }
   }

   @Override
   public void addFootstepCoPsToPlan(FramePoint entryCoP, FramePoint exitCoP)
   {
      upcomingEntryCoPList.add().set(entryCoP);
      upcomingExitCoPList.add().set(exitCoP);
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
   public int getNumberOfRegisteredEntryCoPs()
   {
      return upcomingEntryCoPList.size();
   }

   @Override
   public int getNumberOfRegisteredExitCoPs()
   {
      return upcomingExitCoPList.size();
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
      //TODO complete this method
      return;
   }

   private void computeForDoubleSupportFromCoPWayPoints()
   {
      computeAngularMomentumApproximationForFootsteps(0);
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
      computeAngularMomentumApproximationForFootsteps(0);
   }

   FramePoint p1 = new FramePoint();
   FramePoint p2 = new FramePoint();
   FramePoint p3 = new FramePoint();
   FramePoint p4 = new FramePoint();

   private void computeAngularMomentumApproximationForFootsteps(int startIndex)
   {
      int numberOfFootstepToPlan = Math.min(maxNumberOfFootstepsToPlan, upcomingEntryCoPList.size());
      for (int index = startIndex; index < numberOfFootstepToPlan; index++)
      {
         computeAngularMomentumApproximationForFootstep(index);
      }
   }

   private void computeAngularMomentumApproximationForFootstep(int footIndex)
   {
      TransferAngularMomentumTrajectory transferTrajectory = transferAngularMomentumTrajectories.get(footIndex);
      SwingAngularMomentumTrajectory swingTrajectory = swingAngularMomentumTrajectories.get(footIndex);
      p1 = upcomingExitCoPList.get(footIndex);
      p2 = upcomingEntryCoPList.get(footIndex + 1);
      p3 = upcomingExitCoPList.get(footIndex + 1);
      p4 = upcomingEntryCoPList.get(footIndex + 2);
      swingTrajectory.computeFromCoPWaypoints(p1, p2, p3, p4);
      transferTrajectory.computeFromCoPWaypoints(p1, p2, p3, p4);
   }

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
