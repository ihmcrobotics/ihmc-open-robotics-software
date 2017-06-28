package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.List;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;

public interface AngularMomentumTrajectoryGeneratorInterface 
{
   void updateListeners();

   void createVisualizerForConstantAngularMomentum(YoGraphicsList yoGraphicsList, ArtifactList artifactList);

   void clear();

   void addFootstepToPlan(Footstep footstep, FootstepTiming timing);
   
   void addAngularMomentumWaypointsToPlan(List<AngularMomentumTrajectoryPoint> waypointList);

   void addAngularMomentumWaypointToPlan(AngularMomentumTrajectoryPoint waypoint);
   
   int getNumberOfFootstepsRegistered();

   void update(double currentTime);

   void getDesiredAngularMomentum(FrameVector desiredAngMomToPack);

   void getDesiredAngularMomentum(FrameVector desiredAngMomToPack, FrameVector desiredTorqueToPack);

   void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack);

   void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack);

   void initializeForTransfer(double currentTime);

   void initializeForSwing(double currentTime);

   void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide);

   void computeReferenceAngularMomentumStartingFromSingleSupport(RobotSide supportSide);

   List<FrameVector> getWaypoints();

   List<? extends AngularMomentumTrajectory> getTransferCoPTrajectories();

   List<? extends AngularMomentumTrajectory> getSwingCoPTrajectories();
}
