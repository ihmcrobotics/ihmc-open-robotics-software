package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.List;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;

public interface AngularMomentumTrajectoryGeneratorInterface 
{
   void updateListeners();

   void createVisualizerForConstantAngularMomentum(YoGraphicsList yoGraphicsList, ArtifactList artifactList);

   void clear();

   void addFootstepCoPsToPlan(List<CoPPointsInFoot> copLocations);
   
   void addAngularMomentumWaypointsToPlan(List<AngularMomentumTrajectoryPoint> waypointList);

   void addAngularMomentumWaypointToPlan(AngularMomentumTrajectoryPoint waypoint);
   
   void update(double currentTime);

   void getDesiredAngularMomentum(FrameVector desiredAngMomToPack);

   void getDesiredAngularMomentum(FrameVector desiredAngMomToPack, FrameVector desiredTorqueToPack);

   void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack);

   void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack);

   void initializeForTransfer(double currentTime);

   void initializeForSwing(double currentTime);

   void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide);

   void computeReferenceAngularMomentumStartingFromSingleSupport(RobotSide supportSide);

   List<TrajectoryPoint3D> getWaypoints();

   List<? extends AngularMomentumTrajectoryInterface> getTransferCoPTrajectories();

   List<? extends AngularMomentumTrajectoryInterface> getSwingCoPTrajectories();
}
