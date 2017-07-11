package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ReferenceCoPTrajectoryGeneratorInterface
{

   void updateListeners();

   void initializeParameters(SmoothCMPPlannerParameters parameters);

   void setSymmetricCoPConstantOffsets(int waypointNumber, Vector2D heelOffset);

   void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList);

   void clear();

   /**
    * Add footstep location to planned
    * @param footstep
    */
   void addFootstepToPlan(Footstep footstep, FootstepTiming timing);

   int getNumberOfFootstepsRegistered();

   void update(double currentTime);

   void getDesiredCenterOfPressure(FramePoint desiredCoPToPack);

   void getDesiredCenterOfPressure(FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack);

   void getDesiredCenterOfPressure(YoFramePoint desiredCoPToPack);

   void getDesiredCenterOfPressure(YoFramePoint desiredCoPToPack, YoFrameVector desiredCoPVelocityToPack);

   void initializeForTransfer(double currentTime);

   void initializeForSwing(double currentTime);

   void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide);

   void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide);

   /**
    * Remove first footstep in the upcoming footstep queue from planner
    */
   void removeFootstepQueueFront();

   /**
    * Removes the specified number of footsteps from the queue front
    * @param numberOfFootstepsToRemove number of steps to remove
    */

   void removeFootstepQueueFront(int numberOfFootstepsToRemove);

   /**
    * Removes specified footstep from upcoming footstep queue
    * @param index
    */
   void removeFootstep(int index);

   boolean isDoneWalking();

   void setSafeDistanceFromSupportEdges(double distance);

   List<CoPPointsInFoot> getWaypoints();

   List<? extends CoPTrajectory> getTransferCoPTrajectories();

   List<? extends CoPTrajectory> getSwingCoPTrajectories();
}