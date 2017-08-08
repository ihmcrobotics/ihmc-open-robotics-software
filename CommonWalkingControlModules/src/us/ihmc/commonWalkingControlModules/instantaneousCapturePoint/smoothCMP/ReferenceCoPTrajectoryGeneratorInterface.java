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

   void setSymmetricCoPConstantOffsets(CoPPointName name, Vector2D copOffset);

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

   boolean isDoneWalking();

   void setSafeDistanceFromSupportEdges(double distance);

   List<CoPPointsInFoot> getWaypoints();

   List<? extends CoPTrajectoryInterface> getTransferCoPTrajectories();

   List<? extends CoPTrajectoryInterface> getSwingCoPTrajectories();
}