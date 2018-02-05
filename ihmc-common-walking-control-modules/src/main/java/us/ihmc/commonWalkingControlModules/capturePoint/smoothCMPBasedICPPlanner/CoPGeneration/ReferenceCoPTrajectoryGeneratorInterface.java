package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public interface ReferenceCoPTrajectoryGeneratorInterface
{
   void updateListeners();

   void initializeParameters(SmoothCMPPlannerParameters parameters);

   void setSymmetricCoPConstantOffsets(CoPPointName name, Vector2DReadOnly copOffset);

   void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList);

   void clear();

   /**
    * Add footstep location to planned
    * @param footstep
    */
   void addFootstepToPlan(Footstep footstep, FootstepTiming timing);

   int getNumberOfFootstepsRegistered();

   void update(double currentTime);

   void getDesiredCenterOfPressure(FixedFramePoint3DBasics desiredCoPToPack);
   void getDesiredCenterOfPressure(FixedFramePoint3DBasics desiredCoPToPack, FixedFrameVector3DBasics desiredCoPVelocityToPack);

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