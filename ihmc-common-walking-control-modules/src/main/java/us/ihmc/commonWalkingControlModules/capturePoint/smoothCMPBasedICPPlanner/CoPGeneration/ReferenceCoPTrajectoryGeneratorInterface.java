package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ReferenceCoPTrajectoryGeneratorInterface
{
   void updateListeners();

   void setSymmetricCoPConstantOffsets(CoPPointName name, Vector2DReadOnly copOffset);

   void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList);

   void clear();

   int getNumberOfFootstepsRegistered();

   void update(double currentTime);

   void getDesiredCenterOfPressure(FixedFramePoint3DBasics desiredCoPToPack);
   void getDesiredCenterOfPressure(FixedFramePoint3DBasics desiredCoPToPack, FixedFrameVector3DBasics desiredCoPVelocityToPack);

   void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide, RobotSide previousTransferToSide);

   void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide);

   boolean isDoneWalking();

   void setSafeDistanceFromSupportEdges(double distance);

   List<CoPPointsInFoot> getWaypoints();

   List<? extends CoPTrajectoryInterface> getTransferCoPTrajectories();

   List<? extends CoPTrajectoryInterface> getSwingCoPTrajectories();
}