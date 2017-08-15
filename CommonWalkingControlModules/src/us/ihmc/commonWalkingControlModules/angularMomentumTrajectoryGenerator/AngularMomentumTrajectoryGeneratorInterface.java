package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.List;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;

public interface AngularMomentumTrajectoryGeneratorInterface 
{
   void updateListeners();

   void createVisualizerForConstantAngularMomentum(YoGraphicsList yoGraphicsList, ArtifactList artifactList);

   void clear();

   void update(double currentTime);

   void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack);

   void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack, FrameVector3D desiredTorqueToPack);

   void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack);

   void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack);

   void initializeForTransfer(double currentTime);

   void initializeForSwing(double currentTime);

   void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop);

   void computeReferenceAngularMomentumStartingFromSingleSupport();

   List<? extends AngularMomentumTrajectoryInterface> getTransferAngularMomentumTrajectories();

   List<? extends AngularMomentumTrajectoryInterface> getSwingAngularMomentumTrajectories();
}
