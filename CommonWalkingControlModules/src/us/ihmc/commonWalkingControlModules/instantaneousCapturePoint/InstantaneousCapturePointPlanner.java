package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.utilities.math.geometry.FramePoint;

public interface InstantaneousCapturePointPlanner
{
   public void initializeSingleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime);

   public void initializeDoubleSupportInitialTransfer(TransferToAndNextFootstepsData transferToAndNextFootstepsData, Point3d initialICPPosition, double initialTime);

   public void initializeDoubleSupport(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double initialTime);

   public void initializeSingleSupport(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
         double initialTime, boolean stopIfReachedEnd);

   public void initializeDoubleSupportInitialTransfer(ArrayList<FramePoint> footLocationList, Point3d initialICPPosition, double singleSupportDuration,
         double doubleSupportDuration, double doubleSupportInitialTransferDuration, double omega0, double initialTime, boolean stopIfReachedEnd);

   public void initializeDoubleSupport(ArrayList<FramePoint> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
         double initialTime, boolean stopIfReachedEnd);

   public void getICPPositionAndVelocity(Point3d icpPostionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double time);

}
