package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface PelvisPoseProvider
{
   public abstract boolean checkForNewPosition();

   public abstract boolean checkForNewOrientation();

   public abstract boolean checkForHomePosition();

   public abstract boolean checkForHomeOrientation();
   
   public abstract boolean checkForNewTrajectory();
   
   public abstract void removeLastTrajectory();

   public abstract FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame);
   
   public abstract ReferenceFrame getDesiredPelvisPositionTrajectory(
         ArrayList<Double> time,
         ArrayList<Point3d> position, 
         ArrayList<Vector3d> velocity);

   public abstract FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame);
   
   public abstract ReferenceFrame getDesiredPelvisOrientationTrajectory(double[] time, Point3d[] position, Vector3d[] velocity);

   public abstract double getTrajectoryTime();
   
}