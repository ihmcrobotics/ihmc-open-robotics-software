package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameOrientationWaypoint;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePointWaypoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface PelvisPoseProvider
{
   public abstract boolean checkForNewPosition();

   public abstract boolean checkForNewOrientation();

   public abstract boolean checkForHomePosition();

   public abstract boolean checkForHomeOrientation();
   
   public abstract Double getTrajectoryTimeToHome();
      
   public abstract FramePointWaypoint[] getDesiredPelvisPosition(ReferenceFrame desiredReferenceFrame);

   public abstract FrameOrientationWaypoint[] getDesiredPelvisOrientation(ReferenceFrame desiredReferenceFrame);
   
}