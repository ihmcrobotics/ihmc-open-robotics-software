package us.ihmc.humanoidRobotics.model;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class CenterOfPressureDataHolder implements Settable<CenterOfPressureDataHolder>
{
   private final List<RigidBodyBasics> bodiesWithCenterOfPressures = new ArrayList<>();
   private final RecyclingArrayList<FramePoint2D> centerOfPressures = new RecyclingArrayList<>(FramePoint2D.class);

   // can replace with full trajectory data (backing data for CoMTrajectorySegment and VRP waypoints)
   private final TDoubleArrayList capturePointWaypointTimes = new TDoubleArrayList();
   private final RecyclingArrayList<FramePoint2D> capturePointPositionWaypoints = new RecyclingArrayList<>(FramePoint2D.class);
   private final RecyclingArrayList<FrameVector2D> capturePointVelocityWaypoints = new RecyclingArrayList<>(FrameVector2D.class);

   /**
    * This is used for lookups only and is populated from the {@link #centerOfPressures}. It should not be modified
    * directly and does not represent the state of the class.
    */
   private final transient TLongObjectMap<FramePoint2D> centerOfPressureMap = new TLongObjectHashMap<>();

   public CenterOfPressureDataHolder()
   {
   }

   public CenterOfPressureDataHolder(Collection<RigidBodyBasics> rigidBodies)
   {
      rigidBodies.forEach(rigidBody -> registerRigidBody(rigidBody));
   }

   public CenterOfPressureDataHolder(FullHumanoidRobotModel fullRobotModel)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         registerRigidBody(fullRobotModel.getFoot(robotSide));
      }
   }

   public void clear()
   {
      bodiesWithCenterOfPressures.clear();
      centerOfPressures.clear();
      centerOfPressureMap.clear();

      capturePointWaypointTimes.reset();
      capturePointPositionWaypoints.clear();
      capturePointVelocityWaypoints.clear();
   }

   public void registerRigidBody(RigidBodyBasics rigidBody)
   {
      if (bodiesWithCenterOfPressures.contains(rigidBody))
         throw new RuntimeException("The body: " + rigidBody.getName() + " has already been registered.");

      FramePoint2D cop = centerOfPressures.add();
      cop.setToNaN(ReferenceFrame.getWorldFrame());
      bodiesWithCenterOfPressures.add(rigidBody);
      centerOfPressureMap.put(rigidBody.hashCode(), cop);
   }

   public void registerRigidBody(RigidBodyBasics rigidBody, FramePoint2DReadOnly centerOfPressure)
   {
      if (bodiesWithCenterOfPressures.contains(rigidBody))
         throw new RuntimeException("The body: " + rigidBody.getName() + " has already been registered.");

      FramePoint2D cop = centerOfPressures.add();
      cop.setIncludingFrame(centerOfPressure);
      bodiesWithCenterOfPressures.add(rigidBody);
      centerOfPressureMap.put(rigidBody.hashCode(), cop);
   }

   public int getNumberOfBodiesWithCenterOfPressure()
   {
      return bodiesWithCenterOfPressures.size();
   }

   public RigidBodyBasics getRigidBody(int bodyIndex)
   {
      return bodiesWithCenterOfPressures.get(bodyIndex);
   }

   public void setCenterOfPressure(FramePoint2DReadOnly centerOfPressure, RigidBodyBasics foot)
   {
      centerOfPressureMap.get(foot.hashCode()).setIncludingFrame(centerOfPressure);
   }

   public void setCenterOfPressure(ReferenceFrame referenceFrame, Point2DReadOnly centerOfPressure, RigidBodyBasics foot)
   {
      centerOfPressureMap.get(foot.hashCode()).setIncludingFrame(referenceFrame, centerOfPressure);
   }

   public void setCenterOfPressure(ReferenceFrame referenceFrame, Point2DReadOnly centerOfPressure, int bodyIndex)
   {
      centerOfPressures.get(bodyIndex).setIncludingFrame(referenceFrame, centerOfPressure);
   }

   public void getCenterOfPressure(FramePoint2DBasics centerOfPressureToPack, RigidBodyBasics foot)
   {
      centerOfPressureToPack.setIncludingFrame(centerOfPressureMap.get(foot.hashCode()));
   }

   public FramePoint2D getCenterOfPressure(RigidBodyBasics foot)
   {
      return centerOfPressureMap.get(foot.hashCode());
   }

   public FramePoint2D getCenterOfPressure(int bodyIndex)
   {
      return centerOfPressures.get(bodyIndex);
   }

   public void clearCapturePointWaypoints()
   {
      capturePointWaypointTimes.reset();
      capturePointPositionWaypoints.clear();
      capturePointVelocityWaypoints.clear();
   }

   public void addCapturePointWaypoint(double t, FramePoint2DReadOnly position, FrameVector2DReadOnly velocity)
   {
      capturePointWaypointTimes.add(t);
      capturePointPositionWaypoints.add().set(position);
      capturePointVelocityWaypoints.add().set(velocity);
   }

   public TDoubleArrayList getCapturePointWaypointTimes()
   {
      return capturePointWaypointTimes;
   }

   public RecyclingArrayList<FramePoint2D> getCapturePointPositionWaypoints()
   {
      return capturePointPositionWaypoints;
   }

   public RecyclingArrayList<FrameVector2D> getCapturePointVelocityWaypoints()
   {
      return capturePointVelocityWaypoints;
   }

   @Override
   public void set(CenterOfPressureDataHolder other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfBodiesWithCenterOfPressure(); i++)
      {
         RigidBodyBasics rigidBody = other.getRigidBody(i);
         registerRigidBody(rigidBody, other.getCenterOfPressure(i));
      }
      for (int i = 0; i < other.capturePointWaypointTimes.size(); i++)
      {
         capturePointWaypointTimes.add(other.capturePointWaypointTimes.get(i));
         capturePointPositionWaypoints.add().set(other.capturePointPositionWaypoints.get(i));
         capturePointVelocityWaypoints.add().set(other.capturePointVelocityWaypoints.get(i));
      }
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof CenterOfPressureDataHolder)
      {
         CenterOfPressureDataHolder other = (CenterOfPressureDataHolder) obj;
         if (getNumberOfBodiesWithCenterOfPressure() != other.getNumberOfBodiesWithCenterOfPressure())
            return false;
         for (int i = 0; i < getNumberOfBodiesWithCenterOfPressure(); i++)
         {
            RigidBodyBasics rigidBody = getRigidBody(i);
            if (!getCenterOfPressure(i).equals(other.getCenterOfPressure(rigidBody)))
               return false;
         }
         if (capturePointWaypointTimes.size() != other.capturePointWaypointTimes.size())
            return false;
         if (capturePointPositionWaypoints.size() != other.capturePointPositionWaypoints.size())
            return false;
         if (capturePointVelocityWaypoints.size() != other.capturePointVelocityWaypoints.size())
            return false;
         for (int i = 0; i < capturePointWaypointTimes.size(); i++)
         {
            if (capturePointWaypointTimes.get(i) != other.capturePointWaypointTimes.get(i))
               return false;
         }
         for (int i = 0; i < capturePointPositionWaypoints.size(); i++)
         {
            if (capturePointPositionWaypoints.get(i).equals(other.capturePointPositionWaypoints.get(i)))
               return false;
         }
         for (int i = 0; i < capturePointVelocityWaypoints.size(); i++)
         {
            if (capturePointVelocityWaypoints.get(i).equals(other.capturePointVelocityWaypoints.get(i)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }
}