package us.ihmc.humanoidRobotics.model;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class CenterOfPressureDataHolder implements Settable<CenterOfPressureDataHolder>
{
   private final List<RigidBodyBasics> bodiesWithCenterOfPressures = new ArrayList<>();
   private final RecyclingArrayList<FramePoint2D> centerOfPressures = new RecyclingArrayList<>(FramePoint2D.class);

   private final FramePoint2D perfectCenterOfPressure = new FramePoint2D();

   private final ConvexPolygon2D supportPolygon = new ConvexPolygon2D();
   private final RecyclingArrayList<FramePoint2D> comPositionWaypoints = new RecyclingArrayList<>(FramePoint2D.class);
   private final RecyclingArrayList<FrameVector2D> comVelocityWaypoints = new RecyclingArrayList<>(FrameVector2D.class);
   private final RecyclingArrayList<FramePoint2D> copPositionWaypoints = new RecyclingArrayList<>(FramePoint2D.class);

   private double remainingTimeInContactSequence;

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
      perfectCenterOfPressure.setToNaN();
      supportPolygon.clear();
      comPositionWaypoints.clear();
      comVelocityWaypoints.clear();
      copPositionWaypoints.clear();
      remainingTimeInContactSequence = Double.NaN;
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

   public FramePoint2D getPerfectCenterOfPressure()
   {
      return perfectCenterOfPressure;
   }

   public void setSupportPolygon(ConvexPolygon2DReadOnly supportPolygon)
   {
      this.supportPolygon.set(supportPolygon);
   }

   public ConvexPolygon2D getSupportPolygon()
   {
      return supportPolygon;
   }

   public RecyclingArrayList<FramePoint2D> getComPositionWaypoints()
   {
      return comPositionWaypoints;
   }

   public RecyclingArrayList<FrameVector2D> getComVelocityWaypoints()
   {
      return comVelocityWaypoints;
   }

   public RecyclingArrayList<FramePoint2D> getCopPositionWaypoints()
   {
      return copPositionWaypoints;
   }

   public double getRemainingTimeInContactSequence()
   {
      return remainingTimeInContactSequence;
   }

   public void setRemainingTimeInContactSequence(double remainingTimeInContactSequence)
   {
      this.remainingTimeInContactSequence = remainingTimeInContactSequence;
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
      perfectCenterOfPressure.set(other.perfectCenterOfPressure);
      this.supportPolygon.set(other.supportPolygon);
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
         if (!perfectCenterOfPressure.equals(other.perfectCenterOfPressure))
            return false;
         if (!supportPolygon.equals(other.supportPolygon))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}