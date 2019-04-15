package us.ihmc.humanoidRobotics.model;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
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

   /**
    * This is used for lookups only and is populated from the {@link #centerOfPressures}. It should not be modified
    * directly and does not represent the state of the class.
    */
   private final transient Map<RigidBodyBasics, FramePoint2D> centerOfPressureMap = new LinkedHashMap<>();

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
   }

   public void registerRigidBody(RigidBodyBasics rigidBody)
   {
      if (bodiesWithCenterOfPressures.contains(rigidBody))
         throw new RuntimeException("The body: " + rigidBody.getName() + " has already been registered.");

      FramePoint2D cop = centerOfPressures.add();
      cop.setToNaN(ReferenceFrame.getWorldFrame());
      bodiesWithCenterOfPressures.add(rigidBody);
      centerOfPressureMap.put(rigidBody, cop);
   }

   public void registerRigidBody(RigidBodyBasics rigidBody, FramePoint2DReadOnly centerOfPressure)
   {
      if (bodiesWithCenterOfPressures.contains(rigidBody))
         throw new RuntimeException("The body: " + rigidBody.getName() + " has already been registered.");

      FramePoint2D cop = centerOfPressures.add();
      cop.setIncludingFrame(centerOfPressure);
      bodiesWithCenterOfPressures.add(rigidBody);
      centerOfPressureMap.put(rigidBody, cop);
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
      centerOfPressureMap.get(foot).setIncludingFrame(centerOfPressure);
   }

   public void setCenterOfPressure(ReferenceFrame referenceFrame, Point2DReadOnly centerOfPressure, RigidBodyBasics foot)
   {
      centerOfPressureMap.get(foot).setIncludingFrame(referenceFrame, centerOfPressure);
   }

   public void setCenterOfPressure(ReferenceFrame referenceFrame, Point2DReadOnly centerOfPressure, int bodyIndex)
   {
      centerOfPressures.get(bodyIndex).setIncludingFrame(referenceFrame, centerOfPressure);
   }

   public void getCenterOfPressure(FramePoint2DBasics centerOfPressureToPack, RigidBodyBasics foot)
   {
      centerOfPressureToPack.setIncludingFrame(centerOfPressureMap.get(foot));
   }

   public FramePoint2D getCenterOfPressure(RigidBodyBasics foot)
   {
      return centerOfPressureMap.get(foot);
   }

   public FramePoint2D getCenterOfPressure(int bodyIndex)
   {
      return centerOfPressures.get(bodyIndex);
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
         return true;
      }
      else
      {
         return false;
      }
   }
}