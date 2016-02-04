package us.ihmc.humanoidRobotics.model;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Point2d;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class CenterOfPressureDataHolder
{
   private final Map<String, RigidBody> nameToRigidBodyMap = new LinkedHashMap<String, RigidBody>();
   private final Map<RigidBody, ReferenceFrame> soleFrames;
   private final Map<RigidBody, Point2d> centerOfPressures = new LinkedHashMap<>();

   public CenterOfPressureDataHolder(Map<RigidBody, ReferenceFrame> soleFrames)
   {
      this.soleFrames = soleFrames;
      
      for(RigidBody rigidBody : soleFrames.keySet())
      {
         nameToRigidBodyMap.put(rigidBody.getName(), rigidBody);
         centerOfPressures.put(rigidBody, new Point2d());
      }
   }

   public void setCenterOfPressure(Point2d centerOfPressure, RigidBody foot)
   {
      centerOfPressures.get(foot).set(centerOfPressure);
   }
   
   public void setCenterOfPressureByName(Point2d centerOfPressure, RigidBody foot)
   {
      RigidBody footFromNameMap = nameToRigidBodyMap.get(foot.getName());
      setCenterOfPressure(centerOfPressure, footFromNameMap);
   }

   public void setCenterOfPressure(FramePoint2d centerOfPressure, RigidBody foot)
   {
      if (centerOfPressure != null)
      {
         centerOfPressure.checkReferenceFrameMatch(soleFrames.get(foot));
         centerOfPressure.get(centerOfPressures.get(foot));
      }
      else
      {
         centerOfPressures.get(foot).set(Double.NaN, Double.NaN);
      }
   }
   
   public void setCenterOfPressureByName(FramePoint2d centerOfPressure, RigidBody foot)
   {
      RigidBody footFromNameMap = nameToRigidBodyMap.get(foot.getName());
      setCenterOfPressure(centerOfPressure, footFromNameMap);
   }

   public void getCenterOfPressure(Point2d centerOfPressureToPack, RigidBody foot)
   {
      centerOfPressureToPack.set(centerOfPressures.get(foot));
   }
   
   public void getCenterOfPressureByName(Point2d centerOfPressureToPack, RigidBody foot)
   {      
      RigidBody footFromNameMap = nameToRigidBodyMap.get(foot.getName());
      getCenterOfPressure(centerOfPressureToPack, footFromNameMap);
   }
   
   public void getCenterOfPressure(FramePoint2d centerOfPressureToPack, RigidBody foot)
   {
      centerOfPressureToPack.setIncludingFrame(soleFrames.get(foot), centerOfPressures.get(foot));
   }
   
   public void getCenterOfPressureByName(FramePoint2d centerOfPressureToPack, RigidBody foot)
   {
      RigidBody footFromNameMap = nameToRigidBodyMap.get(foot.getName());
      getCenterOfPressure(centerOfPressureToPack, footFromNameMap);
   }
   
   public Set<RigidBody> getRigidBodies()
   {
      return soleFrames.keySet();
   }
}