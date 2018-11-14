package us.ihmc.humanoidRobotics.model;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class CenterOfPressureDataHolder
{
   private final Map<String, RigidBodyBasics> nameToRigidBodyMap = new LinkedHashMap<String, RigidBodyBasics>();
   private final Map<RigidBodyBasics, ReferenceFrame> soleFrames;
   private final Map<RigidBodyBasics, Point2D> centerOfPressures = new LinkedHashMap<>();

   public CenterOfPressureDataHolder(Map<RigidBodyBasics, ReferenceFrame> soleFrames)
   {
      this.soleFrames = soleFrames;
      
      for(RigidBodyBasics rigidBody : soleFrames.keySet())
      {
         nameToRigidBodyMap.put(rigidBody.getName(), rigidBody);
         centerOfPressures.put(rigidBody, new Point2D());
      }
   }

   public void setCenterOfPressure(Point2D centerOfPressure, RigidBodyBasics foot)
   {
      centerOfPressures.get(foot).set(centerOfPressure);
   }
   
   public void setCenterOfPressureByName(Point2D centerOfPressure, RigidBodyBasics foot)
   {
      RigidBodyBasics footFromNameMap = nameToRigidBodyMap.get(foot.getName());
      setCenterOfPressure(centerOfPressure, footFromNameMap);
   }

   public void setCenterOfPressure(FramePoint2D centerOfPressure, RigidBodyBasics foot)
   {
      if (centerOfPressure != null)
      {
         centerOfPressure.checkReferenceFrameMatch(soleFrames.get(foot));
         centerOfPressures.get(foot).set(centerOfPressure);
      }
      else
      {
         centerOfPressures.get(foot).set(Double.NaN, Double.NaN);
      }
   }
   
   public void setCenterOfPressureByName(FramePoint2D centerOfPressure, RigidBodyBasics foot)
   {
      RigidBodyBasics footFromNameMap = nameToRigidBodyMap.get(foot.getName());
      setCenterOfPressure(centerOfPressure, footFromNameMap);
   }

   public void getCenterOfPressure(Point2D centerOfPressureToPack, RigidBodyBasics foot)
   {
      centerOfPressureToPack.set(centerOfPressures.get(foot));
   }
   
   public void getCenterOfPressureByName(Point2D centerOfPressureToPack, RigidBodyBasics foot)
   {      
      RigidBodyBasics footFromNameMap = nameToRigidBodyMap.get(foot.getName());
      getCenterOfPressure(centerOfPressureToPack, footFromNameMap);
   }
   
   public void getCenterOfPressure(FramePoint2D centerOfPressureToPack, RigidBodyBasics foot)
   {
      centerOfPressureToPack.setIncludingFrame(soleFrames.get(foot), centerOfPressures.get(foot));
   }
   
   public void getCenterOfPressureByName(FramePoint2D centerOfPressureToPack, RigidBodyBasics foot)
   {
      RigidBodyBasics footFromNameMap = nameToRigidBodyMap.get(foot.getName());
      getCenterOfPressure(centerOfPressureToPack, footFromNameMap);
   }
   
   public Set<RigidBodyBasics> getRigidBodies()
   {
      return soleFrames.keySet();
   }
}