package us.ihmc.humanoidRobotics.model;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point2d;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class CenterOfPressureDataHolder
{
   private final List<RigidBody> rigidBodies;
   private final Map<RigidBody, ReferenceFrame> soleFrames;
   private final Map<RigidBody, Point2d> centerOfPressures = new LinkedHashMap<>();

   public CenterOfPressureDataHolder(Map<RigidBody, ReferenceFrame> soleFrames)
   {
      rigidBodies = new ArrayList<>(soleFrames.keySet());
      this.soleFrames = soleFrames;
   }

   public void setCenterOfPressure(Point2d centerOfPressure, RigidBody foot)
   {
      centerOfPressures.get(foot).set(centerOfPressure);
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

   public void getCenterOfPressure(Point2d centerOfPressureToPack, RigidBody foot)
   {
      centerOfPressureToPack.set(centerOfPressures.get(foot));
   }
   
   public void getCenterOfPressure(FramePoint2d centerOfPressureToPack, RigidBody foot)
   {
      centerOfPressureToPack.setIncludingFrame(soleFrames.get(foot), centerOfPressures.get(foot));
   }
   
   public List<RigidBody> getRigidBodies()
   {
      return rigidBodies;
   }
}