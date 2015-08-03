package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.bipedSupportPolygons.CylindricalContactState;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.Wrench;

public class GroundReactionWrenchDistributorOutputData
{
   private final LinkedHashMap<PlaneContactState, FrameVector> forces = new LinkedHashMap<PlaneContactState, FrameVector>();
   private final LinkedHashMap<PlaneContactState, FramePoint2d> centersOfPressure = new LinkedHashMap<PlaneContactState, FramePoint2d>();
   private final LinkedHashMap<PlaneContactState, Double> normalTorques = new LinkedHashMap<PlaneContactState, Double>();
   private final LinkedHashMap<CylindricalContactState, Wrench> wrenchesOfCylinderContacts = new LinkedHashMap<CylindricalContactState,Wrench>();
   public FrameVector getForce(PlaneContactState planeContactState)
   {
      return forces.get(planeContactState);
   }

   public FramePoint2d getCenterOfPressure(PlaneContactState contactState)
   {
      return centersOfPressure.get(contactState);
   }

   public double getNormalTorque(PlaneContactState contactState)
   {
      try
      {
      return normalTorques.get(contactState);
      } catch (NullPointerException e)
      {
         e.printStackTrace();
         return 0;
      }
   }
   
   public Wrench getWrenchOfNonPlaneContact(CylindricalContactState rigidBody)
   {
      return wrenchesOfCylinderContacts.get(rigidBody);
   }
   
   public void setWrench(CylindricalContactState cylinder, Wrench wrench)
   {
      wrenchesOfCylinderContacts.put(cylinder, wrench);
   }
   
   public void set(PlaneContactState planeContactState, FrameVector force, FramePoint2d centerOfPressure, double normalTorque)
   {
      forces.put(planeContactState, force);
      centersOfPressure.put(planeContactState, centerOfPressure);
      normalTorques.put(planeContactState, normalTorque);
   }

   public void reset()
   {
      forces.clear();
      centersOfPressure.clear();
      normalTorques.clear();
      wrenchesOfCylinderContacts.clear();
   }   
}
