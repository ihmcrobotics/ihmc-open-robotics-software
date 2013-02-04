package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class GroundReactionWrenchDistributorInputData
{
   private final ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
   private final LinkedHashMap<PlaneContactState, Double> coefficientsOfFriction = new LinkedHashMap<PlaneContactState, Double>();
   private final LinkedHashMap<PlaneContactState, Double> rotationalCoefficientsOfFriction = new LinkedHashMap<PlaneContactState, Double>();
   
   private SpatialForceVector desiredNetSpatialForceVector;
   private RobotSide upcomingSupportSide;
   
   public void reset()
   {
      contactStates.clear();
      coefficientsOfFriction.clear();
      rotationalCoefficientsOfFriction.clear();
   }

   public void addContact(PlaneContactState contactState, double coefficientOfFriction, double rotationalCoefficientOfFriction)
   {
      contactStates.add(contactState);
      coefficientsOfFriction.put(contactState, coefficientOfFriction);
      rotationalCoefficientsOfFriction.put(contactState, rotationalCoefficientOfFriction);
   }
   
   public void setSpatialForceVectorAndUpcomingSupportSide(SpatialForceVector desiredNetSpatialForceVector, RobotSide upcomingSupportSide)
   {
      this.desiredNetSpatialForceVector = desiredNetSpatialForceVector;
      this.upcomingSupportSide = upcomingSupportSide;
   }
   
   public ArrayList<PlaneContactState> getContactStates()
   {
      return contactStates;
   }
   
   public double getCoefficientOfFriction(PlaneContactState contactState)
   {
      return coefficientsOfFriction.get(contactState);
   }
   
   public double getRotationalCoefficientsOfFriction(PlaneContactState contactState)
   {
      return rotationalCoefficientsOfFriction.get(contactState);
   }
   
   public SpatialForceVector getDesiredNetSpatialForceVector()
   {
      return desiredNetSpatialForceVector;
   }
   
   public RobotSide getUpcomingSupportSide()
   {
      return upcomingSupportSide;
   }
   
}
