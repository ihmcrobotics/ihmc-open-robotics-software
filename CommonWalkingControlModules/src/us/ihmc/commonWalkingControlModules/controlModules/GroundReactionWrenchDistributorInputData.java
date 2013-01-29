package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class GroundReactionWrenchDistributorInputData
{
   private final ArrayList<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
   private final ArrayList<Double> coefficientsOfFriction = new ArrayList<Double>();
   private final ArrayList<Double> rotationalCoefficientsOfFriction = new ArrayList<Double>();
   
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
      coefficientsOfFriction.add(coefficientOfFriction);
      rotationalCoefficientsOfFriction.add(rotationalCoefficientOfFriction);
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
   
   public ArrayList<Double> getCoefficientsOfFriction()
   {
      return coefficientsOfFriction;
   }
   
   public ArrayList<Double> getRotationalCoefficientsOfFriction()
   {
      return rotationalCoefficientsOfFriction;
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
