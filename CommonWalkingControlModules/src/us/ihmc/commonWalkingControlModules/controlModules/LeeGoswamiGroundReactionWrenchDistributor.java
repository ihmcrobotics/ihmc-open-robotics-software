package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LeeGoswamiGroundReactionWrenchDistributor implements GroundReactionWrenchDistributor
{
   private final ReferenceFrame centerOfMassFrame;

   private final HashMap<PlaneContactState, Double> coefficientsOfFriction = new HashMap<PlaneContactState, Double>();
   private final HashMap<PlaneContactState, Double> rotationalCoefficientsOfFriction = new HashMap<PlaneContactState, Double>();

   private final LinkedHashMap<PlaneContactState, FrameVector> forces = new LinkedHashMap<PlaneContactState, FrameVector>();
   private final LinkedHashMap<PlaneContactState, FramePoint2d> centersOfPressure = new LinkedHashMap<PlaneContactState, FramePoint2d>();
   private final LinkedHashMap<PlaneContactState, Double> normalTorques = new LinkedHashMap<PlaneContactState, Double>();

   private final LeeGoswamiForceOptimizer leeGoswamiForceOptimizer;
   private final LeeGoswamiCoPAndNormalTorqueOptimizer leeGoswamiCoPAndNormalTorqueOptimizer;


   public LeeGoswamiGroundReactionWrenchDistributor(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.leeGoswamiForceOptimizer = new LeeGoswamiForceOptimizer(centerOfMassFrame, parentRegistry);
      this.leeGoswamiCoPAndNormalTorqueOptimizer = new LeeGoswamiCoPAndNormalTorqueOptimizer(centerOfMassFrame, parentRegistry);
   }

   public void setWeights(double wk, double epsilonF, double epsilonCoP, double epsilonTauN)
   {
      leeGoswamiForceOptimizer.setWk(wk);
      leeGoswamiForceOptimizer.setEpsilonF(epsilonF);
      leeGoswamiCoPAndNormalTorqueOptimizer.setEpsilonCoP(epsilonCoP);
      leeGoswamiCoPAndNormalTorqueOptimizer.setEpsilonTauN(epsilonTauN);
   }

   public void solve(GroundReactionWrenchDistributorOutputData distributedWrench,
         GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData)
   {
      reset();
      
      ArrayList<PlaneContactState> contactStates = groundReactionWrenchDistributorInputData.getContactStates();
      
      for (PlaneContactState contactState : contactStates)
      {
         addContact(contactState, groundReactionWrenchDistributorInputData.getCoefficientOfFriction(contactState), 
               groundReactionWrenchDistributorInputData.getRotationalCoefficientsOfFriction(contactState));
      }
    
      SpatialForceVector desiredGroundReactionWrench = groundReactionWrenchDistributorInputData.getDesiredNetSpatialForceVector();
      RobotSide upcomingSupportleg = groundReactionWrenchDistributorInputData.getUpcomingSupportSide();
      this.solve(desiredGroundReactionWrench, upcomingSupportleg);
      
      this.getOutputData(distributedWrench);
   }
   
   private void reset()
   {
      // TODO: inefficient; should hang on to a bunch of temporary objects instead of deleting all references to them
      coefficientsOfFriction.clear();
      rotationalCoefficientsOfFriction.clear();
      forces.clear();
      centersOfPressure.clear();
      normalTorques.clear();
   }

   private void addContact(PlaneContactState contactState, double coefficientOfFriction, double rotationalCoefficientOfFriction)
   {
      coefficientsOfFriction.put(contactState, coefficientOfFriction);
      rotationalCoefficientsOfFriction.put(contactState, rotationalCoefficientOfFriction);

      forces.put(contactState, new FrameVector(centerOfMassFrame));
      centersOfPressure.put(contactState, new FramePoint2d(contactState.getPlaneFrame()));
      normalTorques.put(contactState, 0.0);
   }

   private void solve(SpatialForceVector desiredGroundReactionWrench, RobotSide upcomingSupportleg)
   {
      desiredGroundReactionWrench.changeFrame(centerOfMassFrame);

      leeGoswamiForceOptimizer.solve(forces, coefficientsOfFriction, desiredGroundReactionWrench);
      leeGoswamiCoPAndNormalTorqueOptimizer.solve(centersOfPressure, normalTorques, rotationalCoefficientsOfFriction,
              leeGoswamiForceOptimizer.getTorqueError(), forces);
   }

   private void getOutputData(GroundReactionWrenchDistributorOutputData outputData)
   {
      outputData.reset();
      for (PlaneContactState planeContactState : forces.keySet())
      {
         outputData.set(planeContactState, getForce(planeContactState), getCenterOfPressure(planeContactState), getNormalTorque(planeContactState));
      }
   }
   
   private FrameVector getForce(PlaneContactState planeContactState)
   {
      return forces.get(planeContactState);
   }

   private FramePoint2d getCenterOfPressure(PlaneContactState contactState)
   {
      return centersOfPressure.get(contactState);
   }

   private double getNormalTorque(PlaneContactState contactState)
   {
      return normalTorques.get(contactState);
   }
  
}
