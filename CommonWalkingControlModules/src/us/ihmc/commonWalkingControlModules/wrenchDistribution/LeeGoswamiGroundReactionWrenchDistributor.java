package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.LeeGoswamiCoPAndNormalTorqueOptimizer;
import us.ihmc.commonWalkingControlModules.controlModules.LeeGoswamiForceOptimizer;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class LeeGoswamiGroundReactionWrenchDistributor implements GroundReactionWrenchDistributor
{
   private final ReferenceFrame centerOfMassFrame;

   private final LinkedHashMap<PlaneContactState, Double> coefficientsOfFriction = new LinkedHashMap<PlaneContactState, Double>();
   private final LinkedHashMap<PlaneContactState, Double> rotationalCoefficientsOfFriction = new LinkedHashMap<PlaneContactState, Double>();

   private final LinkedHashMap<PlaneContactState, FrameVector> forces = new LinkedHashMap<PlaneContactState, FrameVector>();
   private final LinkedHashMap<PlaneContactState, FramePoint2d> centersOfPressure = new LinkedHashMap<PlaneContactState, FramePoint2d>();
   private final LinkedHashMap<PlaneContactState, Double> normalTorques = new LinkedHashMap<PlaneContactState, Double>();

   private final LeeGoswamiForceOptimizer leeGoswamiForceOptimizer;
   private final LeeGoswamiCoPAndNormalTorqueOptimizer leeGoswamiCoPAndNormalTorqueOptimizer;

   private final double rotationalCoefficientOfFrictionMultiplier;

   public LeeGoswamiGroundReactionWrenchDistributor(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
         double rotationalCoefficientOfFrictionMultiplier)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.leeGoswamiForceOptimizer = new LeeGoswamiForceOptimizer(centerOfMassFrame, parentRegistry);
      this.leeGoswamiCoPAndNormalTorqueOptimizer = new LeeGoswamiCoPAndNormalTorqueOptimizer(centerOfMassFrame, parentRegistry);
      this.rotationalCoefficientOfFrictionMultiplier = rotationalCoefficientOfFrictionMultiplier;
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

      List<PlaneContactState> contactStates = groundReactionWrenchDistributorInputData.getContactStates();

      for (PlaneContactState contactState : contactStates)
      {
         addContact(contactState, contactState.getCoefficientOfFriction());
      }

      SpatialForceVector desiredGroundReactionWrench = groundReactionWrenchDistributorInputData.getDesiredNetSpatialForceVector();
      this.solve(desiredGroundReactionWrench);

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

   private void addContact(PlaneContactState contactState, double coefficientOfFriction)
   {
      coefficientsOfFriction.put(contactState, coefficientOfFriction);
      double rotationalCoefficientOfFriction = coefficientOfFriction * rotationalCoefficientOfFrictionMultiplier;
      rotationalCoefficientsOfFriction.put(contactState, rotationalCoefficientOfFriction);

      forces.put(contactState, new FrameVector(centerOfMassFrame));
      centersOfPressure.put(contactState, new FramePoint2d(contactState.getPlaneFrame()));
      normalTorques.put(contactState, 0.0);
   }

   private void solve(SpatialForceVector desiredGroundReactionWrench)
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