package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.HashMap;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LeeGoswamiGroundReactionWrenchDistributor
{
   private final ReferenceFrame centerOfMassFrame;

   private final HashMap<PlaneContactState, Double> coefficientsOfFriction = new HashMap<PlaneContactState, Double>();
   private final HashMap<PlaneContactState, Double> rotationalCoefficientsOfFriction = new HashMap<PlaneContactState, Double>();

   private final LinkedHashMap<PlaneContactState, FrameVector> forces = new LinkedHashMap<PlaneContactState, FrameVector>();
   private final LinkedHashMap<PlaneContactState, FramePoint2d> centersOfPressure = new LinkedHashMap<PlaneContactState, FramePoint2d>();
   private final HashMap<PlaneContactState, Double> normalTorques = new HashMap<PlaneContactState, Double>();

   private final LeeGoswamiForceOptimizer leeGoswamiForceOptimizer;
   private final LeeGoswamiCoPAndNormalTorqueOptimizer leeGoswamiCoPAndNormalTorqueOptimizer;


   public LeeGoswamiGroundReactionWrenchDistributor(ReferenceFrame centerOfMassFrame, FrameVector gravitationalAcceleration, double mass, int nSupportVectors,
           YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.leeGoswamiForceOptimizer = new LeeGoswamiForceOptimizer(centerOfMassFrame, gravitationalAcceleration, mass, nSupportVectors, parentRegistry);
      this.leeGoswamiCoPAndNormalTorqueOptimizer = new LeeGoswamiCoPAndNormalTorqueOptimizer(centerOfMassFrame, parentRegistry);
   }

   public void reset()
   {
      coefficientsOfFriction.clear();
      rotationalCoefficientsOfFriction.clear();
      forces.clear();
      centersOfPressure.clear();
      normalTorques.clear();
   }

   public void addContact(PlaneContactState contactState, double coefficientOfFriction, double rotationalCoefficientOfFriction)
   {
      coefficientsOfFriction.put(contactState, coefficientOfFriction);
      forces.put(contactState, new FrameVector(contactState.getBodyFrame()));

      rotationalCoefficientsOfFriction.put(contactState, rotationalCoefficientOfFriction);
      forces.put(contactState, new FrameVector(centerOfMassFrame));
      centersOfPressure.put(contactState, new FramePoint2d(contactState.getPlaneFrame()));
      normalTorques.put(contactState, 0.0);
   }

   public void solve(SpatialForceVector desiredNetSpatialForceVector)
   {
      desiredNetSpatialForceVector.changeFrame(centerOfMassFrame);

      leeGoswamiForceOptimizer.solve(forces, coefficientsOfFriction, desiredNetSpatialForceVector);
      leeGoswamiCoPAndNormalTorqueOptimizer.solve(centersOfPressure, normalTorques, rotationalCoefficientsOfFriction, desiredNetSpatialForceVector.getAngularPartCopy(), forces);
   }

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
      return normalTorques.get(contactState);
   }
}
