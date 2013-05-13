package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeInput;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNativeOutput;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylinderAndPlaneContactForceOptimizerMatrixCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.EndEffector;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.EndEffectorOutput;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorInputData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorOutputData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.OptimizerCylinderContactModel;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.OptimizerPlaneContactModel;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

public class OptimizationBasedWrenchDistributor implements GroundReactionWrenchDistributor
{
   CylinderAndPlaneContactForceOptimizerNative nativeOptimizer = new CylinderAndPlaneContactForceOptimizerNative();
   CylinderAndPlaneContactForceOptimizerNativeInput optimizerInput = new CylinderAndPlaneContactForceOptimizerNativeInput();
   CylinderAndPlaneContactForceOptimizerNativeOutput optimizerOutput;
   CylinderAndPlaneContactForceOptimizerMatrixCalculator optimizerInputPopulator;
   CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator optimizerOutputExtractor;

   Map<PlaneContactState, EndEffector> previouslyUsedPlaneEndEffectors = new LinkedHashMap<PlaneContactState, EndEffector>();
   Map<CylindricalContactState, EndEffector> previouslyUsedCylinderEndEffectors = new LinkedHashMap<CylindricalContactState, EndEffector>();
   List<EndEffector> endEffectors = new ArrayList<EndEffector>();
   List<EndEffectorOutput> endEffectorOutputs;
   DenseMatrix64F desiredNetEnvironmentReactionWrench = new DenseMatrix64F(6, 1);
   List<Wrench> wrenches = new ArrayList<Wrench>();
   private CenterOfPressureResolver copResolver = new CenterOfPressureResolver();
   private final ReferenceFrame centerOfMassFrame;

   DenseMatrix64F Cmatrix = CommonOps.diag(new double[]
   {
      1, 1, 1, 1.0, 1.0, 1.0
   });
   double wPhi = 0.000001;
   double wRho = 0.000001;


   public OptimizationBasedWrenchDistributor(ReferenceFrame centerOfMassFrame)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      optimizerInputPopulator = new CylinderAndPlaneContactForceOptimizerMatrixCalculator(centerOfMassFrame);
      optimizerOutputExtractor = new CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator(centerOfMassFrame);
   }

   public void solve(GroundReactionWrenchDistributorOutputData output, GroundReactionWrenchDistributorInputData input)
   {
      List<PlaneContactState> planes = input.getContactStates();
      List<CylindricalContactState> cylinders = input.getCylindricalContactStates();
      input.getDesiredNetSpatialForceVector().packMatrix(desiredNetEnvironmentReactionWrench);

      setupOptimizerInput(planes, cylinders, desiredNetEnvironmentReactionWrench, endEffectors, optimizerInput, wPhi, wRho, Cmatrix);

      try
      {
         solveAndPackOutput(output, planes, cylinders, endEffectors, optimizerInput);
      }
      catch (NoConvergenceException e)
      {
         e.printStackTrace();

         // don't change output in the no convergence case. Hopefully the previous state will be remembered.
         //TODO: think deeply about this behavior.
      }


   }

   public void solveAndPackOutput(GroundReactionWrenchDistributorOutputData output, List<PlaneContactState> planes, List<CylindricalContactState> cylinders,
                                  List<EndEffector> endEffectorsLocal, CylinderAndPlaneContactForceOptimizerNativeInput optimizerInputLocal)
           throws NoConvergenceException
   {
      optimizeWrenchesIntoEndEffectorOutputs(endEffectorsLocal, optimizerInputLocal, endEffectorOutputs);

      int j = 0;
      for (PlaneContactState plane : planes)
      {
         endEffectorOutputs.get(j).packExternallyActingSpatialForceVector(wrenches.get(j));
         FramePoint2d centerOfPressure = new FramePoint2d(ReferenceFrame.getWorldFrame());    // frame will be overwritten
         double normalTorque = copResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure, wrenches.get(j), plane.getPlaneFrame());
         FrameVector force = new FrameVector();
         wrenches.get(j).packLinearPart(force);
         output.set(plane, force, centerOfPressure, normalTorque);
         j++;
      }

      for (CylindricalContactState cylinder : cylinders)
      {
         wrenches.get(j).setToZero(cylinder.getEndEffectorFrame(), centerOfMassFrame);    // sets end effector frame
         output.setWrench(cylinder, wrenches.get(j));
         endEffectorOutputs.get(j).packExternallyActingSpatialForceVector(wrenches.get(j));    // packs using super class SpatialForceVector
         j++;
      }
   }

   public void optimizeWrenchesIntoEndEffectorOutputs(List<EndEffector> endEffectorsLocal,
         CylinderAndPlaneContactForceOptimizerNativeInput optimizerInputLocal,
         List<EndEffectorOutput> endEffectorOutputsLocal) throws NoConvergenceException
   {
      nativeOptimizer.solve(optimizerInputLocal);
      optimizerOutput = nativeOptimizer.getOutput();
      optimizerOutputExtractor.computeAllWrenchesBasedOnNativeOutputAndInput(endEffectorsLocal, optimizerInputLocal, optimizerOutput);

      for (int i = 0; i < endEffectorsLocal.size(); i++)
      {
         endEffectorOutputsLocal.get(i).setExternallyActingSpatialForceVector(optimizerOutputExtractor.getSpatialForceVector(endEffectorsLocal.get(i)));
      }
   }

   public void setupOptimizerInput(List<PlaneContactState> planes, List<CylindricalContactState> cylinders,
                                   DenseMatrix64F desiredNetEnvironmentReactionWrenchLocal, List<EndEffector> endEffectorsLocal,
                                   CylinderAndPlaneContactForceOptimizerNativeInput optimizerInputLocal, double wPhiLocal, double wRhoLocal,
                                   DenseMatrix64F cmatrixLocal)
   {
      endEffectorsLocal.clear();

      for (PlaneContactState plane : planes)
      {
         endEffectorsLocal.add(getOrCreate(plane));
      }

      for (CylindricalContactState cylinder : cylinders)
      {
         endEffectorsLocal.add(getOrCreate(cylinder));
      }


      optimizerInputLocal.resetToZeros();
      optimizerInputLocal.setCylinderBoundedVectorRegularization(wPhiLocal);
      optimizerInputLocal.setGroundReactionForceRegularization(wRhoLocal);
      optimizerInputLocal.setMomentumDotWeight(cmatrixLocal);
      optimizerInputLocal.setWrenchEquationRightHandSide(desiredNetEnvironmentReactionWrenchLocal);

      optimizerInputPopulator.computeAllMatriciesAndPopulateNativeInput(endEffectorsLocal, optimizerInputLocal);
   }

   private EndEffector getOrCreate(PlaneContactState plane)
   {
      if (previouslyUsedPlaneEndEffectors.containsKey(plane))
      {
         EndEffector endEffector = previouslyUsedPlaneEndEffectors.get(plane);
         setEndEffectorFromPlaneContactState(plane, endEffector);

         return endEffector;
      }

      EndEffector endEffector = new EndEffector();
      endEffector.setContactModel(new OptimizerPlaneContactModel());

      setEndEffectorFromPlaneContactState(plane, endEffector);
      previouslyUsedPlaneEndEffectors.put(plane, endEffector);

      return endEffector;
   }

   private EndEffector getOrCreate(CylindricalContactState cylinder)
   {
      if (previouslyUsedCylinderEndEffectors.containsKey(cylinder))
      {
         EndEffector endEffector = previouslyUsedCylinderEndEffectors.get(cylinder);
         setEndEffectorFromCylindricalContactState(cylinder, endEffector);

         return endEffector;
      }

      EndEffector endEffector = new EndEffector();
      endEffector.setContactModel(new OptimizerPlaneContactModel());

      setEndEffectorFromCylindricalContactState(cylinder, endEffector);
      previouslyUsedCylinderEndEffectors.put(cylinder, endEffector);

      return endEffector;
   }

   private void setEndEffectorFromCylindricalContactState(CylindricalContactState cylinder, EndEffector endEffector)
   {
      OptimizerCylinderContactModel optimizerModel = (OptimizerCylinderContactModel) endEffector.getContactModel();
      optimizerModel.setup(cylinder.getCoefficientOfFriction(), cylinder.getCylinderRadius(), cylinder.getHalfHandWidth(), cylinder.getTensileGripForce());
      endEffector.setLoadBearing(cylinder.isInContact());
      endEffector.setReferenceFrame(cylinder.getEndEffectorFrame());
   }

   private void setEndEffectorFromPlaneContactState(PlaneContactState plane, EndEffector endEffector)
   {
      OptimizerPlaneContactModel optimizerPlaneContactModel = (OptimizerPlaneContactModel) endEffector.getContactModel();
      optimizerPlaneContactModel.setup(plane.getCoefficientOfFriction(), plane.getContactPoints(), plane.getBodyFrame());
      endEffector.setLoadBearing(plane.inContact());
      endEffector.setReferenceFrame(plane.getBodyFrame());
   }
}
