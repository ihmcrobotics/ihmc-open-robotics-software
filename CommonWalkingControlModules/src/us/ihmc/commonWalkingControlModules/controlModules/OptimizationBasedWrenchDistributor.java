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
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class OptimizationBasedWrenchDistributor implements GroundReactionWrenchDistributor
{
   private CylinderAndPlaneContactForceOptimizerNative nativeOptimizer;
   private CylinderAndPlaneContactForceOptimizerNativeInput optimizerInput = new CylinderAndPlaneContactForceOptimizerNativeInput();
   private CylinderAndPlaneContactForceOptimizerNativeOutput optimizerOutput;
   private CylinderAndPlaneContactForceOptimizerMatrixCalculator optimizerInputPopulator;
   private CylinderAndPlaneContactForceOptimizerSpatialForceVectorCalculator optimizerOutputExtractor;

   private Map<PlaneContactState, EndEffector> previouslyUsedPlaneEndEffectors = new LinkedHashMap<PlaneContactState, EndEffector>();
   private Map<CylindricalContactState, EndEffector> previouslyUsedCylinderEndEffectors = new LinkedHashMap<CylindricalContactState, EndEffector>();
   private List<EndEffector> endEffectors = new ArrayList<EndEffector>();
   private List<EndEffectorOutput> endEffectorOutputs = new ArrayList<EndEffectorOutput>();
   private DenseMatrix64F desiredNetEnvironmentReactionWrench = new DenseMatrix64F(6, 1);
   private List<Wrench> wrenches = new ArrayList<Wrench>();
   private CenterOfPressureResolver copResolver = new CenterOfPressureResolver();
   private final ReferenceFrame centerOfMassFrame;
   private final BooleanYoVariable debug;
   private final YoVariableRegistry registry;
   private final List<YoFrameVector> wrenchLinearComponents = new ArrayList<YoFrameVector>();
   private final List<YoFrameVector> wrenchRotaryComponents = new ArrayList<YoFrameVector>();
   private final List<YoFramePoint> wrenchOrigins = new ArrayList<YoFramePoint>();

   DenseMatrix64F Cmatrix = CommonOps.diag(new double[]
   {
      1, 1, 1, 10.0, 10.0, 10.0
   });
   double wPhi = 0.000001;
   double wRho = 0.000001;


   public OptimizationBasedWrenchDistributor(ReferenceFrame centerOfMassFrame, YoVariableRegistry parentRegistry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      nativeOptimizer = new CylinderAndPlaneContactForceOptimizerNative(parentRegistry);
      this.registry = parentRegistry;
      this.debug = new BooleanYoVariable(this.getClass().getSimpleName() + "Debug", parentRegistry);
      this.debug.set(false);
      this.centerOfMassFrame = centerOfMassFrame;
      optimizerInputPopulator = new CylinderAndPlaneContactForceOptimizerMatrixCalculator(centerOfMassFrame, parentRegistry);
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
         // TODO: think deeply about this behavior.
      }


   }

   public void solveAndPackOutput(GroundReactionWrenchDistributorOutputData output, List<PlaneContactState> planes, List<CylindricalContactState> cylinders,
                                  List<EndEffector> endEffectors, CylinderAndPlaneContactForceOptimizerNativeInput optimizerInputLocal)
           throws NoConvergenceException
   {
      List<EndEffectorOutput> endEffectorOutputs = this.endEffectorOutputs;
      makeSureEndEffectorOutputsSizeMatchesThatOfEndEffectors(endEffectors, endEffectorOutputs);

      optimizeWrenchesIntoEndEffectorOutputs(endEffectors, optimizerInputLocal, endEffectorOutputs);

      List<Wrench> wrenches = this.wrenches;
      expandWrenchesArrayToFitAllEndEffectors(endEffectors, wrenches);

      int j = 0;
      for (PlaneContactState plane : planes)
      {
         endEffectorOutputs.get(j).packExternallyActingSpatialForceVector(wrenches.get(j));
         FramePoint2d centerOfPressure = new FramePoint2d(centerOfMassFrame);    // frame will be overwritten
         double normalTorque = copResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure, wrenches.get(j), plane.getPlaneFrame());
         FrameVector force = new FrameVector(centerOfMassFrame);
         wrenches.get(j).packLinearPart(force);
         output.set(plane, force, centerOfPressure, normalTorque);
         j++;
      }

      for (CylindricalContactState cylinder : cylinders)
      {
         wrenches.get(j).setToZero(cylinder.getEndEffectorFrame(), centerOfMassFrame);    // sets end effector frame
         output.setWrench(cylinder, wrenches.get(j));
         if (numCalls % 100 == 0)
            printIfDebug("cylinder " + j + " wrench is " + wrenches.get(j).toString());
         endEffectorOutputs.get(j).packExternallyActingSpatialForceVector(wrenches.get(j));    // packs using super class SpatialForceVector
         j++;
      }

      numCalls++;
   }

   int numCalls = 0;

   private void makeSureEndEffectorOutputsSizeMatchesThatOfEndEffectors(List<EndEffector> endEffectorsLocal, List<EndEffectorOutput> endEffectorOutputs)
   {
      if (endEffectorsLocal.size() != endEffectorOutputs.size())
      {
         setEndEffectorOutputsFresh(endEffectorsLocal, endEffectorOutputs);
      }
      else
      {
         resetEndEffectorOutputs(endEffectorsLocal, endEffectorOutputs);
      }
   }

   private void resetEndEffectorOutputs(List<EndEffector> endEffectorsLocal, List<EndEffectorOutput> endEffectorOutputs)
   {
      for (int i = 0; i < endEffectorsLocal.size(); i++)
      {
         endEffectorOutputs.set(i, endEffectorsLocal.get(i).getOutput());
      }
   }

   private void setEndEffectorOutputsFresh(List<EndEffector> endEffectorsLocal, List<EndEffectorOutput> endEffectorOutputs)
   {
      endEffectorOutputs.clear();

      for (int i = 0; i < endEffectorsLocal.size(); i++)
      {
         endEffectorOutputs.add(endEffectorsLocal.get(i).getOutput());
      }
   }

   private void expandWrenchesArrayToFitAllEndEffectors(List<EndEffector> endEffectorsLocal, List<Wrench> wrenchesLocal)
   {
      for (int i = 0; i < endEffectorsLocal.size(); i++)
      {
         if (wrenchesLocal.size() < i + 1)
         {
            ReferenceFrame frame = endEffectorsLocal.get(i).getReferenceFrame();
            wrenchesLocal.add(new Wrench(frame, centerOfMassFrame));
            wrenchLinearComponents.add(new YoFrameVector(frame.getName() + "DistributedWrenchLinear", frame, this.registry));
            wrenchRotaryComponents.add(new YoFrameVector(frame.getName() + "DistributedWrenchRotary", frame, this.registry));
            wrenchOrigins.add(new YoFramePoint(frame.getName() + "DistributedWrenchOrigin", frame, this.registry));
         }
      }
   }

   private void optimizeWrenchesIntoEndEffectorOutputs(List<EndEffector> endEffectorsLocal,
           CylinderAndPlaneContactForceOptimizerNativeInput optimizerInputLocal, List<EndEffectorOutput> endEffectorOutputsLocal)
           throws NoConvergenceException
   {
      nativeOptimizer.solve(optimizerInputLocal);
      optimizerOutput = nativeOptimizer.getOutput();
      optimizerOutputExtractor.computeAllWrenchesBasedOnNativeOutputAndInput(endEffectorsLocal, optimizerInputLocal, optimizerOutput);


      for (int i = 0; i < endEffectorsLocal.size(); i++)
      {
         endEffectorOutputsLocal.get(i).setExternallyActingSpatialForceVector(optimizerOutputExtractor.getSpatialForceVector(endEffectorsLocal.get(i)));
      }
   }

   private void setupOptimizerInput(List<PlaneContactState> planes, List<CylindricalContactState> cylinders,
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
         endEffector.updateFromPlane(plane);

         return endEffector;
      }

      EndEffector endEffector = EndEffector.fromPlane(centerOfMassFrame, plane, registry);
      previouslyUsedPlaneEndEffectors.put(plane, endEffector);

      return endEffector;
   }

   private EndEffector getOrCreate(CylindricalContactState cylinder)
   {
      if (previouslyUsedCylinderEndEffectors.containsKey(cylinder))
      {
         EndEffector endEffector = previouslyUsedCylinderEndEffectors.get(cylinder);
         endEffector.updateFromCylinder(cylinder);

         return endEffector;
      }

      EndEffector endEffector = EndEffector.fromCylinder(centerOfMassFrame, cylinder, registry);
      previouslyUsedCylinderEndEffectors.put(cylinder, endEffector);

      return endEffector;
   }

   private void printIfDebug(String message)
   {
      if (debug.getBooleanValue())
      {
         System.out.println(this.getClass().getSimpleName() + ": " + message);
      }
   }

}
