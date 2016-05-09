package us.ihmc.commonWalkingControlModules.controllerCore;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.AntiGravityJointTorquesVisualizer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.mathfunctions.Matrix;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphic;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class VirtualModelController
{
   private final static boolean VISUALIZE_DESIRED_WRENCHES = true;

   private static final boolean DEBUG = true;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoVariableRegistry registry;
   private final HashMap<RigidBody, YoFrameVector> yoForceVectors = new HashMap<>();
   private final HashMap<RigidBody, YoFramePoint> yoForcePoints = new HashMap<>();
   private final HashMap<RigidBody, FramePoint> endEffectorPoints = new HashMap<>();

   private final GeometricJacobianHolder geometricJacobianHolder;
   private final RigidBody defaultRootBody;

   private Map<InverseDynamicsJoint, Double> jointTorques = new HashMap<>();
   private List<OneDoFJoint> controlledJoints = new ArrayList <>();

   private final List<RigidBody> endEffectors = new ArrayList<>();
   private final HashMap<RigidBody, RigidBody> baseOfEndEffector = new HashMap<>();
   private final HashMap<RigidBody, Wrench> endEffectorWrenches = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> endEffectorSelectionMatrices = new HashMap<>();
   private final HashMap<RigidBody, DenseMatrix64F> jointEffortMatrices = new HashMap<>();

   private final DenseMatrix64F tmpEffortMatrix = new DenseMatrix64F(1, 1);


   public VirtualModelController(GeometricJacobianHolder geometricJacobianHolder, RigidBody defaultRootBody, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.geometricJacobianHolder = geometricJacobianHolder;
      this.defaultRootBody = defaultRootBody;
      this.registry = registry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
   }

   public void registerEndEffector(RigidBody endEffector)
   {
      registerEndEffector(defaultRootBody, endEffector);
   }

   public void registerEndEffector(RigidBody base, RigidBody endEffector)
   {
      if (!endEffectors.contains(endEffector) && endEffector != null)
      {
         endEffectors.add(endEffector);
         baseOfEndEffector.put(endEffector, base);

         OneDoFJoint[] joints = ScrewTools.createOneDoFJointPath(base, endEffector);
         for (int i = 0; i < joints.length; i++)
         {
            if (!controlledJoints.contains(joints[i]))
               controlledJoints.add(joints[i]);
         }

         tmpEffortMatrix.reshape(joints.length, 1);
         tmpEffortMatrix.zero();
         jointEffortMatrices.put(endEffector, tmpEffortMatrix);


         if (VISUALIZE_DESIRED_WRENCHES && registry != null && yoGraphicsListRegistry != null)
         {
            YoFramePoint forceBase = new YoFramePoint(endEffector.getName() + "ForceBase", ReferenceFrame.getWorldFrame(), registry);
            FramePoint endEffectorPoint = new FramePoint(endEffector.getBodyFixedFrame());
            endEffectorPoint.changeFrame(ReferenceFrame.getWorldFrame());
            endEffectorPoints.put(endEffector, endEffectorPoint);

            forceBase.set(endEffectorPoint);

            YoFrameVector forceVector = new YoFrameVector(endEffector.getName() + "DesiredForce", base.getBodyFixedFrame(), registry);

            yoForcePoints.put(endEffector, forceBase);
            yoForceVectors.put(endEffector, forceVector);

            AppearanceDefinition forceAppearance = YoAppearance.AliceBlue();
            YoGraphic forceVisualizer = new YoGraphicVector(endEffector.getName() + "DesiredForce", forceBase, forceVector, 0.05, forceAppearance);
            yoGraphicsListRegistry.registerYoGraphic("forcePointVisualizer", forceVisualizer);
         }
      }
   }

   private final Wrench dummyWrench = new Wrench();
   private final DenseMatrix64F forceSelectionMatrix = new DenseMatrix64F(3, Wrench.SIZE);
   public void submitEndEffectorVirtualForce(RigidBody endEffector, FrameVector force)
   {
      dummyWrench.setToZero();
      dummyWrench.setToZero(baseOfEndEffector.get(endEffector).getBodyFixedFrame(), force.getReferenceFrame());
      dummyWrench.setLinearPart(force);

      forceSelectionMatrix.set(0, 3, 1);
      forceSelectionMatrix.set(1, 4, 1);
      forceSelectionMatrix.set(2, 5, 1);

      endEffectorWrenches.put(endEffector, dummyWrench);
      endEffectorSelectionMatrices.put(endEffector, forceSelectionMatrix);
   }

   public void submitEndEffectorVirtualForce(RigidBody endEffector, FrameVector force, DenseMatrix64F selectionMatrix)
   {
      dummyWrench.setToZero();
      dummyWrench.setToZero(baseOfEndEffector.get(endEffector).getBodyFixedFrame(), force.getReferenceFrame());
      dummyWrench.setLinearPart(force);

      forceSelectionMatrix.set(0, 3, 1);
      forceSelectionMatrix.set(1, 4, 1);
      forceSelectionMatrix.set(2, 5, 1);

      endEffectorWrenches.put(endEffector, dummyWrench);
      endEffectorSelectionMatrices.put(endEffector, forceSelectionMatrix);
   }

   public void submitEndEffectorVirtualWrench(RigidBody endEffector, Wrench wrench)
   {
      submitEndEffectorVirtualWrench(endEffector, wrench, new CommonOps().identity(Wrench.SIZE, Wrench.SIZE));
   }

   public void submitEndEffectorVirtualWrench(VirtualWrenchCommand virtualWrenchCommand)
   {
      submitEndEffectorVirtualWrench(virtualWrenchCommand.getRigidBody(), virtualWrenchCommand.getVirtualWrench(), virtualWrenchCommand.getSelectionMatrix());
   }

   public void submitEndEffectorVirtualWrench(RigidBody endEffector, Wrench wrench, DenseMatrix64F selectionMatrix)
   {
      wrench.changeBodyFrameAttachedToSameBody(endEffector.getBodyFixedFrame());

      endEffectorWrenches.put(endEffector, wrench);
      endEffectorSelectionMatrices.put(endEffector, selectionMatrix);
   }

   public void reset()
   {
      endEffectorWrenches.clear();
      endEffectorSelectionMatrices.clear();
      jointTorques.clear();
   }


   private final DenseMatrix64F tmpEndEffectorWrench = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tmpJacobianMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);

   private final DenseMatrix64F fullJacobianMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F fullObjectiveWrench = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F fullSelectionMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F fullEffortMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F isObjective = new DenseMatrix64F(1, Wrench.SIZE);

   public void compute(VirtualModelControlSolution virtualModelControlSolutionToPack)
   {
      for (RigidBody endEffector : endEffectors)
      {
         if (!endEffectorWrenches.containsKey(endEffector) || !endEffectorSelectionMatrices.containsKey(endEffector))
            throw new RuntimeException("Not all registered end effectors have required forces to compute desired joint torques.");
      }
      fullSelectionMatrix.reshape(1, 1);
      fullJacobianMatrix.reshape(controlledJoints.size(), Wrench.SIZE);
      fullEffortMatrix.reshape(controlledJoints.size(), 1);

      for (RigidBody endEffector : endEffectors)
      {
         DenseMatrix64F endEffectorSelectionMatrix = endEffectorSelectionMatrices.get(endEffector);
         int taskSize = endEffectorSelectionMatrix.getNumCols();
         isObjective.zero();

         // compute all rows containing value for full selection matrix
         for (int i = 0; i < taskSize; i++)
         {
            for (int j = 0; j < Wrench.SIZE; j++)
            {
               if (endEffectorSelectionMatrix.get(i, j) == 1)
               {
                  isObjective.set(0, j, 1);
                  break;
               }
            }
         }

         // get jacobian
         OneDoFJoint[] jointsInChain = ScrewTools.createOneDoFJointPath(baseOfEndEffector.get(endEffector), endEffector);
         long jacobianID = geometricJacobianHolder.getOrCreateGeometricJacobian(jointsInChain, endEffector.getBodyFixedFrame());
         GeometricJacobian jacobian = geometricJacobianHolder.getJacobian(jacobianID);

         // check and set frames
         Wrench endEffectorWrench = endEffectorWrenches.get(endEffector);
         endEffectorWrench.changeFrame(jacobian.getJacobianFrame());

         if (VISUALIZE_DESIRED_WRENCHES && registry != null && yoGraphicsListRegistry != null)
         {
            endEffectorPoints.get(endEffector).setToZero(endEffector.getBodyFixedFrame());
            endEffectorPoints.get(endEffector).changeFrame(ReferenceFrame.getWorldFrame());
            yoForcePoints.get(endEffector).set(endEffectorPoints.get(endEffector));
            yoForceVectors.get(endEffector).set(endEffectorWrench.getLinearPart());
         }

         DenseMatrix64F jointEffortMatrix = jointEffortMatrices.get(endEffector);
         jointEffortMatrix.zero();

         DenseMatrix64F jacobianMatrix = jacobian.getJacobianMatrix();
         endEffectorWrench.getMatrix(wrenchMatrix);

         // Apply selection matrix
         tmpEndEffectorWrench.reshape(taskSize, 1);
         tmpJacobianMatrix.reshape(taskSize, jacobianMatrix.numCols);
         CommonOps.mult(endEffectorSelectionMatrix, wrenchMatrix, tmpEndEffectorWrench);
         CommonOps.mult(endEffectorSelectionMatrix, jacobianMatrix, tmpJacobianMatrix);

         // add new selected wrench values to full wrench objective matrix
         int objectiveIndex = 0;
         for (int i = 0; i < Wrench.SIZE; i++)
         {
            if (isObjective.get(0, i) == 1)
            {
               MatrixTools.setMatrixBlock(fullObjectiveWrench, i, 0, tmpEndEffectorWrench, objectiveIndex, 0, 1, 1, 1.0);
               objectiveIndex++;
            }
         }

         // add new jacobian to full objective jacobian
         objectiveIndex = 0;
         for (int i = 0; i < Wrench.SIZE; i++)
         {
            if (isObjective.get(i) == 1)
            {
               for (int j = 0; j < jointsInChain.length; j++)
               {
                  fullJacobianMatrix.set(i, controlledJoints.indexOf(jointsInChain[i]), tmpJacobianMatrix.get(objectiveIndex, j));
               }
               objectiveIndex++;
            }
         }


         /*
         // Compute desired joint torques
         CommonOps.multTransA(tmpJacobianMatrix, tmpEndEffectorWrench, jointEffortMatrix);

         // Write torques to map
         int index = 0;
         for (InverseDynamicsJoint joint : jacobian.getJointsInOrder())
         {
            jointTorques.put(joint, jointEffortMatrix.get(index));
            index++;
         }
         */
      }

      CommonOps.multTransA(fullJacobianMatrix, fullObjectiveWrench, fullEffortMatrix);

      // Write torques to map
      int index = 0;
      for (InverseDynamicsJoint joint : controlledJoints)
      {
         jointTorques.put(joint, fullEffortMatrix.get(index));
         index++;
      }


      virtualModelControlSolutionToPack.setJointTorques(jointTorques);
   }
}
