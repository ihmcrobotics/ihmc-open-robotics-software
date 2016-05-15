package us.ihmc.commonWalkingControlModules.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphic;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class VirtualModelController
{
   private final static boolean VISUALIZE_DESIRED_WRENCHES = false;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoVariableRegistry registry;
   private final Map<RigidBody, YoFrameVector> yoForceVectors = new HashMap<>();
   private final Map<RigidBody, YoFramePoint> yoForcePoints = new HashMap<>();
   private final Map<RigidBody, FramePoint> endEffectorPoints = new HashMap<>();

   private final GeometricJacobianHolder geometricJacobianHolder;
   private final RigidBody defaultRootBody;

   private final Map<InverseDynamicsJoint, Double> jointTorques = new HashMap<>();

   private final VirtualModelControlDataHandler vmcDataHandler = new VirtualModelControlDataHandler();

   private final DenseMatrix64F fullJTMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F fullObjectiveWrench = new DenseMatrix64F(0, 0, true); // make it row major
   private final DenseMatrix64F fullEffortMatrix = new DenseMatrix64F(1, 1);

   public VirtualModelController(GeometricJacobianHolder geometricJacobianHolder, RigidBody defaultRootBody, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.geometricJacobianHolder = geometricJacobianHolder;
      this.defaultRootBody = defaultRootBody;
      this.registry = registry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      fullJTMatrix.reshape(0, 0);
      fullObjectiveWrench.reshape(0, 0);
   }

   public void registerEndEffector(RigidBody endEffector)
   {
      registerEndEffector(defaultRootBody, endEffector);
   }

   public void registerEndEffector(RigidBody base, RigidBody endEffector)
   {
      OneDoFJoint[] joints = ScrewTools.createOneDoFJointPath(base, endEffector);
      registerEndEffector(endEffector, base, joints);
   }

   public void registerEndEffector(RigidBody endEffector, RigidBody baseOfControl, OneDoFJoint[] jointsToUse)
   {
      if (!vmcDataHandler.hasBody(endEffector))
      {
         vmcDataHandler.addBodyForControl(endEffector);
         vmcDataHandler.addJointsForControl(endEffector, jointsToUse);

         if (VISUALIZE_DESIRED_WRENCHES && registry != null && yoGraphicsListRegistry != null)
         {
            YoFramePoint forceBase = new YoFramePoint(endEffector.getName() + "ForceBase", ReferenceFrame.getWorldFrame(), registry);
            FramePoint endEffectorPoint = new FramePoint(endEffector.getBodyFixedFrame());
            endEffectorPoint.changeFrame(ReferenceFrame.getWorldFrame());
            endEffectorPoints.put(endEffector, endEffectorPoint);

            forceBase.set(endEffectorPoint);

            YoFrameVector forceVector = new YoFrameVector(endEffector.getName() + "DesiredForce", baseOfControl.getBodyFixedFrame(), registry);

            yoForcePoints.put(endEffector, forceBase);
            yoForceVectors.put(endEffector, forceVector);

            AppearanceDefinition forceAppearance = YoAppearance.AliceBlue();
            YoGraphic forceVisualizer = new YoGraphicVector(endEffector.getName() + "DesiredForce", forceBase, forceVector, 0.05, forceAppearance);
            yoGraphicsListRegistry.registerYoGraphic("forcePointVisualizer", forceVisualizer);
         }
      }
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

      vmcDataHandler.addDesiredWrench(endEffector, wrench);
      vmcDataHandler.addDesiredSelectionMatrix(endEffector, selectionMatrix);
   }

   public void reset()
   {
      vmcDataHandler.reset();
      jointTorques.clear();
   }

   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F tmpWrench = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tmpJMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tmpJTMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F matrixToCopy = new DenseMatrix64F(1, 1);

   public void compute(VirtualModelControlSolution virtualModelControlSolutionToPack)
   {
      matrixToCopy.reshape(0, 0);
      fullJTMatrix.reshape(0, 0);
      fullObjectiveWrench.reshape(0, 0);
      fullEffortMatrix.reshape(vmcDataHandler.numberOfControlledJoints, 1);
      matrixToCopy.zero();
      fullJTMatrix.zero();
      fullObjectiveWrench.zero();
      fullEffortMatrix.zero();

      for (RigidBody endEffector : vmcDataHandler.getControlledBodies())
      {
         DenseMatrix64F endEffectorSelectionMatrix = vmcDataHandler.getDesiredSelectionMatrix(endEffector);
         int taskSize = endEffectorSelectionMatrix.getNumRows();

         // get jacobian
         long jacobianID = geometricJacobianHolder.getOrCreateGeometricJacobian(vmcDataHandler.getJointsForControl(endEffector), defaultRootBody.getBodyFixedFrame());

         // check and set frames
         Wrench endEffectorWrench = vmcDataHandler.getDesiredWrench(endEffector);
         endEffectorWrench.changeFrame(geometricJacobianHolder.getJacobian(jacobianID).getJacobianFrame());

         if (VISUALIZE_DESIRED_WRENCHES && (registry != null) && (yoGraphicsListRegistry != null))
         {
            endEffectorPoints.get(endEffector).setToZero(endEffector.getBodyFixedFrame());
            endEffectorPoints.get(endEffector).changeFrame(ReferenceFrame.getWorldFrame());
            yoForcePoints.get(endEffector).set(endEffectorPoints.get(endEffector));
            yoForceVectors.get(endEffector).set(endEffectorWrench.getLinearPart());
         }

         // Apply selection matrix
         int numberOfJoints = vmcDataHandler.jointsInChain(endEffector);
         tmpWrench.reshape(taskSize, 1);
         tmpJMatrix.reshape(taskSize, numberOfJoints);
         tmpJTMatrix.reshape(numberOfJoints, taskSize);
         endEffectorWrench.getMatrix(wrenchMatrix);
         CommonOps.mult(endEffectorSelectionMatrix, wrenchMatrix, tmpWrench);
         CommonOps.mult(endEffectorSelectionMatrix, geometricJacobianHolder.getJacobian(jacobianID).getJacobianMatrix(), tmpJMatrix);
         CommonOps.transpose(tmpJMatrix, tmpJTMatrix);

         // append wrench to the end of the current objective wrench vector
         int previousSize = fullObjectiveWrench.getNumRows();
         int newSize = previousSize + taskSize;
         fullObjectiveWrench.reshape(newSize, 1, true);
         CommonOps.extract(tmpWrench, 0, taskSize, 0, 1, fullObjectiveWrench, previousSize, 0);

         // insert new jacobian into full objective jacobian
         matrixToCopy.set(fullJTMatrix);
         fullJTMatrix.reshape(vmcDataHandler.numberOfControlledJoints, newSize);
         fullJTMatrix.zero();
         CommonOps.extract(matrixToCopy, 0, matrixToCopy.getNumRows(), 0, matrixToCopy.getNumCols(), fullJTMatrix, 0, 0);
         for (int i = 0; i < numberOfJoints; i++)
         {
            CommonOps.extract(tmpJTMatrix, i, i+1, 0, taskSize, fullJTMatrix, vmcDataHandler.indexOfInTree(endEffector, i), previousSize);
         }
      }

      // compute forces
      CommonOps.mult(fullJTMatrix, fullObjectiveWrench, fullEffortMatrix);

      // Write torques to map
      int index = 0;
      for (InverseDynamicsJoint joint : vmcDataHandler.getControlledJoints())
      {
         jointTorques.put(joint, fullEffortMatrix.get(index));
         index++;
      }

      virtualModelControlSolutionToPack.setJointTorques(jointTorques);
   }
}
