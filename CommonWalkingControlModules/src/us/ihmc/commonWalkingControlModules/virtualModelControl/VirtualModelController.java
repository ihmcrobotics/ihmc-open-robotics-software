package us.ihmc.commonWalkingControlModules.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphic;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.HashMap;
import java.util.Map;

public class VirtualModelController
{
   private final static boolean VISUALIZE_DESIRED_WRENCHES = false;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoVariableRegistry registry;
   private final Map<RigidBody, YoFrameVector> yoForceVectors = new HashMap<>();
   private final Map<RigidBody, YoFramePoint> yoForcePoints = new HashMap<>();
   private final Map<RigidBody, FramePoint> controlledBodyPoints = new HashMap<>();

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

   public void registerControlledBody(RigidBody controlledBody)
   {
      registerControlledBody(controlledBody, defaultRootBody);
   }

   public void registerControlledBody(RigidBody controlledBody, RigidBody baseOfControl)
   {
      OneDoFJoint[] joints = ScrewTools.createOneDoFJointPath(baseOfControl, controlledBody);
      registerControlledBody(controlledBody, joints);
   }

   public void registerControlledBody(RigidBody controlledBody, OneDoFJoint[] jointsToUse)
   {
      vmcDataHandler.addBodyForControl(controlledBody);
      vmcDataHandler.addJointsForControl(controlledBody, jointsToUse);
   }

   public void submitControlledBodyVirtualWrench(RigidBody controlledBody, Wrench wrench)
   {
      submitControlledBodyVirtualWrench(controlledBody, wrench, new CommonOps().identity(Wrench.SIZE, Wrench.SIZE));
   }

   public void submitControlledBodyVirtualWrench(VirtualWrenchCommand virtualWrenchCommand)
   {
      submitControlledBodyVirtualWrench(virtualWrenchCommand.getControlledBody(), virtualWrenchCommand.getVirtualWrench(), virtualWrenchCommand.getSelectionMatrix());
   }

   public void submitControlledBodyVirtualWrench(RigidBody controlledBody, Wrench wrench, DenseMatrix64F selectionMatrix)
   {
      wrench.changeBodyFrameAttachedToSameBody(controlledBody.getBodyFixedFrame());

      vmcDataHandler.addDesiredWrench(controlledBody, wrench);
      vmcDataHandler.addDesiredSelectionMatrix(controlledBody, selectionMatrix);
   }


   public void reset()
   {
      vmcDataHandler.reset();
      jointTorques.clear();
   }

   public void clear()
   {
      vmcDataHandler.clear();
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

      for (RigidBody controlledBody : vmcDataHandler.getControlledBodies())
      {
         if (vmcDataHandler.hasWrench(controlledBody) && vmcDataHandler.hasSelectionMatrix(controlledBody))
         {
            vmcDataHandler.loadBody(controlledBody);

            int taskSize = vmcDataHandler.selectionMatrix.getNumRows();
            int numberOfControlChains = vmcDataHandler.numberOfChains(controlledBody);

            // check and set frames
            vmcDataHandler.wrench.changeFrame(defaultRootBody.getBodyFixedFrame());

            if (VISUALIZE_DESIRED_WRENCHES && (registry != null) && (yoGraphicsListRegistry != null))
            {
               controlledBodyPoints.get(controlledBody).setToZero(controlledBody.getBodyFixedFrame());
               controlledBodyPoints.get(controlledBody).changeFrame(ReferenceFrame.getWorldFrame());
               yoForcePoints.get(controlledBody).set(controlledBodyPoints.get(controlledBody));
               yoForceVectors.get(controlledBody).set(vmcDataHandler.wrench.getLinearPart());
            }

            // apply selection matrix to wrench
            tmpWrench.reshape(taskSize, 1);
            vmcDataHandler.wrench.getMatrix(wrenchMatrix);
            CommonOps.mult(vmcDataHandler.selectionMatrix, wrenchMatrix, tmpWrench);

            // append wrench to the end of the current objective wrench vector
            int previousSize = fullObjectiveWrench.getNumRows();
            int newSize = previousSize + taskSize;
            fullObjectiveWrench.reshape(newSize, 1, true);
            CommonOps.extract(tmpWrench, 0, taskSize, 0, 1, fullObjectiveWrench, previousSize, 0);

            for (int chainID = 0; chainID < numberOfControlChains; chainID++)
            {
               // get jacobian
               long jacobianID = geometricJacobianHolder .getOrCreateGeometricJacobian(vmcDataHandler.getJointsForControl(controlledBody, chainID), defaultRootBody.getBodyFixedFrame());

               // Apply selection matrix to jacobian
               int numberOfJoints = vmcDataHandler.jointsInChain(controlledBody, chainID);
               tmpJMatrix.reshape(taskSize, numberOfJoints);
               tmpJTMatrix.reshape(numberOfJoints, taskSize);
               CommonOps.mult(vmcDataHandler.selectionMatrix, geometricJacobianHolder.getJacobian(jacobianID).getJacobianMatrix(), tmpJMatrix);
               CommonOps.transpose(tmpJMatrix, tmpJTMatrix);

               // insert new jacobian into full objective jacobian
               matrixToCopy.set(fullJTMatrix);
               fullJTMatrix.reshape(vmcDataHandler.numberOfControlledJoints, newSize);
               fullJTMatrix.zero();
               CommonOps.extract(matrixToCopy, 0, matrixToCopy.getNumRows(), 0, matrixToCopy.getNumCols(), fullJTMatrix, 0, 0);
               for (int jointID = 0; jointID < numberOfJoints; jointID++)
               {
                  CommonOps.extract(tmpJTMatrix, jointID, jointID + 1, 0, taskSize, fullJTMatrix, vmcDataHandler.indexOfInTree(controlledBody, chainID, jointID),
                        previousSize);
               }
            }
         }
         else
         {
            PrintTools.warn(this, "Do not have a wrench or selection matrix for body " + controlledBody.getName() + ", skipping this body.");
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
