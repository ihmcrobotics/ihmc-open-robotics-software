package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

public class VirtualModelController
{
   private final static boolean USE_SUPER_JACOBIAN = true;
   private final static boolean DISPLAY_GRAVITY_WRENCHES = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();
   private final RigidBodyBasics defaultRootBody;

   private final Map<JointBasics, MutableDouble> jointTorques = new HashMap<>();

   private final VirtualModelControlDataHandler vmcDataHandler = new VirtualModelControlDataHandler();

   private final DMatrixRMaj fullJTMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj fullObjectiveWrench = new DMatrixRMaj(0, 0, true); // make it row major
   private final DMatrixRMaj fullEffortMatrix = new DMatrixRMaj(1, 1);

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", ReferenceFrame.getWorldFrame());
   private final Wrench tempWrench = new Wrench();
   private final DMatrixRMaj tempSelectionMatrix = new DMatrixRMaj(Wrench.SIZE, Wrench.SIZE);

   private final WrenchVisualizer gravityWrenchVisualizer;
   private final Map<RigidBodyBasics, Wrench> gravityWrenchMap = new HashMap<>();

   private final ReferenceFrame centerOfMassFrame;

   private List<RigidBodyBasics> allBodies = new ArrayList<>();

   public VirtualModelController(RigidBodyBasics defaultRootBody, ReferenceFrame centerOfMassFrame, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.defaultRootBody = defaultRootBody;
      this.centerOfMassFrame = centerOfMassFrame;

      for (JointBasics joint : defaultRootBody.childrenSubtreeIterable())
         jointTorques.put(joint, new MutableDouble());

      if (DISPLAY_GRAVITY_WRENCHES)
      {
         JointBasics[] joints = {defaultRootBody.getParentJoint()};
         RigidBodyBasics[] allBodies = MultiBodySystemTools.collectSubtreeSuccessors(joints);
         this.allBodies = Arrays.asList(allBodies);
         for (int i = 0; i < allBodies.length; i++)
            gravityWrenchMap.put(allBodies[i], new Wrench(allBodies[i].getBodyFixedFrame(), ReferenceFrame.getWorldFrame()));
         gravityWrenchVisualizer = new WrenchVisualizer("GravityWrenches", this.allBodies, 10.0, yoGraphicsListRegistry, registry, YoAppearance.Blue(),
                                                        YoAppearance.Blue());
      }
      else
      {
         gravityWrenchVisualizer = null;
      }

      parentRegistry.addChild(registry);
   }

   public void registerControlledBody(RigidBodyBasics controlledBody)
   {
      registerControlledBody(controlledBody, defaultRootBody);
   }

   public void registerControlledBody(RigidBodyBasics controlledBody, RigidBodyBasics baseOfControl)
   {
      OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(baseOfControl, controlledBody);
      if (joints.length > 1)
      {
         if (MultiBodySystemTools.isAncestor(joints[1].getPredecessor(), joints[0].getPredecessor()))
            registerControlledBody(controlledBody, joints);
         else // need to reorder them
         {
            int size = joints.length;
            OneDoFJointBasics[] newJoints = new OneDoFJointBasics[size];
            for (int i = 0; i < size; i++)
            {
               newJoints[i] = joints[size - i - 1];
            }
            registerControlledBody(controlledBody, newJoints);
         }
      }
      else
      {
         registerControlledBody(controlledBody, joints);
      }
   }

   public void registerControlledBody(RigidBodyBasics controlledBody, OneDoFJointBasics[] jointsToUse)
   {
      vmcDataHandler.addBodyForControl(controlledBody);
      vmcDataHandler.addJointsForControl(controlledBody, jointsToUse);
   }

   public void submitControlledBodyVirtualWrench(RigidBodyBasics controlledBody, Wrench wrench)
   {
      submitControlledBodyVirtualWrench(controlledBody, wrench, CommonOps_DDRM.identity(Wrench.SIZE, Wrench.SIZE));
   }

   public void submitControlledBodyVirtualWrench(VirtualWrenchCommand virtualWrenchCommand)
   {
      virtualWrenchCommand.getDesiredWrench(controlFrame, tempWrench);
      virtualWrenchCommand.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      submitControlledBodyVirtualWrench(virtualWrenchCommand.getEndEffector(), tempWrench, tempSelectionMatrix);
   }

   public void submitControlledBodyVirtualWrench(RigidBodyBasics controlledBody, Wrench wrench, DMatrixRMaj selectionMatrix)
   {
      vmcDataHandler.addDesiredWrench(controlledBody, wrench);
      vmcDataHandler.addDesiredSelectionMatrix(controlledBody, selectionMatrix);
   }

   public void reset()
   {
      vmcDataHandler.reset();
   }

   public void clear()
   {
      vmcDataHandler.clear();
   }

   private final DMatrixRMaj wrenchMatrix = new DMatrixRMaj(Wrench.SIZE, 1);
   private final DMatrixRMaj tmpWrench = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj tmpJMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj tmpJTMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj matrixToCopy = new DMatrixRMaj(1, 1);

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

      if (USE_SUPER_JACOBIAN)
      {
         // cycles through all chains controlling all bodies, assembles one problem, and solves it via jacobian transpose
         for (RigidBodyBasics controlledBody : vmcDataHandler.getControlledBodies())
         {
            if (vmcDataHandler.hasWrench(controlledBody) && vmcDataHandler.hasSelectionMatrix(controlledBody))
            {
               int numberOfControlChains = vmcDataHandler.numberOfChains(controlledBody);

               // if it doesn't have any control joints, it's controlled through ground contact forces
               if (numberOfControlChains > 0)
               {
                  // get wrench and selection matrix for current body
                  Wrench wrench = vmcDataHandler.getDesiredWrench(controlledBody);
                  DMatrixRMaj selectionMatrix = vmcDataHandler.getDesiredSelectionMatrix(controlledBody);
                  int taskSize = selectionMatrix.getNumRows();

                  // check and set frames
                  wrench.changeFrame(defaultRootBody.getBodyFixedFrame());
                  wrench.setBodyFrame(controlledBody.getBodyFixedFrame());

                  // apply selection matrix to wrench
                  tmpWrench.reshape(taskSize, 1);
                  wrench.get(wrenchMatrix);
                  CommonOps_DDRM.mult(selectionMatrix, wrenchMatrix, tmpWrench);

                  // change wrench frame for yo variable
                  wrench.changeFrame(ReferenceFrame.getWorldFrame());

                  // append wrench to the end of the current objective wrench vector
                  int previousSize = fullObjectiveWrench.getNumRows();
                  int newSize = previousSize + taskSize;
                  fullObjectiveWrench.reshape(newSize, 1, true);
                  CommonOps_DDRM.extract(tmpWrench, 0, taskSize, 0, 1, fullObjectiveWrench, previousSize, 0);

                  // go through all chains controlling the current body
                  for (int chainID = 0; chainID < numberOfControlChains; chainID++)
                  {
                     // get jacobian
                     geometricJacobianCalculator.clear();
                     geometricJacobianCalculator.setKinematicChain(vmcDataHandler.getJointsForControl(controlledBody, chainID));
                     geometricJacobianCalculator.setJacobianFrame(defaultRootBody.getBodyFixedFrame());
                     geometricJacobianCalculator.reset();

                     // Apply selection matrix to jacobian
                     int numberOfJoints = vmcDataHandler.jointsInChain(controlledBody, chainID);
                     tmpJMatrix.reshape(taskSize, numberOfJoints);
                     tmpJTMatrix.reshape(numberOfJoints, taskSize);
                     CommonOps_DDRM.mult(selectionMatrix, geometricJacobianCalculator.getJacobianMatrix(), tmpJMatrix);
                     CommonOps_DDRM.transpose(tmpJMatrix, tmpJTMatrix);

                     // insert new jacobian into full objective jacobian
                     matrixToCopy.set(fullJTMatrix);
                     fullJTMatrix.reshape(vmcDataHandler.numberOfControlledJoints, newSize);
                     fullJTMatrix.zero();
                     CommonOps_DDRM.extract(matrixToCopy, 0, matrixToCopy.getNumRows(), 0, matrixToCopy.getNumCols(), fullJTMatrix, 0, 0);
                     for (int jointID = 0; jointID < numberOfJoints; jointID++)
                     {
                        CommonOps_DDRM.extract(tmpJTMatrix, jointID, jointID + 1, 0, taskSize, fullJTMatrix,
                                          vmcDataHandler.indexOfInTree(controlledBody, chainID, jointID), previousSize);
                     }
                  }
               }
            }
            else
            {
               LogTools.warn("Do not have a wrench or selection matrix for body " + controlledBody.getName() + ", skipping this body.");
            }
         }

         // compute forces
         CommonOps_DDRM.mult(fullJTMatrix, fullObjectiveWrench, fullEffortMatrix);
      }
      // we do each chain individually
      else
      {
         for (RigidBodyBasics controlledBody : vmcDataHandler.getControlledBodies())
         {
            if (vmcDataHandler.hasWrench(controlledBody) && vmcDataHandler.numberOfChains(controlledBody) > 0)
            {
               Wrench wrench = vmcDataHandler.getDesiredWrench(controlledBody);
               DMatrixRMaj selectionMatrix = vmcDataHandler.getDesiredSelectionMatrix(controlledBody);
               int taskSize = selectionMatrix.getNumRows();

               // check and set frames
               wrench.changeFrame(defaultRootBody.getBodyFixedFrame());
               wrench.setBodyFrame(controlledBody.getBodyFixedFrame());

               // apply selection matrix to wrench
               tmpWrench.reshape(taskSize, 1);
               wrench.get(wrenchMatrix);
               CommonOps_DDRM.mult(selectionMatrix, wrenchMatrix, tmpWrench);

               // get jacobian
               geometricJacobianCalculator.clear();
               geometricJacobianCalculator.setKinematicChain(vmcDataHandler.getJointsForControl(controlledBody, 0));
               geometricJacobianCalculator.setJacobianFrame(defaultRootBody.getBodyFixedFrame());
               geometricJacobianCalculator.reset();

               // Apply selection matrix to jacobian
               int numberOfJoints = vmcDataHandler.jointsInChain(controlledBody, 0);
               tmpJMatrix.reshape(taskSize, numberOfJoints);
               tmpJTMatrix.reshape(numberOfJoints, taskSize);
               CommonOps_DDRM.mult(selectionMatrix, geometricJacobianCalculator.getJacobianMatrix(), tmpJMatrix);
               CommonOps_DDRM.transpose(tmpJMatrix, tmpJTMatrix);

               // get torques
               DMatrixRMaj jointTorques = new DMatrixRMaj(tmpJTMatrix.getNumRows(), 1);
               CommonOps_DDRM.mult(tmpJTMatrix, tmpWrench, jointTorques);

               wrench.changeFrame(ReferenceFrame.getWorldFrame());

               // put into full thing
               for (int j = 0; j < vmcDataHandler.jointsInChain(controlledBody, 0); j++)
                  CommonOps_DDRM.extract(jointTorques, j, j + 1, 0, 1, fullEffortMatrix, vmcDataHandler.indexOfInTree(controlledBody, 0, j), 0);
            }
         }
      }

      // Write torques to map
      int index = 0;
      for (JointBasics joint : vmcDataHandler.getControlledJoints())
      {
         jointTorques.get(joint).setValue(fullEffortMatrix.get(index));
         index++;
      }

      if (DISPLAY_GRAVITY_WRENCHES)
      {
         for (RigidBodyBasics rigidBody : allBodies)
         {
            Wrench gravityWrench = gravityWrenchMap.get(rigidBody);
            gravityWrench.set(new FrameVector3D(ReferenceFrame.getWorldFrame(), 0, 0, 0),
                              new FrameVector3D(ReferenceFrame.getWorldFrame(), 0, 0, -9.81 * rigidBody.getInertia().getMass()));
         }
         gravityWrenchVisualizer.visualize(gravityWrenchMap);
      }

      virtualModelControlSolutionToPack.setJointTorques(fullEffortMatrix);
   }
}
