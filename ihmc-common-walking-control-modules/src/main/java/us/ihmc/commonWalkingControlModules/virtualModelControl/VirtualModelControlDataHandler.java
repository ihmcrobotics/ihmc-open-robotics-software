package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class VirtualModelControlDataHandler
{
   public final List<RigidBodyBasics> controlledBodies = new ArrayList<>();
   private final Map<RigidBodyBasics, List<OneDoFJoint[]>> jointChainsForControl = new LinkedHashMap<>();
   private final List<OneDoFJoint> controlledJoints = new ArrayList <>();
   public int numberOfControlledJoints = 0;

   private final Map<RigidBodyBasics, Wrench> desiredWrenches = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, FrameVector3D> desiredForces = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, FrameVector3D> desiredTorques = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, DenseMatrix64F> desiredSelectionMatrices = new LinkedHashMap<>();

   public VirtualModelControlDataHandler()
   {
   }

   public void clear()
   {
      desiredForces.clear();
      desiredTorques.clear();
      desiredWrenches.clear();
      desiredSelectionMatrices.clear();
   }

   public void reset()
   {
      controlledBodies.clear();
      jointChainsForControl.clear();
      numberOfControlledJoints = 0;

      clear();
   }

   public void addBodyForControl(RigidBodyBasics bodyForControl)
   {
      if (!controlledBodies.contains(bodyForControl))
      {
         controlledBodies.add(bodyForControl);
         jointChainsForControl.put(bodyForControl, new ArrayList<>());
      }
      else
      {
         PrintTools.warn(this, "Class has already registered " + bodyForControl.getName() + ".");
      }
   }

   public void addJointsForControl(RigidBodyBasics controlledBody, OneDoFJoint[] jointsToUse)
   {
      // check joint order
      int length = jointsToUse.length;
      if (length > 0)
      {
         boolean rightOrder = true;
         if (length > 1)
            MultiBodySystemTools.isAncestor(jointsToUse[1].getPredecessor(), jointsToUse[0].getPredecessor());

         OneDoFJoint[] orderedJointsToUse;
         if (rightOrder)
            orderedJointsToUse = jointsToUse;
         else
         {
            orderedJointsToUse = new OneDoFJoint[length];
            for (int i = 0; i < length; i++)
               orderedJointsToUse[i] = jointsToUse[length - 1 - i];
         }

         jointChainsForControl.get(controlledBody).add(orderedJointsToUse);

         for (int i = 0; i < length; i++)
            if (!controlledJoints.contains(orderedJointsToUse[i]))
               controlledJoints.add(orderedJointsToUse[i]);

         numberOfControlledJoints = controlledJoints.size();
      }
   }

   public void addDesiredWrench(RigidBodyBasics controlledBody, Wrench desiredWrench)
   {
      if (hasBody(controlledBody))
      {
         desiredWrenches.put(controlledBody, desiredWrench);
      }
   }

   public void addDesiredSelectionMatrix(RigidBodyBasics controlledBody, DenseMatrix64F selectionMatrix)
   {
      if (hasBody(controlledBody))
      {
         if (desiredSelectionMatrices.get(controlledBody) != null)
            PrintTools.warn(this, "Class already contains selection matrix for body " + controlledBody.getName() + ". It is being overwritten.");
         desiredSelectionMatrices.put(controlledBody, selectionMatrix);
      }
   }

   public boolean hasBody(RigidBodyBasics controlledBody)
   {
      return controlledBodies.contains(controlledBody) || controlledBody == null;
   }

   public boolean hasWrench(RigidBodyBasics controlledBody)
   {
      return desiredWrenches.get(controlledBody) != null;
   }

   public boolean hasSelectionMatrix(RigidBodyBasics controlledBody)
   {
      return desiredSelectionMatrices.get(controlledBody) != null;
   }

   public int jointsInChain(RigidBodyBasics controlledBody, int chainID)
   {
      return jointChainsForControl.get(controlledBody).get(chainID).length;
   }

   public int numberOfChains(RigidBodyBasics controlledBody)
   {
      return jointChainsForControl.get(controlledBody).size();
   }

   public int indexOfInTree(RigidBodyBasics controlledBody, int chainID, int jointNumberInChain)
   {
      return controlledJoints.indexOf(jointChainsForControl.get(controlledBody).get(chainID)[jointNumberInChain]);
   }

   public List<OneDoFJoint> getControlledJoints()
   {
      return controlledJoints;
   }

   public List<RigidBodyBasics> getControlledBodies()
   {
      return controlledBodies;
   }

   public OneDoFJoint[] getJointsForControl(RigidBodyBasics controlledBody, int chainID)
   {
      return jointChainsForControl.get(controlledBody).get(chainID);
   }

   public Wrench getDesiredWrench(RigidBodyBasics controlledBody)
   {
      return desiredWrenches.get(controlledBody);
   }

   public DenseMatrix64F getDesiredSelectionMatrix(RigidBodyBasics controlledBody)
   {
      return desiredSelectionMatrices.get(controlledBody);
   }
}
