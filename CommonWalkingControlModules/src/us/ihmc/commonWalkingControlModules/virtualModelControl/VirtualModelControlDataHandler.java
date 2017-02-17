package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.tools.io.printing.PrintTools;

public class VirtualModelControlDataHandler
{
   public final List<RigidBody> controlledBodies = new ArrayList<RigidBody>();
   private final Map<RigidBody, List<OneDoFJoint[]>> jointChainsForControl = new LinkedHashMap<>();
   private final List<OneDoFJoint> controlledJoints = new ArrayList <>();
   public int numberOfControlledJoints = 0;

   private final Map<RigidBody, Wrench> desiredWrenches = new LinkedHashMap<>();
   private final Map<RigidBody, DenseMatrix64F> desiredSelectionMatrices = new LinkedHashMap<>();

   public VirtualModelControlDataHandler()
   {
   }

   public void clear()
   {
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

   public void addBodyForControl(RigidBody bodyForControl)
   {
      if (!controlledBodies.contains(bodyForControl))
      {
         controlledBodies.add(bodyForControl);
         jointChainsForControl.put(bodyForControl, new ArrayList<OneDoFJoint[]>());
      }
      else
      {
         PrintTools.warn(this, "Class has already registered " + bodyForControl.getName() + ".");
      }
   }

   public void addJointsForControl(RigidBody controlledBody, OneDoFJoint[] jointsToUse)
   {
      // check joint order
      int length = jointsToUse.length;
      if (length > 0)
      {
         boolean rightOrder = true;
         if (length > 1)
            ScrewTools.isAncestor(jointsToUse[1].getPredecessor(), jointsToUse[0].getPredecessor());

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

   public void addDesiredWrench(RigidBody controlledBody, Wrench desiredWrench)
   {
      if (hasBody(controlledBody))
      {
         if (desiredWrenches.get(controlledBody) != null)
            PrintTools.warn(this, "Class already contains wrench for body " + controlledBody.getName() + ". It is being overwritten.");
         desiredWrenches.put(controlledBody, desiredWrench);
      }
   }

   public void addDesiredSelectionMatrix(RigidBody controlledBody, DenseMatrix64F selectionMatrix)
   {
      if (hasBody(controlledBody))
      {
         if (desiredSelectionMatrices.get(controlledBody) != null)
            PrintTools.warn(this, "Class already contains selection matrix for body " + controlledBody.getName() + ". It is being overwritten.");
         desiredSelectionMatrices.put(controlledBody, selectionMatrix);
      }
   }

   public boolean hasBody(RigidBody controlledBody)
   {
      return controlledBodies.contains(controlledBody) || controlledBody == null;
   }

   public boolean hasWrench(RigidBody controlledBody)
   {
      return desiredWrenches.get(controlledBody) != null;
   }

   public boolean hasSelectionMatrix(RigidBody controlledBody)
   {
      return desiredSelectionMatrices.get(controlledBody) != null;
   }

   public int jointsInChain(RigidBody controlledBody, int chainID)
   {
      return jointChainsForControl.get(controlledBody).get(chainID).length;
   }

   public int numberOfChains(RigidBody controlledBody)
   {
      return jointChainsForControl.get(controlledBody).size();
   }

   public int indexOfInTree(RigidBody controlledBody, int chainID, int jointNumberInChain)
   {
      return controlledJoints.indexOf(jointChainsForControl.get(controlledBody).get(chainID)[jointNumberInChain]);
   }

   public List<OneDoFJoint> getControlledJoints()
   {
      return controlledJoints;
   }

   public List<RigidBody> getControlledBodies()
   {
      return controlledBodies;
   }

   public List<OneDoFJoint[]> getJointChainsForControl(RigidBody controlledBody)
   {
      return jointChainsForControl.get(controlledBody);
   }

   public OneDoFJoint[] getJointsForControl(RigidBody controlledBody, int chainID)
   {
      return jointChainsForControl.get(controlledBody).get(chainID);
   }

   public Wrench getDesiredWrench(RigidBody controlledBody)
   {
      return desiredWrenches.get(controlledBody);
   }

   public DenseMatrix64F getDesiredSelectionMatrix(RigidBody controlledBody)
   {
      return desiredSelectionMatrices.get(controlledBody);
   }
}
