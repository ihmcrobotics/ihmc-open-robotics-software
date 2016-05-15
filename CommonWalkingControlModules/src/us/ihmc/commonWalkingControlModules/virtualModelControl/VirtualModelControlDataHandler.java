package us.ihmc.commonWalkingControlModules.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class VirtualModelControlDataHandler
{
   public final List<RigidBody> controlledBodies = new ArrayList<RigidBody>();
   public int numberOfControlledJoints = 0;

   private final Map<RigidBody, OneDoFJoint[]> jointsForControl = new LinkedHashMap<>();
   private final List<OneDoFJoint> controlledJoints = new ArrayList <>();

   private final Map<RigidBody, Wrench> desiredWrenches = new LinkedHashMap<>();
   private final Map<RigidBody, DenseMatrix64F> desiredSelectionMatrices = new LinkedHashMap<>();

   public VirtualModelControlDataHandler()
   {
   }

   public void reset()
   {
      desiredWrenches.clear();
      desiredSelectionMatrices.clear();
   }

   public void addBodyForControl(RigidBody bodyForControl)
   {
      if (!controlledBodies.contains(bodyForControl))
         controlledBodies.add(bodyForControl);
      else
         PrintTools.warn(this, "Class already contains body " + bodyForControl.getName() + " for control!!");
   }

   public void addJointsForControl(RigidBody controlledBody, OneDoFJoint[] jointsToUse)
   {
      if(jointsForControl.get(controlledBody) != null)
         PrintTools.warn(this, "Class already contains joints for body " + controlledBody.getName() + " to use. These are being overwritten.");
      jointsForControl.put(controlledBody, jointsToUse);

      for (int i = 0; i < jointsToUse.length; i++)
         if (!controlledJoints.contains(jointsToUse[i]))
            controlledJoints.add(jointsToUse[i]);

      numberOfControlledJoints = controlledJoints.size();
   }

   public void addDesiredWrench(RigidBody controlledBody, Wrench desiredWrench)
   {
      if (desiredWrenches.get(controlledBody) != null)
         PrintTools.warn(this, "Class already contains wrench for body " + controlledBody.getName() + ". It is being overwritten.");
      desiredWrenches.put(controlledBody, desiredWrench);
   }

   public void addDesiredSelectionMatrix(RigidBody controlledBody, DenseMatrix64F selectionMatrix)
   {
      if (desiredSelectionMatrices.get(controlledBody) != null)
         PrintTools.warn(this, "Class already contains selection matrix for body " + controlledBody.getName() + ". It is being overwritten.");
      desiredSelectionMatrices.put(controlledBody, selectionMatrix);
   }

   public boolean hasBody(RigidBody controlledBody)
   {
      return controlledBodies.contains(controlledBody) || controlledBody == null;
   }

   public int jointsInChain(RigidBody controlledBody)
   {
      return jointsForControl.get(controlledBody).length;
   }

   public int indexOfInTree(RigidBody controlledBody, int jointNumberInChain)
   {
      return controlledJoints.indexOf(jointsForControl.get(controlledBody)[jointNumberInChain]);
   }

   public List<OneDoFJoint> getControlledJoints()
   {
      return controlledJoints;
   }

   public List<RigidBody> getControlledBodies()
   {
      return controlledBodies;
   }

   public OneDoFJoint[] getJointsForControl(RigidBody controlledBody)
   {
      return jointsForControl.get(controlledBody);
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
