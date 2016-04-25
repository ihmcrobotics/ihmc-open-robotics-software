package us.ihmc.aware.controller.force;

import java.util.BitSet;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.JointRole;
import us.ihmc.aware.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedForceJointInitializationController implements QuadrupedForceController
{
   private final SDFFullQuadrupedRobotModel fullRobotModel;

   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the {@link FullRobotModel#getOneDoFJoints()}
    * array.
    */
   private final BitSet initialized;

   public QuadrupedForceJointInitializationController(QuadrupedRuntimeEnvironment environment)
   {
      this.fullRobotModel = environment.getFullRobotModel();
      this.initialized = new BitSet(fullRobotModel.getOneDoFJoints().length);
   }

   @Override
   public void onEntry()
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];

         if (fullRobotModel.getNameForOneDoFJoint(joint).getRole() == JointRole.LEG)
         {
            joint.setUnderPositionControl(false);
         }
      }
   }

   @Override
   public QuadrupedForceControllerEvent process()
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];

         // Only set a desired if the actuator has just come online.
         if (!initialized.get(i) && joint.isOnline())
         {
            joint.setTau(0.0);
            initialized.set(i);
         }
      }

      return allJointsInitialized() ? QuadrupedForceControllerEvent.JOINTS_INITIALIZED : null;
   }

   @Override
   public void onExit()
   {
   }

   private boolean allJointsInitialized()
   {
      return initialized.cardinality() == initialized.length();
   }
}

