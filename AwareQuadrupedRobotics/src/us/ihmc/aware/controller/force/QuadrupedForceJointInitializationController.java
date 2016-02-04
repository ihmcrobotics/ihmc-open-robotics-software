package us.ihmc.aware.controller.force;

import java.util.Arrays;

import com.google.common.primitives.Booleans;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.JointRole;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedForceJointInitializationController implements QuadrupedForceController
{
   private final FullRobotModel fullRobotModel;
   private final QuadrupedRobotParameters parameters;

   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the
    * {@link FullRobotModel#getOneDoFJoints()} array.
    */
   private final boolean initialized[];

   public QuadrupedForceJointInitializationController(QuadrupedRuntimeEnvironment environment,
         QuadrupedRobotParameters parameters)
   {
      this.fullRobotModel = environment.getFullRobotModel();
      this.parameters = parameters;
      this.initialized = new boolean[fullRobotModel.getOneDoFJoints().length];
   }

   @Override
   public void onEntry()
   {
      Arrays.fill(initialized, false);

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         // Initialize neck joints to position control mode and all other joints to force control mode.
         // TODO: This is a hack until we have separate neck/body controllers.
         QuadrupedJointName jointName = parameters.getJointMap().getJointNameForSDFName(joint.getName());
         JointRole jointRole = parameters.getJointMap().getJointRole(jointName.toString());

         joint.setUnderPositionControl(jointRole == JointRole.NECK);
      }
   }

   @Override
   public QuadrupedForceControllerEvent process()
   {
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];

         // Only set a desired if the actuator has just come online.
         if (!initialized[i] && joint.isOnline())
         {
            joint.setTau(0.0);
            initialized[i] = true;
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
      return !Booleans.contains(initialized, false);
   }
}

