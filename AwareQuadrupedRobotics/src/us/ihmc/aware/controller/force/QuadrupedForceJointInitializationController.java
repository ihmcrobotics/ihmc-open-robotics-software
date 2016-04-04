package us.ihmc.aware.controller.force;

import java.util.Arrays;

import com.google.common.primitives.Booleans;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedForceJointInitializationController implements QuadrupedForceController
{
   private final SDFFullRobotModel fullRobotModel;
   private final QuadrupedJointNameMap jointMap;

   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the
    * {@link FullRobotModel#getOneDoFJoints()} array.
    */
   private final boolean initialized[];

   public QuadrupedForceJointInitializationController(QuadrupedRuntimeEnvironment environment, QuadrupedJointNameMap jointMap)
   {
      this.fullRobotModel = environment.getFullRobotModel();
      this.jointMap = jointMap;
      this.initialized = new boolean[fullRobotModel.getOneDoFJoints().length];
   }

   @Override
   public void onEntry()
   {
      Arrays.fill(initialized, false);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < jointMap.getLegJointNames().length; i++)
         {
            // initialize leg joint mode to force control
            LegJointName legJointName = jointMap.getLegJointNames()[i];
            String jointName = jointMap.getLegJointName(robotQuadrant, legJointName);
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
            joint.setUnderPositionControl(false);
         }
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

