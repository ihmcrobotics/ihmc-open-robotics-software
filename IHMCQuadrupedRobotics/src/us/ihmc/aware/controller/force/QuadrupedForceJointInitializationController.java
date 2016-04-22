package us.ihmc.aware.controller.force;

import java.util.Arrays;
import java.util.List;

import com.google.common.primitives.Booleans;
import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.aware.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedForceJointInitializationController implements QuadrupedForceController
{
   private final SDFFullQuadrupedRobotModel fullRobotModel;

   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the
    * {@link FullRobotModel#getOneDoFJoints()} array.
    */
   private final boolean initialized[];

   public QuadrupedForceJointInitializationController(QuadrupedRuntimeEnvironment environment)
   {
      this.fullRobotModel = environment.getFullRobotModel();
      this.initialized = new boolean[fullRobotModel.getOneDoFJoints().length];
   }

   @Override
   public void onEntry()
   {
      Arrays.fill(initialized, false);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         List<OneDoFJoint> joints = fullRobotModel.getLegOneDoFJoints(robotQuadrant);
         for (int i = 0; i < joints.size(); i++)
         {
            joints.get(i).setUnderPositionControl(false);
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

