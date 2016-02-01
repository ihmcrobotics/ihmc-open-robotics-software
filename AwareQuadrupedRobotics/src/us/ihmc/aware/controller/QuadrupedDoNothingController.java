package us.ihmc.aware.controller;

import java.util.Arrays;

import com.google.common.primitives.Booleans;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedDoNothingController implements QuadrupedController
{
   private final FullRobotModel fullRobotModel;

   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the
    * {@link FullRobotModel#getOneDoFJoints()} array.
    */
   private final boolean initialized[];

   public QuadrupedDoNothingController(QuadrupedRuntimeEnvironment environment)
   {
      this.fullRobotModel = environment.getFullRobotModel();
      this.initialized = new boolean[fullRobotModel.getOneDoFJoints().length];
   }

   @Override
   public void onEntry()
   {
      Arrays.fill(initialized, false);

      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         joint.setUnderPositionControl(true);
      }
   }

   @Override
   public QuadrupedControllerEvent process()
   {
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];

         // Only set a desired if the actuator has just come online.
         if (!initialized[i] && joint.isOnline())
         {
            joint.setqDesired(joint.getQ());
            initialized[i] = true;
         }
      }

      return allJointsInitialized() ? QuadrupedControllerEvent.JOINTS_INITIALIZED : null;
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

