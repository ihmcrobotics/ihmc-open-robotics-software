package us.ihmc.quadrupedRobotics.controller;

import com.google.common.primitives.Booleans;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedDoNothingController extends QuadrupedController implements QuadrupedJointInitializer
{
   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the
    * {@link FullRobotModel#getOneDoFJoints()} array.
    */
   private final BooleanYoVariable initialized[];

   private final OneDoFJoint[] oneDoFJoints;   
   
   public QuadrupedDoNothingController(FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      super(QuadrupedControllerState.DO_NOTHING);

      YoVariableRegistry registry = new YoVariableRegistry("QuadrupedDoNothingController");
      oneDoFJoints = fullRobotModel.getOneDoFJoints();
      
      this.initialized = new BooleanYoVariable[oneDoFJoints.length];
      for(int i = 0; i < initialized.length; i++)
      {
         initialized[i] = new BooleanYoVariable(oneDoFJoints[i].getName() + "_initialized", registry);
      }
      
      parentRegistry.addChild(registry);
         
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];

         // Only set a desired if the actuator has just come online.
         
         if(!joint.isEnabled())
         {
            joint.setqDesired(joint.getQ());
         }
         else if(!initialized[i].getBooleanValue())
         {
            joint.setqDesired(joint.getQ());
            initialized[i].set(true);;
         }
         
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (OneDoFJoint joint : oneDoFJoints)
      {
         joint.setUnderPositionControl(true);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public RobotMotionStatus getMotionStatus()
   {
      return RobotMotionStatus.STANDING;
   }

   @Override
   public boolean allJointsInitialized()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         if(!initialized[i].getBooleanValue())
         {
            return false;
         }
      }
      
      return true;
   }
}

