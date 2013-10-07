package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.util.statemachines.State;

public class InverseDynamicsJointController extends State<HighLevelState>
{
//   private final ReferenceFrame centerOfMassFrame;
   private final TwistCalculator twistCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   
   private final InverseDynamicsJoint[] allJoints;
   
   public InverseDynamicsJointController(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, double gravityZ, CommonWalkingReferenceFrames referenceFrames)
   {
      super(HighLevelState.INVERSE_DYNAMICS_JOINT_CONTROL);
      
//      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      this.twistCalculator = twistCalculator;
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
   
//      RigidBody elevator = fullRobotModel.getElevator();
//      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      
      allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
//      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());
      
      
   }

   @Override
   public void doAction()
   {
      inverseDynamicsCalculator.reset();
      for(InverseDynamicsJoint joint : allJoints)
      {
         joint.setDesiredAccelerationToZero();
      }
      
      twistCalculator.compute();
      inverseDynamicsCalculator.compute();
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

}
