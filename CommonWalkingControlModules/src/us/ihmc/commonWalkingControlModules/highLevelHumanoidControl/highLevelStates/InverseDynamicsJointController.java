package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.util.statemachines.State;

public class InverseDynamicsJointController extends State<HighLevelState>
{
   private final TwistCalculator twistCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   
   private final RigidBody chest;
   
   private final InverseDynamicsJoint[] allJoints;
   private final Wrench gravityWrench;
   
   public InverseDynamicsJointController(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, double gravityZ, CommonWalkingReferenceFrames referenceFrames)
   {
      super(HighLevelState.INVERSE_DYNAMICS_JOINT_CONTROL);
      
      this.twistCalculator = twistCalculator;
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
   
      RigidBody elevator = fullRobotModel.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      
      allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      chest = fullRobotModel.getChest();
      gravityWrench = new Wrench(chest.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());
      
   }

   @Override
   public void doAction()
   {
      inverseDynamicsCalculator.reset();
      for(InverseDynamicsJoint joint : allJoints)
      {
         joint.setDesiredAccelerationToZero();
      }
      
      inverseDynamicsCalculator.setExternalWrench(chest, gravityWrench);
      
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
