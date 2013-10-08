package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.statemachines.State;

public class InverseDynamicsJointController extends State<HighLevelState>
{
   private final TwistCalculator twistCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   
   private final FullRobotModel fullRobotModel;
   private final RigidBody chest;
   
   private final InverseDynamicsJoint[] allJoints;
   private final Wrench comWrench;
   private final Wrench rootJointWrench = new Wrench();
   
   private final ReferenceFrame centerOfMassFrame;
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameVector rootJointForce = new YoFrameVector("rootJointForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector rootJointTorque = new YoFrameVector("rootJointTorque", ReferenceFrame.getWorldFrame(), registry);
   
   private final double totalMass, gravityZ;
   
   
   public InverseDynamicsJointController(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, double gravityZ, CommonWalkingReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      super(HighLevelState.INVERSE_DYNAMICS_JOINT_CONTROL);
      
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      
      this.fullRobotModel = fullRobotModel;
      
      this.gravityZ = gravityZ;
      
      this.twistCalculator = twistCalculator;
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);
   
      RigidBody elevator = fullRobotModel.getElevator();
      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      
      allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      chest = fullRobotModel.getChest();
      comWrench = new Wrench(chest.getBodyFixedFrame(), chest.getBodyFixedFrame());
      
      parentRegistry.addChild(registry);
   }

   @Override
   public void doAction()
   {
      inverseDynamicsCalculator.reset();
      for(InverseDynamicsJoint joint : allJoints)
      {
         joint.setDesiredAccelerationToZero();
      }
      
      comWrench.set(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());
      comWrench.changeFrame(chest.getBodyFixedFrame());

      inverseDynamicsCalculator.setExternalWrench(chest, comWrench);
      
      twistCalculator.compute();
      inverseDynamicsCalculator.compute();
      
      fullRobotModel.getRootJoint().packWrench(rootJointWrench);
      rootJointWrench.changeFrame(ReferenceFrame.getWorldFrame());
      
      FrameVector force = rootJointWrench.getLinearPartAsFrameVectorCopy();
      FrameVector torque = rootJointWrench.getAngularPartAsFrameVectorCopy();
      
      rootJointForce.set(force);
      rootJointTorque.set(torque);
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
