package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

/**
 * The objective of this class is to help the passing commands between two instances of the same
 * robot.
 * <p>
 * The main use-case is for passing commands from one thread to another. In such context, each
 * thread has its own instance of the robot and the corresponding reference frame tree.
 * </p>
 * <p>
 * The main challenge when passing commands is to retrieve the joints, rigid-bodies, and reference
 * frames properly.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class CrossRobotCommandResolver
{
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;
   private final RigidBodyHashCodeResolver rigidBodyHashCodeResolver;
   private final JointHashCodeResolver jointHashCodeResolver;

   public CrossRobotCommandResolver(ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver, RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                                    JointHashCodeResolver jointHashCodeResolver)
   {
      this.referenceFrameHashCodeResolver = referenceFrameHashCodeResolver;
      this.rigidBodyHashCodeResolver = rigidBodyHashCodeResolver;
      this.jointHashCodeResolver = jointHashCodeResolver;
   }

   public void resolveCenterOfPressureCommand(CenterOfPressureCommand in, CenterOfPressureCommand out)
   {
      out.setConstraintType(in.getConstraintType());
      int contactingRigidBodyHashCode = in.getContactingRigidBody().hashCode();
      out.setContactingRigidBody(rigidBodyHashCodeResolver.castAndGetRigidBody(contactingRigidBodyHashCode));
      int weightFrameHashCode = in.getWeight().getReferenceFrame().hashCode();
      out.getWeight().setIncludingFrame(referenceFrameHashCodeResolver.getReferenceFrame(weightFrameHashCode), in.getWeight());
      int desiredCoPFrameHashCode = in.getDesiredCoP().getReferenceFrame().hashCode();
      out.getDesiredCoP().setIncludingFrame(referenceFrameHashCodeResolver.getReferenceFrame(desiredCoPFrameHashCode), in.getDesiredCoP());
   }
}
