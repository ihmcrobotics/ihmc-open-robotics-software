package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.lists.DenseMatrixArrayList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * {@link JointTorqueCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link JointTorqueCommand} is to notify the virtual model
 * control module that the given set of joints are to also include the joint torque
 * during the next control tick.
 * </p>
 * <p>
 * It is usually the result of the {@link OneDoFJointFeedbackController}.
 * </p>
 * 
 * @author Robert Griffin
 *
 */
public class JointTorqueCommand implements VirtualModelControlCommand<JointTorqueCommand>
{
   /**
    * Initial capacity for the lists used in this command. It is to prevent memory allocation at the
    * beginning of the control.
    */
   private final int initialCapacity = 15;
   /**
    * The list of the joint names ordered as the {@link #joints} list. It is useful for when passing
    * the command to another thread that uses a different instance of the same robot
    */
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   /** The list of joints for which desired torques are assigned. */
   private final List<InverseDynamicsJoint> joints = new ArrayList<>(initialCapacity);
   /**
    * The list of the desired torques for each joint. The list follows the same ordering as
    * the {@link #joints} list. Each {@link DenseMatrix64F} in this list, is a N-by-1 vector where N
    * is equal to the number of degrees of freedom of the joint it is associated with.
    */
   private final DenseMatrixArrayList desiredTorques = new DenseMatrixArrayList(initialCapacity);

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public JointTorqueCommand()
   {
      clear();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(JointTorqueCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         jointNames.add(other.jointNames.get(i));
      }
      desiredTorques.set(other.desiredTorques);
   }

   /**
    * Clears the internal memory. This action does not generate garbage, the data is simply 'marked'
    * as cleared but the memory will be recycled when setting up that command after calling this
    * method.
    */
   public void clear()
   {
      joints.clear();
      jointNames.clear();
      desiredTorques.clear();
   }

   /**
    * Adds a joint to be controlled to this command.
    * <p>
    * The joint is added at the last position, i.e. at the index
    * {@code i == this.getNumberOfJoints()}.
    * </p>
    * 
    * @param joint the joint to be controlled.
    * @param desiredTorque the joint torque to be achieved in the next control tick.
    */
   public void addJoint(OneDoFJoint joint, double desiredTorque)
   {
      joints.add(joint);
      jointNames.add(joint.getName());
      DenseMatrix64F jointDesiredTorque = desiredTorques.add();
      jointDesiredTorque.reshape(1, 1);
      jointDesiredTorque.set(0, 0, desiredTorque);
   }

   /**
    * Adds a joint to be controlled to this command.
    * <p>
    * The joint is added at the last position, i.e. at the index
    * {@code i == this.getNumberOfJoints()}.
    * </p>
    * 
    * @param joint the joint to be controlled.
    * @param desiredTorque the joint torque to be achieved in the next control tick. It
    *           is expected to be a N-by-1 vector with N equal to
    *           {@code joint.getDegreesOfFreedom()}. Not modified.
    * @throws RuntimeException if the {@code desiredTorque} is not a N-by-1 vector.
    */
   public void addJoint(InverseDynamicsJoint joint, DenseMatrix64F desiredTorque)
   {
      checkConsistency(joint, desiredTorque);
      joints.add(joint);
      jointNames.add(joint.getName());
      desiredTorques.add().set(desiredTorque);
   }

   /**
    * Edits the desired torque for the {@code jointIndex}<sup>th</sup> joint in this command.
    * <p>
    * This method does not change the weight associated with the joint.
    * </p>
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @param desiredTorque the joint torque to be achieved in the next control tick.
    */
   public void setOneDoFJointDesiredTorque(int jointIndex, double desiredTorque)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredTorques.get(jointIndex).reshape(1, 1);
      desiredTorques.get(jointIndex).set(0, 0, desiredTorque);
   }

   /**
    * Edits the desired torque for the {@code jointIndex}<sup>th</sup> joint in this command.
    * <p>
    * This method does not change the weight associated with the joint.
    * </p>
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @param desiredTorque the joint torque to be achieved in the next control tick. It
    *           is expected to be a N-by-1 vector with N equal to
    *           {@code joint.getDegreesOfFreedom()}. Not modified.
    * @throws RuntimeException if the {@code desiredTorque} is not a N-by-1 vector.
    */
   public void setDesiredTorque(int jointIndex, DenseMatrix64F desiredTorque)
   {
      checkConsistency(joints.get(jointIndex), desiredTorque);
      desiredTorques.get(jointIndex).set(desiredTorque);
   }

   /**
    * This method changes the internal references to each joint in this command to the joints
    * contained in the map from joint name to {@code InverseDynamicsJoint}.
    * <p>
    * This is useful when passing the command to another thread which may hold onto a different
    * instance of the same robot.
    * </p>
    * 
    * @param nameToJointMap the map from joint names to the new joints that this command should
    *           refer to. Not modified.
    * @throws RuntimeException if the given map does not have all this command's joints.
    */
   public void retrieveJointsFromName(Map<String, ? extends InverseDynamicsJoint> nameToJointMap)
   {
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         String jointName = jointNames.get(i);
         InverseDynamicsJoint newJointReference = nameToJointMap.get(jointName);
         if (newJointReference == null)
            throw new RuntimeException("The given map is missing the joint: " + jointName);
         joints.set(i, newJointReference);
      }
   }

   private void checkConsistency(InverseDynamicsJoint joint, DenseMatrix64F desiredTorque)
   {
      MathTools.checkEquals(joint.getDegreesOfFreedom(), desiredTorque.getNumRows());
   }

   /**
    * Gets the number of joints registered in this command.
    * 
    * @return the number of joints in this command.
    */
   public int getNumberOfJoints()
   {
      return joints.size();
   }

   /**
    * Gets the internal reference to this command's joint list.
    * 
    * @return the list of joints to be controlled.
    */
   public List<InverseDynamicsJoint> getJoints()
   {
      return joints;
   }

   /**
    * Gets the {@code jointIndex}<sup>th</th> joint in this command.
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @return one of the joints to be controlled.
    */
   public InverseDynamicsJoint getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   /**
    * Gets the name of the {@code jointIndex}<sup>th</sup> joint in this command.
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @return the joint name.
    */
   public String getJointName(int jointIndex)
   {
      return jointNames.get(jointIndex);
   }

   /**
    * Gets the desired torque associated with the {@code jointIndex}<sup>th</sup> joint of
    * this command.
    *
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @return the N-by-1 desired torque where N is the joint number of degrees of freedom.
    */
   public DenseMatrix64F getDesiredTorque(int jointIndex)
   {
      return desiredTorques.get(jointIndex);
   }

   /**
    * Gets the internal reference to this command's joint desired torque list.
    * 
    * @return the list of desired torques to be achieved by the joints.
    */
   public DenseMatrixArrayList getDesiredTorques()
   {
      return desiredTorques;
   }

   /**
    * {@inheritDoc}
    * 
    * @return {@link ControllerCoreCommandType#TASKSPACE}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINTSPACE;
   }

   public String toString()
   {
      String ret = getClass().getSimpleName() + ": ";
      for (int i = 0; i < joints.size(); i++)
      {
         ret += joints.get(i).getName();
         if (i < joints.size() - 1)
            ret += ", ";
         else
            ret += ".";
      }
      return ret;
   }
}
