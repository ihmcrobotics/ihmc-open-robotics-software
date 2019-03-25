package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * A command that contains instructions about the wrench exerted by a body in contact with the
 * environment. The command requires that the body specified is configured as a contactable body in
 * the controller core. It is also required that the body is set to be in contact with the
 * environment using a {@link PlaneContactStateCommand}.
 * <p>
 * This command can limit the wrench (or parts of it) exerted by the body in the form of a
 * constraint, or add an objective to the QP to prefer a certain wrench at that body.
 * </p>
 */
public class ContactWrenchCommand implements InverseDynamicsCommand<ContactWrenchCommand>, VirtualModelControlCommand<ContactWrenchCommand>
{
   /**
    * The constraint type for this command.
    * <p>
    * Specifies whether the wrench provided here needs to be achieved exactly, as an objective, or if
    * the wrench is a limit on the allowed wrench that is exerted by the rigid body.
    * </p>
    */
   private ConstraintType constraintType;

   /**
    * The body which exerts the wrench on the environment.
    */
   private RigidBodyBasics rigidBody;

   /**
    * The wrench used in this command needs to have its body frame match the body frame of the rigid
    * body exerting the wrench.
    */
   private final Wrench wrench = new Wrench();

   /**
    * The weight matrix to be used in the optimization only if the constraint type is set to
    * {@link ConstraintType#OBJECTIVE}.
    */
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();

   /**
    * A selection matrix that allows the user to specify what components of the provided wrench should
    * be used in the controller core.
    */
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   public ContactWrenchCommand()
   {
   }

   public ContactWrenchCommand(ConstraintType constraintType)
   {
      setConstraintType(constraintType);
   }

   public void setRigidBody(RigidBodyBasics rigidBody)
   {
      this.rigidBody = rigidBody;
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   public Wrench getWrench()
   {
      return wrench;
   }

   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   @Override
   public void set(ContactWrenchCommand other)
   {
      this.constraintType = other.constraintType;
      this.rigidBody = other.rigidBody;
      this.wrench.setIncludingFrame(other.wrench);
      this.weightMatrix.set(other.weightMatrix);
      this.selectionMatrix.set(other.selectionMatrix);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.CONTACT_WRENCH;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ContactWrenchCommand)
      {
         ContactWrenchCommand other = (ContactWrenchCommand) object;
         if (constraintType != other.constraintType)
            return false;
         if (rigidBody != other.rigidBody)
            return false;
         if (!wrench.equals(other.wrench))
            return false;
         if (!weightMatrix.equals(other.weightMatrix))
            return false;
         if (!selectionMatrix.equals(other.selectionMatrix))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": constraint: " + constraintType + ", body: " + rigidBody + ", wrench: " + wrench + ", weight: " + weightMatrix
            + ", selection: " + selectionMatrix;
   }
}
