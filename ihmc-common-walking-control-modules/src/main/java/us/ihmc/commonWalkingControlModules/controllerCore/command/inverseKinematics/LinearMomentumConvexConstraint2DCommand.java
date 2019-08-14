package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Vector2D;

/**
 * {@link LinearMomentumConvexConstraint2DCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link LinearMomentumConvexConstraint2DCommand} is to notify the inverse
 * kinematics optimization module to constrain the x and y components of the linear momentum to
 * remain inside a convex set of linear constraints.
 * </p>
 * <p>
 * This command can notably be used to enforce the center of mass to remain within the support
 * polygon.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class LinearMomentumConvexConstraint2DCommand implements InverseKinematicsCommand<LinearMomentumConvexConstraint2DCommand>
{
   /**
    * The convex set of constraints to apply on the linear part of the momentum.
    * <p>
    * The resulting set of constraints enforces the linear momentum to remain on the right side of each
    * line formed by each pair of consecutive vertices.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>This command is ignored when there is less than 2 vertices.
    * <li>When there is only 2 vertices, a line constraint is applied, i.e. the linear momentum is
    * constrained to remain on the right side of the line.
    * </ul>
    * </p>
    * <p>
    * It is assumed that the given vertices shape a convex polygon.
    * </p>
    */
   private final RecyclingArrayList<Vector2D> linearMomentumConstraintVertices = new RecyclingArrayList<>(Vector2D::new);

   public void clear()
   {
      linearMomentumConstraintVertices.clear();
   }

   @Override
   public void set(LinearMomentumConvexConstraint2DCommand other)
   {
      clear();
      for (int i = 0; i < other.linearMomentumConstraintVertices.size(); i++)
         linearMomentumConstraintVertices.add().set(other.linearMomentumConstraintVertices.get(i));
   }

   /**
    * Adds a vertex to the convex set used to constrain the linear momentum.
    * <p>
    * The data represents the linear momentum at the center of mass expressed in world's coordinates.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>This command is ignored when there is less than 2 vertices.
    * <li>When there is only 2 vertices, a line constraint is applied, i.e. the linear momentum is
    * constrained to remain on the right side of the line.
    * </ul>
    * </p>
    * 
    * @return the new vertex to be set.
    */
   public Vector2D addLinearMomentumConstraintVertex()
   {
      return linearMomentumConstraintVertices.add();
   }

   public List<Vector2D> getLinearMomentumConstraintVertices()
   {
      return linearMomentumConstraintVertices;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.MOMENTUM_CONVEX_CONSTRAINT;
   }
}
