package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.robotics.weightMatrices.SolverWeightLevels.HARD_CONSTRAINT;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

/**
 * {@link MomentumRateCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link MomentumRateCommand} is to notify the inverse dynamics optimization
 * module to get the robot to achieve the desired rate of change of momentum during the next control
 * tick.
 * </p>
 * <p>
 * This command can notably be used to control the center of mass acceleration by using the linear
 * part of the rate of change of momentum.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class MomentumRateCommand implements InverseDynamicsCommand<MomentumRateCommand>, VirtualModelControlCommand<MomentumRateCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private int commandId;
   /**
    * It defines the 6-components of the desired rate of change of momentum to achieve for the next
    * control.
    * <p>
    * More precisely, it represents the desired rate of change of momentum at the center of mass while
    * the data is expressed in world frame.
    * </p>
    */
   private final DMatrixRMaj momentumRateOfChange = new DMatrixRMaj(Momentum.SIZE, 1);
   /**
    * Weights on a per component basis to use in the optimization function. A higher weight means
    * higher priority of this task.
    */
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   /**
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) for the rate of change of
    * momentum that are to be controlled. It is initialized such that the controller will by default
    * control all the DoFs.
    * <p>
    * If the selection frame is not set, it is assumed that the selection frame is equal to the
    * centroidal frame.
    * </p>
    */
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   /**
    * Whether this command should be treated considering the full robot or not. Default value is
    * {@code true}.
    * <p>
    * Typically, the momentum rate command is used to control the center of mass of the whole robot, in
    * which case {@code considerAllJoints} should be {@code true}.
    * </p>
    * <p>
    * If the control of only part of the robot's momentum rate is desired, then this should be
    * {@code false} and {@link #jointSelection} should contain the list of joint of interest. For
    * instance, this command can be used to regulate the rate of change of angular momentum generated
    * by only the arms. In such scenario, {@code considerAllJoints} is {@code false} and
    * {@code jointSelection} contains only the arm joints.
    * </p>
    */
   private boolean considerAllJoints = true;
   /**
    * When {@link #considerAllJoints} is {@code true}, this list is used to select the part of the
    * robot to be considered.
    */
   private final List<JointReadOnly> jointSelection = new ArrayList<>();

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public MomentumRateCommand()
   {
      weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
      weightMatrix.setLinearWeights(0.0, 0.0, 0.0);
      considerAllJoints = true;
      jointSelection.clear();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(MomentumRateCommand other)
   {
      commandId = other.commandId;
      weightMatrix.set(other.weightMatrix);
      selectionMatrix.set(other.selectionMatrix);
      momentumRateOfChange.set(other.momentumRateOfChange);
      considerAllJoints = other.considerAllJoints;
      jointSelection.clear();
      for (int i = 0; i < other.jointSelection.size(); i++)
         jointSelection.add(other.jointSelection.get(i));
   }

   /**
    * Copies all the fields of the given {@link MomentumRateCommand} into this except for the desired
    * rate of change of momentum.
    *
    * @param command the command to copy the properties from. Not modified.
    */
   public void setProperties(SpatialAccelerationCommand command)
   {
      commandId = command.getCommandId();
      weightMatrix.set(command.getWeightMatrix());
      command.getSelectionMatrix(selectionMatrix);
   }

   /**
    * Sets the desired momentum rate to submit for the optimization to zero.
    */
   public void setMomentumRateToZero()
   {
      momentumRateOfChange.zero();
   }

   /**
    * Sets the desired rate of change of momentum to submit for the optimization.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while the
    * data is expressed in world frame.
    * </p>
    * 
    * @param momentumRateOfChange the desired momentum rate at the center of mass expressed in world
    *                             frame. Not modified.
    */
   public void setMomentumRate(DMatrixRMaj momentumRateOfChange)
   {
      this.momentumRateOfChange.set(momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of momentum to submit for the optimization.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while the
    * data is expressed in world frame.
    * </p>
    * 
    * @param angularMomentumRateOfChange the desired angular momentum rate at the center of mass
    *                                    expressed in world frame. Not modified.
    * @param linearMomentumRateOfChange  the desired linear momentum rate in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code angularMomentumRateOfChange} or
    *                                         {@code linearMomentumRateOfChange} is not expressed in
    *                                         world frame.
    */
   public void setMomentumRate(FrameVector3DReadOnly angularMomentumRateOfChange, FrameVector3DReadOnly linearMomentumRateOfChange)
   {
      angularMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);
      linearMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      angularMomentumRateOfChange.get(0, momentumRateOfChange);
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of angular momentum to submit for the optimization and sets the
    * linear part to zero.
    * <p>
    * It is assumed to represent the desired rate of change of momentum at the center of mass while the
    * data is expressed in world frame.
    * </p>
    * 
    * @param angularMomentumRateOfChange the desired angular momentum rate at the center of mass
    *                                    expressed in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code angularMomentumRateOfChange} is not expressed
    *                                         in world frame.
    */
   public void setAngularMomentumRate(FrameVector3DReadOnly angularMomentumRateOfChange)
   {
      angularMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      momentumRateOfChange.zero();
      angularMomentumRateOfChange.get(0, momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of linear momentum to submit for the optimization and sets the
    * angular part to zero.
    * 
    * @param linearMomentumRateOfChange the desired linear momentum rate in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code linearMomentumRateOfChange} is not expressed in
    *                                         world frame.
    */
   public void setLinearMomentumRate(FrameVector3DReadOnly linearMomentumRateOfChange)
   {
      linearMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      momentumRateOfChange.zero();
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   /**
    * Sets the desired rate of change of linear momentum in the XY-plane to submit for the optimization
    * and sets the angular part and the z component of the linear part to zero.
    * 
    * @param linearMomentumRateOfChange the desired linear momentum rate in world frame. Not modified.
    * @throws ReferenceFrameMismatchException if {@code linearMomentumRateOfChange} is not expressed in
    *                                         world frame.
    */
   public void setLinearMomentumXYRate(FrameVector2DReadOnly linearMomentumRateOfChange)
   {
      linearMomentumRateOfChange.checkReferenceFrameMatch(worldFrame);

      momentumRateOfChange.zero();
      linearMomentumRateOfChange.get(3, momentumRateOfChange);
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the 6-by-6 identity matrix.
    * <p>
    * This specifies that the 6 degrees of freedom for the rate of change of momentum are to be
    * controlled.
    * </p>
    */
   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.resetSelection();
   }

   /**
    * Convenience method that sets up the selection matrix such that only the linear part of this
    * command will be considered in the optimization.
    */
   public void setSelectionMatrixForLinearControl()
   {
      selectionMatrix.setToLinearSelectionOnly();
   }

   /**
    * Convenience method that sets up the selection matrix by disabling the angular part of this
    * command and applying the given selection matrix to the linear part.
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the control frame.
    * </p>
    * 
    * @param linearSelectionMatrix the selection matrix to apply to the linear part of this command.
    *                              Not modified.
    */
   public void setSelectionMatrixForLinearControl(SelectionMatrix3D linearSelectionMatrix)
   {
      selectionMatrix.clearSelection();
      selectionMatrix.setLinearPart(linearSelectionMatrix);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the x and y components of the
    * linear part of this command will be considered in the optimization.
    */
   public void setSelectionMatrixForLinearXYControl()
   {
      selectionMatrix.setToLinearSelectionOnly();
      selectionMatrix.selectLinearZ(false);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the angular part of this
    * command will be considered in the optimization.
    */
   public void setSelectionMatrixForAngularControl()
   {
      selectionMatrix.setToAngularSelectionOnly();
   }

   /**
    * Sets this command's selection matrix to the given one.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) for the rate of change
    * momentum that are to be controlled. It is initialized such that the controller will by default
    * control all the DoFs.
    * </p>
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the centroidal frame.
    * </p>
    * 
    * @param selectionMatrix the selection matrix to copy data from. Not modified.
    */
   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   /**
    * Sets all the weights to {@link SolverWeightLevels#HARD_CONSTRAINT} such that this command will be
    * treated as a hard constraint.
    * <p>
    * This is usually undesired as with improper commands setup as hard constraints the optimization
    * problem can simply be impossible to solve.
    * </p>
    */
   public void setAsHardConstraint()
   {
      setWeight(HARD_CONSTRAINT);
   }

   /**
    * Sets the weight to use in the optimization problem.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param weight the weight value to use for this command.
    */
   public void setWeight(double weight)
   {
      weightMatrix.setAngularWeights(weight, weight, weight);
      weightMatrix.setLinearWeights(weight, weight, weight);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weight to use for the angular part of this command. Not modified.
    * @param linear  the weight to use for the linear part of this command. Not modified.
    */
   public void setWeight(double angular, double linear)
   {
      weightMatrix.setLinearWeights(linear, linear, linear);
      weightMatrix.setAngularWeights(angular, angular, angular);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angularX the weight to use for the x-axis of the angular part of this command.
    * @param angularY the weight to use for the y-axis of the angular part of this command.
    * @param angularZ the weight to use for the z-axis of the angular part of this command.
    * @param linearX  the weight to use for the x-axis of the linear part of this command.
    * @param linearY  the weight to use for the y-axis of the linear part of this command.
    * @param linearZ  the weight to use for the z-axis of the linear part of this command.
    */
   public void setWeights(double angularX, double angularY, double angularZ, double linearX, double linearY, double linearZ)
   {
      weightMatrix.setAngularWeights(angularX, angularY, angularZ);
      weightMatrix.setLinearWeights(linearX, linearY, linearZ);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param weightMatrix weight matrix holding the weights to use for each component of the desired
    *                     acceleration. Not modified.
    */
   public void setWeights(WeightMatrix6D weightMatrix)
   {
      this.weightMatrix.set(weightMatrix);
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param angular the weights to use for the angular part of this command. Not modified.
    */
   public void setAngularWeight(double angular)
   {
      weightMatrix.setAngularWeights(angular, angular, angular);
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weights to use for the angular part of this command. Not modified.
    */
   public void setAngularWeights(Tuple3DReadOnly angular)
   {
      weightMatrix.setAngularWeights(angular.getX(), angular.getY(), angular.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param linear the weights to use for the angular part of this command. Not modified.
    */
   public void setLinearWeights(Tuple3DReadOnly linear)
   {
      weightMatrix.setLinearWeights(linear.getX(), linear.getY(), linear.getZ());
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weights to use for the angular part of this command. Not modified.
    * @param linear  the weights to use for the linear part of this command. Not modified.
    */
   public void setWeights(Tuple3DReadOnly angular, Tuple3DReadOnly linear)
   {
      setLinearWeights(linear);
      setAngularWeights(angular);
   }

   /**
    * Sets the weights to use in the optimization problem for each rotational degree of freedom to
    * zero.
    * <p>
    * By doing so, the angular part of this command will be ignored during the optimization. Note that
    * it is less expensive to call {@link #setSelectionMatrixForLinearControl()} as by doing so the
    * angular part will not be submitted to the optimization.
    * </p>
    */
   public void setAngularWeightsToZero()
   {
      weightMatrix.setAngularWeights(0.0, 0.0, 0.0);
   }

   /**
    * Sets the weights to use in the optimization problem for each translational degree of freedom to
    * zero.
    * <p>
    * By doing so, the linear part of this command will be ignored during the optimization. Note that
    * it is less expensive to call {@link #setSelectionMatrixForAngularControl()} as by doing so the
    * linear part will not be submitted to the optimization.
    * </p>
    */
   public void setLinearWeightsToZero()
   {
      weightMatrix.setLinearWeights(0.0, 0.0, 0.0);
   }

   /**
    * Indicates whether this command should be treated considering the full robot or only part of it.
    * 
    * @param considerAllJoints {@code true} to consider all the robot, {@code false} to consider only
    *                          part of it. Default is {@code true}.
    */
   public void setConsiderAllJoints(boolean considerAllJoints)
   {
      this.considerAllJoints = considerAllJoints;
   }

   /**
    * Only used when {@link #isConsiderAllJoints()} is {@code false}, adds a joint to consider when
    * treating this command.
    * 
    * @param joint another joint to consider with this command.
    */
   public void addJointToSelection(JointReadOnly joint)
   {
      jointSelection.add(joint);
   }

   /**
    * Only used when {@link #isConsiderAllJoints()} is {@code false}, adds joints to consider when
    * treating this command.
    * 
    * @param joints other joints to consider with this command.
    */
   public void addJointsToSelection(JointReadOnly[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         jointSelection.add(joints[i]);
      }
   }

   /**
    * Finds if this command is to be considered as a hard constraint during the optimization.
    * <p>
    * This command is considered to be a hard constraint if at least one of the weights is equal to
    * {@link SolverWeightLevels#HARD_CONSTRAINT}.
    * </p>
    * 
    * @return {@code true} if this command should be considered as a hard constraint, {@code false} is
    *         it should be part of the optimization objective.
    */
   public boolean isHardConstraint()
   {
      return weightMatrix.containsHardConstraint();
   }

   /**
    * Packs the weights to use for this command in {@code weightMatrixToPack} as follows:
    * 
    * <pre>
    *     / w0 0  0  0  0  0  \
    *     | 0  w1 0  0  0  0  |
    * W = | 0  0  w2 0  0  0  |
    *     | 0  0  0  w3 0  0  |
    *     | 0  0  0  0  w4 0  |
    *     \ 0  0  0  0  0  w5 /
    * </pre>
    * <p>
    * The three first weights (w0, w1, w2) are the weights to use for the angular part of this command.
    * The three others (w3, w4, w5) are for the linear part.
    * </p>
    * <p>
    * The packed matrix is a 6-by-6 matrix independently from the selection matrix.
    * </p>
    * 
    * @param weightMatrixToPack the weight matrix to use in the optimization. The given matrix is
    *                           reshaped to ensure proper size. Modified.
    */
   public void getWeightMatrix(DMatrixRMaj weightMatrixToPack)
   {
      weightMatrixToPack.reshape(Momentum.SIZE, Momentum.SIZE);
      weightMatrix.getFullWeightMatrixInFrame(worldFrame, weightMatrixToPack);
   }

   /**
    * Returns the weight matrix used in this command:
    *
    * @return the reference to the weights to use with this command.
    */
   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   /**
    * Gets the 6-by-6 selection matrix expressed in the given {@code destinationFrame} to use with this
    * command.
    * 
    * @param destinationFrame      the reference frame in which the selection matrix should be
    *                              expressed in.
    * @param selectionMatrixToPack the dense-matrix in which the selection matrix of this command is
    *                              stored in. Modified.
    */
   public void getSelectionMatrix(ReferenceFrame destinationFrame, DMatrixRMaj selectionMatrixToPack)
   {
      selectionMatrix.getCompactSelectionMatrixInFrame(destinationFrame, selectionMatrixToPack);
   }

   /**
    * Packs the value of the selection matrix carried by this command into the given
    * {@code selectionMatrixToPack}.
    * 
    * @param selectionMatrixToPack the selection matrix to pack.
    */
   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix);
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   /**
    * Gets the reference to the desired rate of change of momentum carried by this command.
    * <p>
    * It represents the desired rate of change of momentum at the center of mass while the data is
    * expressed in world frame.
    * </p>
    * 
    * @return the desired rate of change of momentum.
    */
   public DMatrixRMaj getMomentumRate()
   {
      return momentumRateOfChange;
   }

   /**
    * Packs the value of the desired rate of change of momentum into two frame vectors.
    * 
    * @param angularPartToPack frame vector to pack the desired rate of change of angular momentum at
    *                          the center of mass. Modified.
    * @param linearPartToPack  frame vector to pack the desired rate of change of linear momentum.
    *                          Modified.
    */
   public void getMomentumRate(FrameVector3DBasics angularPartToPack, FrameVector3DBasics linearPartToPack)
   {
      angularPartToPack.setIncludingFrame(worldFrame, 0, momentumRateOfChange);
      linearPartToPack.setIncludingFrame(worldFrame, 3, momentumRateOfChange);
   }

   /**
    * Whether the full robot is to be considered when treating this command.
    * 
    * @return {@code true} if the full robot is to be considered, {@code false} otherwise in which case
    *         {@link #getJointSelection()} is used to select the part of the robot that is to be
    *         considered.
    */
   public boolean isConsiderAllJoints()
   {
      return considerAllJoints;
   }

   /**
    * Only used when {@link #isConsiderAllJoints()} is {@code false}, specifies which part of the robot
    * is to be considered when treating this command.
    * 
    * @return the joint selection.
    */
   public List<JointReadOnly> getJointSelection()
   {
      return jointSelection;
   }

   /**
    * {@inheritDoc}
    * 
    * @return {@link ControllerCoreCommandType#TASKSPACE}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.MOMENTUM;
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MomentumRateCommand)
      {
         MomentumRateCommand other = (MomentumRateCommand) object;

         if (commandId != other.commandId)
            return false;
         if (!MatrixFeatures_DDRM.isEquals(momentumRateOfChange, other.momentumRateOfChange))
            return false;
         if (!weightMatrix.equals(other.weightMatrix))
            return false;
         if (!selectionMatrix.equals(other.selectionMatrix))
            return false;
         if (considerAllJoints != other.considerAllJoints)
            return false;
         if (!jointSelection.equals(other.jointSelection))
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
      String stringOfMomentumRate = EuclidCoreIOTools.getStringOf("(", ")", ",", momentumRateOfChange.getData());
      return getClass().getSimpleName() + ": momentum rate: " + stringOfMomentumRate + ", selection matrix = " + selectionMatrix;
   }
}
