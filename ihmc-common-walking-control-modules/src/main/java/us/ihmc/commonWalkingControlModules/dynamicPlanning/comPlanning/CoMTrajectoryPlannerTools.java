package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.List;

public class CoMTrajectoryPlannerTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double minDuration = 1.0e-5;
   public static final double sufficientlyLarge = 1.0e10;
   public static final double sufficientlyLongTime = 1.0e2;
   public static final double sufficientlyLargeThird = Math.pow(1.0e10, 1.0 / 3.0);

   public static void computeVRPWaypoints(double nominalCoMHeight, double gravityZ, double omega, FrameVector3DReadOnly currentCoMVelocity,
                                          List<? extends ContactStateProvider> contactSequence, RecyclingArrayList<FramePoint3D> startVRPPositionsToPack,
                                          RecyclingArrayList<FramePoint3D> endVRPPositionsToPack)
   {
      computeVRPWaypoints(nominalCoMHeight, gravityZ, omega, currentCoMVelocity, contactSequence, startVRPPositionsToPack, endVRPPositionsToPack, true);
   }

   public static void computeVRPWaypoints(double nominalCoMHeight, double gravityZ, double omega, FrameVector3DReadOnly currentCoMVelocity,
                                          List<? extends ContactStateProvider> contactSequence, RecyclingArrayList<FramePoint3D> startVRPPositionsToPack,
                                          RecyclingArrayList<FramePoint3D> endVRPPositionsToPack, boolean adjustWaypointHeightForHeightChange)
   {
      startVRPPositionsToPack.clear();
      endVRPPositionsToPack.clear();

      double initialHeightVelocity = currentCoMVelocity.getZ();
      double finalHeightVelocity;

      for (int i = 0; i < contactSequence.size() - 1; i++)
      {
         ContactStateProvider contactStateProvider = contactSequence.get(i);
         boolean finalContact = i == contactSequence.size() - 1;
         ContactStateProvider nextContactStateProvider = null;
         if (!finalContact)
            nextContactStateProvider = contactSequence.get(i + 1);

         FramePoint3D start = startVRPPositionsToPack.add();
         FramePoint3D end = endVRPPositionsToPack.add();

         start.set(contactStateProvider.getCopStartPosition());
         start.addZ(nominalCoMHeight);
         end.set(contactStateProvider.getCopEndPosition());
         end.addZ(nominalCoMHeight);

         if (adjustWaypointHeightForHeightChange)
         {
            double duration = contactStateProvider.getTimeInterval().getDuration();
            duration = Math.signum(duration) * Math.max(Math.abs(duration), minDuration);
            if (!contactStateProvider.getContactState().isLoadBearing())
            {
               finalHeightVelocity = initialHeightVelocity - gravityZ * duration;
            }
            else
            {
               if (!finalContact && !nextContactStateProvider.getContactState().isLoadBearing())
               { // next is a jump, current one is load bearing
                  ContactStateProvider nextNextContactStateProvider = contactSequence.get(i + 2);
                  double heightBeforeJump = contactStateProvider.getCopEndPosition().getZ();
                  double finalHeightAfterJump = nextNextContactStateProvider.getCopStartPosition().getZ();

                  double heightChangeWhenJumping = finalHeightAfterJump - heightBeforeJump;
                  double durationOfJump = nextContactStateProvider.getTimeInterval().getDuration();

                  /* delta z = v0 T - 0.5 g T^2
                   * v0 =  delta z / T + 0.5 g T**/
                  finalHeightVelocity = heightChangeWhenJumping / durationOfJump + 0.5 * gravityZ * durationOfJump;
               }
               else
               { // next is is load bearing, current is load bearing.
                  finalHeightVelocity = 0.0;
               }
            }

            // offset the height VRP waypoint based on the desired velocity change
            double heightVelocityChange = finalHeightVelocity - initialHeightVelocity;
            double offset = heightVelocityChange / (MathTools.square(omega) * duration);
            start.subZ(offset);
            end.subZ(offset);

            initialHeightVelocity = finalHeightVelocity;
         }
      }

      ContactStateProvider contactStateProvider = contactSequence.get(contactSequence.size() - 1);

      FramePoint3D start = startVRPPositionsToPack.add();
      FramePoint3D end = endVRPPositionsToPack.add();

      start.set(contactStateProvider.getCopStartPosition());
      start.addZ(nominalCoMHeight);
      end.set(contactStateProvider.getCopEndPosition());
      end.addZ(nominalCoMHeight);
   }

   /**
    * <p> Sets the continuity constraint on the initial CoM position. This DOES result in a initial discontinuity on the desired DCM location,
    * coming from a discontinuity on the desired CoM Velocity. </p>
    * <p> This constraint should be used for the initial position of the center of mass to properly initialize the trajectory. </p>
    * <p> Recall that the equation for the center of mass is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p>
    *    This constraint defines
    * </p>
    * <p>
    *    x<sub>0</sub>(0) = x<sub>d</sub>,
    * </p>
    * <p>
    *    substituting in the coefficients into the constraint matrix.
    * </p>
    * @param centerOfMassLocationForConstraint x<sub>d</sub> in the above equations
    */
   public static void addCoMPositionConstraint(FramePoint3DReadOnly centerOfMassLocationForConstraint, double omega, double time, int sequenceId, int rowStart,
                                               DMatrixRMaj constraintMatrixToPack, DMatrixRMaj xObjectiveMatrixToPack,
                                               DMatrixRMaj yObjectiveMatrixToPack, DMatrixRMaj zObjectiveMatrixToPack)
   {
      centerOfMassLocationForConstraint.checkReferenceFrameMatch(worldFrame);

      time = Math.min(time, sufficientlyLongTime);

      int colStart = 6 * sequenceId;
      constraintMatrixToPack.set(rowStart, colStart, getCoMPositionFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, colStart + 1, getCoMPositionSecondCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, colStart + 2, getCoMPositionThirdCoefficientTimeFunction(time));
      constraintMatrixToPack.set(rowStart, colStart + 3, getCoMPositionFourthCoefficientTimeFunction(time));
      constraintMatrixToPack.set(rowStart, colStart + 4, getCoMPositionFifthCoefficientTimeFunction(time));
      constraintMatrixToPack.set(rowStart, colStart + 5, getCoMPositionSixthCoefficientTimeFunction());

      xObjectiveMatrixToPack.add(rowStart, 0, centerOfMassLocationForConstraint.getX());
      yObjectiveMatrixToPack.add(rowStart, 0, centerOfMassLocationForConstraint.getY());
      zObjectiveMatrixToPack.add(rowStart, 0, centerOfMassLocationForConstraint.getZ());
   }

   /**
    * <p> Sets a constraint on the desired DCM position. This constraint is useful for constraining the terminal location of the DCM trajectory. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> and the center of mass velocity is defined by </p>
    * <p>
    *    d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    *    &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    *     2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>
    * </p>
    * <p>
    *    This constraint is then combining these two, saying
    * </p>
    * <p> x<sub>i</sub>(t<sub>i</sub>) + 1 / &omega; d/dt x<sub>i</sub>(t<sub>i</sub>) = &xi;<sub>d</sub>,</p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId i in the above equations
    * @param time t<sub>i</sub> in the above equations
    * @param desiredDCMPosition desired DCM location. &xi;<sub>d</sub> in the above equations.
    */
   public static void addDCMPositionConstraint(int sequenceId, int rowStart, double time, double omega, FramePoint3DReadOnly desiredDCMPosition,
                                               DMatrixRMaj constraintMatrixToPack, DMatrixRMaj xObjectiveMatrixToPack,
                                               DMatrixRMaj yObjectiveMatrixToPack, DMatrixRMaj zObjectiveMatrixToPack)
   {
      desiredDCMPosition.checkReferenceFrameMatch(worldFrame);

      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      // add constraints on terminal DCM position
      constraintMatrixToPack.set(rowStart, startIndex, getDCMPositionFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, startIndex + 1, getDCMPositionSecondCoefficientTimeFunction());
      constraintMatrixToPack.set(rowStart, startIndex + 2, getDCMPositionThirdCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, startIndex + 3, getDCMPositionFourthCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, startIndex + 4, getDCMPositionFifthCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, startIndex + 5, getDCMPositionSixthCoefficientTimeFunction());

      xObjectiveMatrixToPack.add(rowStart, 0, desiredDCMPosition.getX());
      yObjectiveMatrixToPack.add(rowStart, 0, desiredDCMPosition.getY());
      zObjectiveMatrixToPack.add(rowStart, 0, desiredDCMPosition.getZ());
   }

   /**
    * <p> Adds a constraint for the desired VRP position.</p>
    * <p> Recall that the VRP is defined as </p>
    * <p> v<sub>i</sub>(t<sub>i</sub>) =  c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * (c<sub>4,i</sub> - 6/&omega;<sup>2</sup> c<sub>2,i</sub>) t<sub>i</sub> - 2/&omega; c<sub>3,i</sub> + c<sub>5,i</sub></p>.
    * <p> This constraint then says </p>
    * <p> v<sub>i</sub>(t<sub>i</sub>) = J v<sub>d</sub> </p>
    * <p> where J is a Jacobian that maps from a vector of desired VRP waypoints to the constraint form, and </p>
    * <p> v<sub>d,j</sub> = v<sub>r</sub> </p>
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointPositionIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPPosition reference VRP position, v<sub>r</sub> in the above equations.
    */
   public static void addVRPPositionConstraint(int sequenceId, int constraintNumber, int vrpWaypointPositionIndex, double time, double omega,
                                               FramePoint3DReadOnly desiredVRPPosition, DMatrixRMaj constraintMatrixToPack,
                                               DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack,
                                               DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj vrpWaypointJacobianToPack)
   {
      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      desiredVRPPosition.checkReferenceFrameMatch(worldFrame);

      constraintMatrixToPack.set(constraintNumber, startIndex + 0, CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 1, CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintNumber, startIndex + 2, CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 3, CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 4, CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 5, CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction());

      vrpWaypointJacobianToPack.set(constraintNumber, vrpWaypointPositionIndex, 1.0);

      xObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getX());
      yObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getY());
      zObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getZ());
   }

   /**
    * <p> Adds a constraint for the desired VRP velocity.</p>
    * <p> Recall that the VRP velocity is defined as </p>
    * <p> d/dt v<sub>i</sub>(t<sub>i</sub>) =  3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> + 2 c<sub>3,i</sub> t<sub>i</sub> +
    * (c<sub>4,i</sub> - 6/&omega;<sup>2</sup> c<sub>2,i</sub>).
    * <p> This constraint then says </p>
    * <p> d/dt v<sub>i</sub>(t<sub>i</sub>) = J v<sub>d</sub> </p>
    * <p> where J is a Jacobian that maps from a vector of desired VRP waypoints to the constraint form, and </p>
    * <p> v<sub>d,j</sub> = d/dt v<sub>r</sub> </p>
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointVelocityIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPVelocity reference VRP veloctiy, d/dt v<sub>r</sub> in the above equations.
    */
   public static void addVRPVelocityConstraint(int sequenceId, int constraintRow, int vrpWaypointVelocityIndex, double omega, double time,
                                               FrameVector3DReadOnly desiredVRPVelocity, DMatrixRMaj constraintMatrixToPack,
                                               DMatrixRMaj xObjectiveMatrixToPack, DMatrixRMaj yObjectiveMatrixToPack,
                                               DMatrixRMaj zObjectiveMatrixToPack, DMatrixRMaj vrpWaypointJacobianToPack)
   {
      int startIndex = 6 * sequenceId;

      desiredVRPVelocity.checkReferenceFrameMatch(worldFrame);

      constraintMatrixToPack.set(constraintRow, startIndex + 0, CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, startIndex + 1, CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, startIndex + 2, CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintRow, startIndex + 3, CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintRow, startIndex + 4, CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, startIndex + 5, CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction());

      vrpWaypointJacobianToPack.set(constraintRow, vrpWaypointVelocityIndex, 1.0);

      xObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getX());
      yObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getY());
      zObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getZ());
   }

   /**
    * <p> Set a continuity constraint on the CoM position at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    *    c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    *    c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> x<sub>i-1</sub>(T<sub>i-1</sub>) = x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   public static void addCoMPositionContinuityConstraint(int previousSequence, int nextSequence, int constraintRow, double omega, double previousDuration,
                                                         DMatrixRMaj constraintMatrixToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintRow, previousStartIndex, getCoMPositionFirstCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 1, getCoMPositionSecondCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 2, getCoMPositionThirdCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 3, getCoMPositionFourthCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 4, getCoMPositionFifthCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 5, getCoMPositionSixthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, nextStartIndex, -getCoMPositionFirstCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 1, -getCoMPositionSecondCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 2, -getCoMPositionThirdCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 3, -getCoMPositionFourthCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 4, -getCoMPositionFifthCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 5, -getCoMPositionSixthCoefficientTimeFunction());
   }

   /**
    * <p> Set a continuity constraint on the CoM velocity at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    *    &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    *     2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> d / dt x<sub>i-1</sub>(T<sub>i-1</sub>) = d / dt x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   public static void addCoMVelocityContinuityConstraint(int previousSequence, int nextSequence, int constraintRow, double omega, double previousDuration,
                                                         DMatrixRMaj constraintMatrixToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintRow, previousStartIndex, getCoMVelocityFirstCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 1, getCoMVelocitySecondCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 2, getCoMVelocityThirdCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 3, getCoMVelocityFourthCoefficientTimeFunction(previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 4, getCoMVelocityFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 5, getCoMVelocitySixthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, nextStartIndex, -getCoMVelocityFirstCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 1, -getCoMVelocitySecondCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 2, -getCoMVelocityThirdCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 3, -getCoMVelocityFourthCoefficientTimeFunction(0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 4, -getCoMVelocityFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 5, -getCoMVelocitySixthCoefficientTimeFunction());
   }

   /**
    * <p> Adds a constraint for the CoM trajectory to have an acceleration equal to gravity at time t.</p>
    * <p> Recall that the CoM acceleration is defined as </p>
    * d<sup>2</sup> / dt<sup>2</sup> x<sub>i</sub>(t<sub>i</sub>) = &omega;<sup>2</sup> c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> +
    * &omega;<sup>2</sup> c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 6 c<sub>2,i</sub> t<sub>i</sub> + 2 c<sub>3,i</sub>
    * <p> This constraint then states that </p>
    * <p> d<sup>2</sup> / dt<sup>2</sup> x<sub>i</sub>(t<sub>i</sub>) = -g, </p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainCoMAccelerationToGravity(int sequenceId, int constraintRow, double omega, double time, double gravityZ,
                                                        DMatrixRMaj constraintMatrixToPack, DMatrixRMaj zObjectiveMatrixToPack)
   {
      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintRow, startIndex, getCoMAccelerationFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintRow, startIndex + 1, getCoMAccelerationSecondCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintRow, startIndex + 2, getCoMAccelerationThirdCoefficientTimeFunction(time));
      constraintMatrixToPack.set(constraintRow, startIndex + 3, getCoMAccelerationFourthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, startIndex + 4, getCoMAccelerationFifthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, startIndex + 5, getCoMAccelerationSixthCoefficientTimeFunction());

      zObjectiveMatrixToPack.set(constraintRow, 0, -Math.abs(gravityZ));
   }

   /**
    * <p> Adds a constraint for the CoM trajectory to have a jerk equal to 0.0 at time t.</p>
    * <p> Recall that the CoM jerk is defined as </p>
    * d<sup>3</sup> / dt<sup>3</sup> x<sub>i</sub>(t<sub>i</sub>) = &omega;<sup>3</sup> c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    * &omega;<sup>3</sup> c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 6 c<sub>2,i</sub>
    * <p> This constraint then states that </p>
    * <p> d<sup>3</sup> / dt<sup>3</sup> x<sub>i</sub>(t<sub>i</sub>) = 0.0, </p>
    * <p> substituting in the appropriate coefficients. </p>
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainCoMJerkToZero(double time, double omega, int sequenceId, int rowStart, DMatrixRMaj matrixToPack)
   {
      time = Math.min(time, sufficientlyLongTime);

      int colStart = 6 * sequenceId;
      matrixToPack.set(rowStart, colStart, getCoMJerkFirstCoefficientTimeFunction(omega, time));
      matrixToPack.set(rowStart, colStart + 1, getCoMJerkSecondCoefficientTimeFunction(omega, time));
      matrixToPack.set(rowStart, colStart + 2, getCoMJerkThirdCoefficientTimeFunction());
      matrixToPack.set(rowStart, colStart + 3, getCoMJerkFourthCoefficientTimeFunction());
      matrixToPack.set(rowStart, colStart + 4, getCoMJerkFifthCoefficientTimeFunction());
      matrixToPack.set(rowStart, colStart + 5, getCoMJerkSixthCoefficientTimeFunction());
   }

   public static double getCoMCoefficientTimeFunction(int order, int coefficient, double omega, double time)
   {
      switch (order)
      {
      case 0:
         return getCoMPositionCoefficientTimeFunction(coefficient, omega, time);
      case 1:
         return getCoMVelocityCoefficientTimeFunction(coefficient, omega, time);
      case 2:
         return getCoMAccelerationCoefficientTimeFunction(coefficient, omega, time);
      case 3:
         return getCoMJerkCoefficientTimeFunction(coefficient, omega, time);
      default:
         throw new IllegalArgumentException("The order " + order + " must be less than 3.");
      }
   }

   public static double getCoMPositionCoefficientTimeFunction(int coefficient, double omega, double time)
   {
      switch (coefficient)
      {
      case 0:
         return CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, time);
      case 1:
         return CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, time);
      case 2:
         return CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(time);
      case 3:
         return CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(time);
      case 4:
         return CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(time);
      case 5:
         return CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction();
      default:
         throw new IllegalArgumentException("Coefficient number " + coefficient + " must be less than 6.");
      }
   }

   public static double getCoMVelocityCoefficientTimeFunction(int coefficient, double omega, double time)
   {
      switch (coefficient)
      {
      case 0:
         return CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, time);
      case 1:
         return CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, time);
      case 2:
         return CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(time);
      case 3:
         return CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(time);
      case 4:
         return CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction();
      case 5:
         return CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction();
      default:
         throw new IllegalArgumentException("Coefficient number " + coefficient + " must be less than 6.");
      }
   }

   public static double getCoMAccelerationCoefficientTimeFunction(int coefficient, double omega, double time)
   {
      switch (coefficient)
      {
      case 0:
         return CoMTrajectoryPlannerTools.getCoMAccelerationFirstCoefficientTimeFunction(omega, time);
      case 1:
         return CoMTrajectoryPlannerTools.getCoMAccelerationSecondCoefficientTimeFunction(omega, time);
      case 2:
         return CoMTrajectoryPlannerTools.getCoMAccelerationThirdCoefficientTimeFunction(time);
      case 3:
         return CoMTrajectoryPlannerTools.getCoMAccelerationFourthCoefficientTimeFunction();
      case 4:
         return CoMTrajectoryPlannerTools.getCoMAccelerationFifthCoefficientTimeFunction();
      case 5:
         return CoMTrajectoryPlannerTools.getCoMAccelerationSixthCoefficientTimeFunction();
      default:
         throw new IllegalArgumentException("Coefficient number " + coefficient + " must be less than 6.");
      }
   }

   public static double getCoMJerkCoefficientTimeFunction(int coefficient, double omega, double time)
   {
      switch (coefficient)
      {
      case 0:
         return CoMTrajectoryPlannerTools.getCoMJerkFirstCoefficientTimeFunction(omega, time);
      case 1:
         return CoMTrajectoryPlannerTools.getCoMJerkSecondCoefficientTimeFunction(omega, time);
      case 2:
         return CoMTrajectoryPlannerTools.getCoMJerkThirdCoefficientTimeFunction();
      case 3:
         return CoMTrajectoryPlannerTools.getCoMJerkFourthCoefficientTimeFunction();
      case 4:
         return CoMTrajectoryPlannerTools.getCoMJerkFifthCoefficientTimeFunction();
      case 5:
         return CoMTrajectoryPlannerTools.getCoMJerkSixthCoefficientTimeFunction();
      default:
         throw new IllegalArgumentException("Coefficient number " + coefficient + " must be less than 6.");
      }
   }

   /**
    * e<sup>&omega; t</sup>
    */
   public static double getCoMPositionFirstCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * e<sup>-&omega; t</sup>
    */
   public static double getCoMPositionSecondCoefficientTimeFunction(double omega, double time)
   {
      return Math.exp(-omega * time);
   }

   /**
    * t<sup>3</sup>
    */
   public static double getCoMPositionThirdCoefficientTimeFunction(double time)
   {
      return Math.min(sufficientlyLarge, time * time * time);
   }

   /**
    * t<sup>2</sup>
    */
   public static double getCoMPositionFourthCoefficientTimeFunction(double time)
   {
      return Math.min(sufficientlyLarge, time * time);
   }

   /**
    * t
    */
   public static double getCoMPositionFifthCoefficientTimeFunction(double time)
   {
      return Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   public static double getCoMPositionSixthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * &omega; e<sup>&omega; t</sup>
    */
   public static double getCoMVelocityFirstCoefficientTimeFunction(double omega, double time)
   {
      return omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * -&omega; e<sup>-&omega; t</sup>
    */
   public static double getCoMVelocitySecondCoefficientTimeFunction(double omega, double time)
   {
      return -omega * Math.exp(-omega * time);
   }

   /**
    * 3 t<sup>2</sup>
    */
   public static double getCoMVelocityThirdCoefficientTimeFunction(double time)
   {
      return 3.0 * Math.min(sufficientlyLarge, time * time);
   }

   /**
    * 2 t
    */
   public static double getCoMVelocityFourthCoefficientTimeFunction(double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   public static double getCoMVelocityFifthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   public static double getCoMVelocitySixthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * &omega;<sup>2</sup> e<sup>&omega; t</sup>
    */
   public static double getCoMAccelerationFirstCoefficientTimeFunction(double omega, double time)
   {
      return omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * &omega;<sup>2</sup> e<sup>-&omega; t</sup>
    */
   public static double getCoMAccelerationSecondCoefficientTimeFunction(double omega, double time)
   {
      return omega * omega * Math.exp(-omega * time);
   }

   /**
    * 6 t
    */
   public static double getCoMAccelerationThirdCoefficientTimeFunction(double time)
   {
      return 6.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 2
    */
   public static double getCoMAccelerationFourthCoefficientTimeFunction()
   {
      return 2.0;
   }

   /**
    * 0.0
    */
   public static double getCoMAccelerationFifthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getCoMAccelerationSixthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * &omega;<sup>3</sup> e<sup>&omega; t</sup>
    */
   public static double getCoMJerkFirstCoefficientTimeFunction(double omega, double time)
   {
      return omega * omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * -&omega;<sup>3</sup> e<sup>-&omega; t</sup>
    */
   public static double getCoMJerkSecondCoefficientTimeFunction(double omega, double time)
   {
      return -omega * omega * omega * Math.exp(-omega * time);
   }

   /**
    * 6.0
    */
   public static double getCoMJerkThirdCoefficientTimeFunction()
   {
      return 6.0;
   }

   /**
    * 0.0
    */
   public static double getCoMJerkFourthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getCoMJerkFifthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getCoMJerkSixthCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 2 e<sup>&omega; t</sup>
    */
   public static double getDCMPositionFirstCoefficientTimeFunction(double omega, double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * 0.0
    */
   public static double getDCMPositionSecondCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * t<sup>3</sup> + 3.0 / &omega; t<sup>2</sup>
    */
   public static double getDCMPositionThirdCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time * time) + 3.0 / omega * Math.min(sufficientlyLarge, time * time);
   }

   /**
    * t<sup>2</sup> + 2.0 / &omega; t
    */
   public static double getDCMPositionFourthCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time) + 2.0 / omega * Math.min(sufficientlyLarge, time);
   }

   /**
    * t + 1/ &omega;
    */
   public static double getDCMPositionFifthCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time) + 1.0 / omega;
   }

   /**
    * 1.0
    */
   public static double getDCMPositionSixthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   public static double getVRPPositionFirstCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getVRPPositionSecondCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * t<sup>3</sup> - 6.0 t / &omega;<sup>2</sup>
    */
   public static double getVRPPositionThirdCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time * time) - 6.0 * Math.min(sufficientlyLarge, time) / (omega * omega);
   }

   /**
    * t<sup>2</sup> - 2.0 / &omega;<sup>2</sup>
    */
   public static double getVRPPositionFourthCoefficientTimeFunction(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time) - 2.0 / (omega * omega);
   }

   /**
    * t
    */
   public static double getVRPPositionFifthCoefficientTimeFunction(double time)
   {
      return Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   public static double getVRPPositionSixthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   public static double getVRPVelocityFirstCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   public static double getVRPVelocitySecondCoefficientTimeFunction()
   {
      return 0.0;
   }

   /**
    * 3 t<sup>2</sup> - 6 / &omega;<sup>2</sup>
    */
   public static double getVRPVelocityThirdCoefficientTimeFunction(double omega, double time)
   {
      return 3.0 * Math.min(sufficientlyLarge, time * time) - 6.0 / (omega * omega);
   }

   /**
    * 2 t
    */
   public static double getVRPVelocityFourthCoefficientTimeFunction(double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   public static double getVRPVelocityFifthCoefficientTimeFunction()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   public static double getVRPVelocitySixthCoefficientTimeFunction()
   {
      return 0.0;
   }

   public static void constructDesiredCoMPosition(FixedFramePoint3DBasics comPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      comPositionToPack.checkReferenceFrameMatch(worldFrame);
      comPositionToPack.setToZero();
      comPositionToPack.scaleAdd(getCoMPositionFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionSecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionThirdCoefficientTimeFunction(timeInPhase), thirdCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionFourthCoefficientTimeFunction(timeInPhase), fourthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionFifthCoefficientTimeFunction(timeInPhase), fifthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionSixthCoefficientTimeFunction(), sixthCoefficient, comPositionToPack);
   }

   public static void constructDesiredCoMVelocity(FixedFrameVector3DBasics comVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      comVelocityToPack.checkReferenceFrameMatch(worldFrame);
      comVelocityToPack.setToZero();
      comVelocityToPack.scaleAdd(getCoMVelocityFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocitySecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityThirdCoefficientTimeFunction(timeInPhase), thirdCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityFourthCoefficientTimeFunction(timeInPhase), fourthCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityFifthCoefficientTimeFunction(), fifthCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocitySixthCoefficientTimeFunction(), sixthCoefficient, comVelocityToPack);
   }

   public static void constructDesiredCoMAcceleration(FixedFrameVector3DBasics comAccelerationToPack, FramePoint3DReadOnly firstCoefficient,
                                                      FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                      FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                      FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      comAccelerationToPack.checkReferenceFrameMatch(worldFrame);
      comAccelerationToPack.setToZero();
      comAccelerationToPack.scaleAdd(getCoMAccelerationFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationSecondCoefficientTimeFunction(omega, timeInPhase), secondCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationThirdCoefficientTimeFunction(timeInPhase), thirdCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationFourthCoefficientTimeFunction(), fourthCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationFifthCoefficientTimeFunction(), fifthCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationSixthCoefficientTimeFunction(), sixthCoefficient, comAccelerationToPack);
   }

   public static void constructDesiredDCMPosition(FixedFramePoint3DBasics dcmPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      dcmPositionToPack.checkReferenceFrameMatch(worldFrame);
      dcmPositionToPack.setToZero();
      dcmPositionToPack.scaleAdd(getDCMPositionFirstCoefficientTimeFunction(omega, timeInPhase), firstCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionSecondCoefficientTimeFunction(), secondCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionThirdCoefficientTimeFunction(omega, timeInPhase), thirdCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionFourthCoefficientTimeFunction(omega, timeInPhase), fourthCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionFifthCoefficientTimeFunction(omega, timeInPhase), fifthCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(getDCMPositionSixthCoefficientTimeFunction(), sixthCoefficient, dcmPositionToPack);
   }

   public static void constructDesiredVRPPosition(FixedFramePoint3DBasics vrpPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      vrpPositionToPack.checkReferenceFrameMatch(worldFrame);
      vrpPositionToPack.setToZero();
      vrpPositionToPack.scaleAdd(getVRPPositionFirstCoefficientTimeFunction(), firstCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionSecondCoefficientTimeFunction(), secondCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionThirdCoefficientTimeFunction(omega, timeInPhase), thirdCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionFourthCoefficientTimeFunction(omega, timeInPhase), fourthCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionFifthCoefficientTimeFunction(timeInPhase), fifthCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(getVRPPositionSixthCoefficientTimeFunction(), sixthCoefficient, vrpPositionToPack);
   }

   public static void constructDesiredVRPVelocity(FixedFrameVector3DBasics vrpVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      vrpVelocityToPack.checkReferenceFrameMatch(worldFrame);
      vrpVelocityToPack.setToZero();
      vrpVelocityToPack.scaleAdd(getVRPVelocityFirstCoefficientTimeFunction(), firstCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocitySecondCoefficientTimeFunction(), secondCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocityThirdCoefficientTimeFunction(omega, timeInPhase), thirdCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocityFourthCoefficientTimeFunction(timeInPhase), fourthCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocityFifthCoefficientTimeFunction(), fifthCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(getVRPVelocitySixthCoefficientTimeFunction(), sixthCoefficient, vrpVelocityToPack);
   }
}
