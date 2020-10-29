package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix;
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
   private static final boolean SET_ZERO_VALUES = false;
   public static final double minDuration = 1.0e-5;
   public static final double sufficientlyLarge = 1.0e10;
   public static final double sufficientlyLongTime = 1.0e2;

   public static void computeVRPWaypoints(double nominalCoMHeight,
                                          double gravityZ,
                                          double omega,
                                          FrameVector3DReadOnly currentCoMVelocity,
                                          List<? extends ContactStateProvider> contactSequence,
                                          RecyclingArrayList<FramePoint3D> startVRPPositionsToPack,
                                          RecyclingArrayList<FramePoint3D> endVRPPositionsToPack)
   {
      computeVRPWaypoints(nominalCoMHeight, gravityZ, omega, currentCoMVelocity, contactSequence, startVRPPositionsToPack, endVRPPositionsToPack, true);
   }

   public static void computeVRPWaypoints(double nominalCoMHeight,
                                          double gravityZ,
                                          double omega,
                                          FrameVector3DReadOnly currentCoMVelocity,
                                          List<? extends ContactStateProvider> contactSequence,
                                          RecyclingArrayList<FramePoint3D> startVRPPositionsToPack,
                                          RecyclingArrayList<FramePoint3D> endVRPPositionsToPack,
                                          boolean adjustWaypointHeightForHeightChange)
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
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p>
    * This constraint defines
    * </p>
    * <p>
    * x<sub>0</sub>(0) = x<sub>d</sub>,
    * </p>
    * <p>
    * substituting in the coefficients into the constraint matrix.
    * </p>
    *
    * @param centerOfMassLocationForConstraint x<sub>d</sub> in the above equations
    */
   public static void addCoMPositionConstraint(FramePoint3DReadOnly centerOfMassLocationForConstraint,
                                               double omega,
                                               double time,
                                               int sequenceId,
                                               int rowStart,
                                               DMatrix constraintMatrixToPack,
                                               DMatrix xObjectiveMatrixToPack,
                                               DMatrix yObjectiveMatrixToPack,
                                               DMatrix zObjectiveMatrixToPack)
   {
      centerOfMassLocationForConstraint.checkReferenceFrameMatch(worldFrame);

      time = Math.min(time, sufficientlyLongTime);

      int colStart = 6 * sequenceId;
      constraintMatrixToPack.set(rowStart, colStart, getCoMPositionFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, colStart + 1, getCoMPositionSecondCoefficientTimeFunction(omega, time));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      { // assuming the matrix is initialized as zero, these don't have to be set
         constraintMatrixToPack.set(rowStart, colStart + 2, getCoMPositionThirdCoefficientTimeFunction(time));
         constraintMatrixToPack.set(rowStart, colStart + 3, getCoMPositionFourthCoefficientTimeFunction(time));
         constraintMatrixToPack.set(rowStart, colStart + 4, getCoMPositionFifthCoefficientTimeFunction(time));
      }
      constraintMatrixToPack.set(rowStart, colStart + 5, getCoMPositionSixthCoefficientTimeFunction());

      xObjectiveMatrixToPack.set(rowStart, 0, centerOfMassLocationForConstraint.getX());
      yObjectiveMatrixToPack.set(rowStart, 0, centerOfMassLocationForConstraint.getY());
      zObjectiveMatrixToPack.set(rowStart, 0, centerOfMassLocationForConstraint.getZ());
   }

   public static void addCoMPositionObjective(double weight,
                                              FramePoint3DReadOnly centerOfMassLocationForConstraint,
                                              double omega,
                                              double time,
                                              int sequenceId,
                                              int rowStart,
                                              DMatrix objectiveJacobianToPack,
                                              DMatrix xObjectiveMatrixToPack,
                                              DMatrix yObjectiveMatrixToPack,
                                              DMatrix zObjectiveMatrixToPack)
   {
      centerOfMassLocationForConstraint.checkReferenceFrameMatch(worldFrame);

      time = Math.min(time, sufficientlyLongTime);

      int colStart = 6 * sequenceId;
      addEquals(objectiveJacobianToPack, rowStart, colStart, weight * getCoMPositionFirstCoefficientTimeFunction(omega, time));
      addEquals(objectiveJacobianToPack, rowStart, colStart + 1, weight * getCoMPositionSecondCoefficientTimeFunction(omega, time));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      { // assuming the matrix is initialized as zero, these don't have to be set
         addEquals(objectiveJacobianToPack, rowStart, colStart + 2, weight * getCoMPositionThirdCoefficientTimeFunction(time));
         addEquals(objectiveJacobianToPack, rowStart, colStart + 3, weight * getCoMPositionFourthCoefficientTimeFunction(time));
         addEquals(objectiveJacobianToPack, rowStart, colStart + 4, weight * getCoMPositionFifthCoefficientTimeFunction(time));
      }
      addEquals(objectiveJacobianToPack, rowStart, colStart + 5, weight * getCoMPositionSixthCoefficientTimeFunction());

      xObjectiveMatrixToPack.set(rowStart, 0, weight * centerOfMassLocationForConstraint.getX());
      yObjectiveMatrixToPack.set(rowStart, 0, weight * centerOfMassLocationForConstraint.getY());
      zObjectiveMatrixToPack.set(rowStart, 0, weight * centerOfMassLocationForConstraint.getZ());
   }

   public static void addCoMPositionObjective(double weight,
                                              FramePoint3DReadOnly centerOfMassLocationForConstraint,
                                              double omega,
                                              double time,
                                              int sequenceId,
                                              DMatrix hessianToPack,
                                              DMatrix xGradientToPack,
                                              DMatrix yGradientToPack,
                                              DMatrix zGradientToPack)
   {
      centerOfMassLocationForConstraint.checkReferenceFrameMatch(worldFrame);

      time = Math.min(time, sufficientlyLongTime);
      int startIdx = 6 * sequenceId;

      double c0 = getCoMPositionFirstCoefficientTimeFunction(omega, time);
      double c1 = getCoMPositionSecondCoefficientTimeFunction(omega, time);
      double c5 = getCoMPositionSixthCoefficientTimeFunction();

      addEquals(hessianToPack, startIdx, startIdx, weight * c0 * c0);
      addEquals(hessianToPack, startIdx, startIdx + 1, weight * c0 * c1);
      addEquals(hessianToPack, startIdx, startIdx + 5, weight * c0 * c5);

      addEquals(hessianToPack, startIdx + 1, startIdx, weight * c1 * c0);
      addEquals(hessianToPack, startIdx + 1, startIdx + 1, weight * c1 * c1);
      addEquals(hessianToPack, startIdx + 1, startIdx + 5, weight * c1 * c5);

      addEquals(hessianToPack, startIdx + 5, startIdx, weight * c5 * c0);
      addEquals(hessianToPack, startIdx + 5, startIdx + 1, weight * c5 * c1);
      addEquals(hessianToPack, startIdx + 5, startIdx + 5, weight * c5 * c5);

      addEquals(xGradientToPack, startIdx, 0, 2.0 * weight * c0 * centerOfMassLocationForConstraint.getX());
      addEquals(xGradientToPack, startIdx + 1, 0, 2.0 * weight * c1 * centerOfMassLocationForConstraint.getX());
      addEquals(xGradientToPack, startIdx + 5, 0, 2.0 * weight * c5 * centerOfMassLocationForConstraint.getX());

      addEquals(yGradientToPack, startIdx, 0, 2.0 * weight * c0 * centerOfMassLocationForConstraint.getY());
      addEquals(yGradientToPack, startIdx + 1, 0, 2.0 * weight * c1 * centerOfMassLocationForConstraint.getY());
      addEquals(yGradientToPack, startIdx + 5, 0, 2.0 * weight * c5 * centerOfMassLocationForConstraint.getY());

      addEquals(zGradientToPack, startIdx, 0, 2.0 * weight * c0 * centerOfMassLocationForConstraint.getZ());
      addEquals(zGradientToPack, startIdx + 1, 0, 2.0 * weight * c1 * centerOfMassLocationForConstraint.getZ());
      addEquals(zGradientToPack, startIdx + 5, 0, 2.0 * weight * c5 * centerOfMassLocationForConstraint.getZ());

      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      { // assuming the matrix is initialized as zero, these don't have to be set

         double c2 = getCoMPositionThirdCoefficientTimeFunction(time);
         double c3 = getCoMPositionFourthCoefficientTimeFunction(time);
         double c4 = getCoMPositionFifthCoefficientTimeFunction(time);

         addEquals(hessianToPack, startIdx, startIdx + 2, weight * c0 * c2);
         addEquals(hessianToPack, startIdx, startIdx + 3, weight * c0 * c3);
         addEquals(hessianToPack, startIdx, startIdx + 4, weight * c0 * c4);

         addEquals(hessianToPack, startIdx + 1, startIdx + 2, weight * c1 * c2);
         addEquals(hessianToPack, startIdx + 1, startIdx + 3, weight * c1 * c3);
         addEquals(hessianToPack, startIdx + 1, startIdx + 4, weight * c1 * c4);

         addEquals(hessianToPack, startIdx + 2, startIdx, weight * c2 * c0);
         addEquals(hessianToPack, startIdx + 2, startIdx + 1, weight * c2 * c1);
         addEquals(hessianToPack, startIdx + 2, startIdx + 2, weight * c2 * c2);
         addEquals(hessianToPack, startIdx + 2, startIdx + 3, weight * c2 * c3);
         addEquals(hessianToPack, startIdx + 2, startIdx + 4, weight * c2 * c4);
         addEquals(hessianToPack, startIdx + 2, startIdx + 5, weight * c2 * c5);

         addEquals(hessianToPack, startIdx + 3, startIdx, weight * c3 * c0);
         addEquals(hessianToPack, startIdx + 3, startIdx + 1, weight * c3 * c1);
         addEquals(hessianToPack, startIdx + 3, startIdx + 2, weight * c3 * c2);
         addEquals(hessianToPack, startIdx + 3, startIdx + 3, weight * c3 * c3);
         addEquals(hessianToPack, startIdx + 3, startIdx + 4, weight * c3 * c4);
         addEquals(hessianToPack, startIdx + 3, startIdx + 5, weight * c3 * c5);

         addEquals(hessianToPack, startIdx + 4, startIdx, weight * c4 * c0);
         addEquals(hessianToPack, startIdx + 4, startIdx + 1, weight * c4 * c1);
         addEquals(hessianToPack, startIdx + 4, startIdx + 2, weight * c4 * c2);
         addEquals(hessianToPack, startIdx + 4, startIdx + 3, weight * c4 * c3);
         addEquals(hessianToPack, startIdx + 4, startIdx + 4, weight * c4 * c4);
         addEquals(hessianToPack, startIdx + 4, startIdx + 5, weight * c4 * c5);

         addEquals(hessianToPack, startIdx + 5, startIdx + 2, weight * c5 * c2);
         addEquals(hessianToPack, startIdx + 5, startIdx + 3, weight * c5 * c3);
         addEquals(hessianToPack, startIdx + 5, startIdx + 4, weight * c5 * c4);

         addEquals(xGradientToPack, startIdx + 2, 0, 2.0 * weight * c2 * centerOfMassLocationForConstraint.getX());
         addEquals(xGradientToPack, startIdx + 3, 0, 2.0 * weight * c3 * centerOfMassLocationForConstraint.getX());
         addEquals(xGradientToPack, startIdx + 4, 0, 2.0 * weight * c4 * centerOfMassLocationForConstraint.getX());

         addEquals(yGradientToPack, startIdx + 2, 0, 2.0 * weight * c2 * centerOfMassLocationForConstraint.getY());
         addEquals(yGradientToPack, startIdx + 3, 0, 2.0 * weight * c3 * centerOfMassLocationForConstraint.getY());
         addEquals(yGradientToPack, startIdx + 4, 0, 2.0 * weight * c4 * centerOfMassLocationForConstraint.getY());

         addEquals(zGradientToPack, startIdx + 2, 0, 2.0 * weight * c2 * centerOfMassLocationForConstraint.getZ());
         addEquals(zGradientToPack, startIdx + 3, 0, 2.0 * weight * c3 * centerOfMassLocationForConstraint.getZ());
         addEquals(zGradientToPack, startIdx + 4, 0, 2.0 * weight * c4 * centerOfMassLocationForConstraint.getZ());
      }
   }

   /**
    * <p> Sets the continuity constraint on the initial CoM velocity. </p>
    * <p> This constraint should be used for the initial velocity of the center of mass to properly initialize the trajectory. </p>
    * <p> Recall that the equation for the center of mass is defined by </p>
    * <p>
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p>
    * This constraint defines
    * </p>
    * <p>
    * x<sub>0</sub>(0) = x<sub>d</sub>,
    * </p>
    * <p>
    * substituting in the coefficients into the constraint matrix.
    * </p>
    *
    * @param centerOfMassVelocityForConstraint x<sub>d</sub> in the above equations
    */
   public static void addCoMVelocityConstraint(FrameVector3DReadOnly centerOfMassVelocityForConstraint,
                                               double omega,
                                               double time,
                                               int sequenceId,
                                               int rowStart,
                                               DMatrix constraintMatrixToPack,
                                               DMatrix xObjectiveMatrixToPack,
                                               DMatrix yObjectiveMatrixToPack,
                                               DMatrix zObjectiveMatrixToPack)
   {
      centerOfMassVelocityForConstraint.checkReferenceFrameMatch(worldFrame);

      time = Math.min(time, sufficientlyLongTime);

      int colStart = 6 * sequenceId;
      constraintMatrixToPack.set(rowStart, colStart, getCoMVelocityFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, colStart + 1, getCoMVelocitySecondCoefficientTimeFunction(omega, time));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         constraintMatrixToPack.set(rowStart, colStart + 2, getCoMVelocityThirdCoefficientTimeFunction(time));
         constraintMatrixToPack.set(rowStart, colStart + 3, getCoMVelocityFourthCoefficientTimeFunction(time));
      }
      constraintMatrixToPack.set(rowStart, colStart + 4, getCoMVelocityFifthCoefficientTimeFunction());
      if (SET_ZERO_VALUES)
         constraintMatrixToPack.set(rowStart, colStart + 5, getCoMVelocitySixthCoefficientTimeFunction());

      xObjectiveMatrixToPack.set(rowStart, 0, centerOfMassVelocityForConstraint.getX());
      yObjectiveMatrixToPack.set(rowStart, 0, centerOfMassVelocityForConstraint.getY());
      zObjectiveMatrixToPack.set(rowStart, 0, centerOfMassVelocityForConstraint.getZ());
   }

   public static void addCoMVelocityObjective(double weight,
                                              FrameVector3DReadOnly centerOfMassVelocityForConstraint,
                                              double omega,
                                              double time,
                                              int sequenceId,
                                              int rowStart,
                                              DMatrix objectiveJacobianToPack,
                                              DMatrix xObjectiveMatrixToPack,
                                              DMatrix yObjectiveMatrixToPack,
                                              DMatrix zObjectiveMatrixToPack)
   {
      centerOfMassVelocityForConstraint.checkReferenceFrameMatch(worldFrame);

      time = Math.min(time, sufficientlyLongTime);

      int colStart = 6 * sequenceId;
      addEquals(objectiveJacobianToPack, rowStart, colStart, weight * getCoMVelocityFirstCoefficientTimeFunction(omega, time));
      addEquals(objectiveJacobianToPack, rowStart, colStart + 1, weight * getCoMVelocitySecondCoefficientTimeFunction(omega, time));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         addEquals(objectiveJacobianToPack, rowStart, colStart + 2, weight * getCoMVelocityThirdCoefficientTimeFunction(time));
         addEquals(objectiveJacobianToPack, rowStart, colStart + 3, weight * getCoMVelocityFourthCoefficientTimeFunction(time));
      }
      addEquals(objectiveJacobianToPack, rowStart, colStart + 4, weight * getCoMVelocityFifthCoefficientTimeFunction());
      if (SET_ZERO_VALUES)
         addEquals(objectiveJacobianToPack, rowStart, colStart + 5, weight * getCoMVelocitySixthCoefficientTimeFunction());

      addEquals(xObjectiveMatrixToPack, rowStart, 0, weight * centerOfMassVelocityForConstraint.getX());
      addEquals(yObjectiveMatrixToPack, rowStart, 0, weight * centerOfMassVelocityForConstraint.getY());
      addEquals(zObjectiveMatrixToPack, rowStart, 0, weight * centerOfMassVelocityForConstraint.getZ());
   }

   public static void addCoMVelocityObjective(double weight,
                                              FrameVector3DReadOnly centerOfMassVelocityForConstraint,
                                              double omega,
                                              double time,
                                              int sequenceId,
                                              DMatrix hessianToPack,
                                              DMatrix xGradientToPack,
                                              DMatrix yGradientToPack,
                                              DMatrix zGradientToPack)
   {
      centerOfMassVelocityForConstraint.checkReferenceFrameMatch(worldFrame);

      time = Math.min(time, sufficientlyLongTime);
      int startIdx = 6 * sequenceId;

      double c0 = getCoMVelocityFirstCoefficientTimeFunction(omega, time);
      double c1 = getCoMVelocitySecondCoefficientTimeFunction(omega, time);
      double c4 = getCoMVelocityFifthCoefficientTimeFunction();

      addEquals(hessianToPack, startIdx, startIdx, weight * c0 * c0);
      addEquals(hessianToPack, startIdx, startIdx + 1, weight * c0 * c1);
      addEquals(hessianToPack, startIdx, startIdx + 4, weight * c0 * c4);

      addEquals(hessianToPack, startIdx + 1, startIdx, weight * c1 * c0);
      addEquals(hessianToPack, startIdx + 1, startIdx + 1, weight * c1 * c1);
      addEquals(hessianToPack, startIdx + 1, startIdx + 4, weight * c1 * c4);

      addEquals(hessianToPack, startIdx + 4, startIdx, weight * c4 * c0);
      addEquals(hessianToPack, startIdx + 4, startIdx + 1, weight * c4 * c1);
      addEquals(hessianToPack, startIdx + 4, startIdx + 4, weight * c4 * c4);

      addEquals(xGradientToPack, startIdx, 0, 2.0 * weight * c0 * centerOfMassVelocityForConstraint.getX());
      addEquals(xGradientToPack, startIdx + 1, 0, 2.0 * weight * c1 * centerOfMassVelocityForConstraint.getX());
      addEquals(xGradientToPack, startIdx + 4, 0, 2.0 * weight * c4 * centerOfMassVelocityForConstraint.getX());

      addEquals(yGradientToPack, startIdx, 0, 2.0 * weight * c0 * centerOfMassVelocityForConstraint.getY());
      addEquals(yGradientToPack, startIdx + 1, 0, 2.0 * weight * c1 * centerOfMassVelocityForConstraint.getY());
      addEquals(yGradientToPack, startIdx + 4, 0, 2.0 * weight * c4 * centerOfMassVelocityForConstraint.getY());

      addEquals(zGradientToPack, startIdx, 0, 2.0 * weight * c0 * centerOfMassVelocityForConstraint.getZ());
      addEquals(zGradientToPack, startIdx + 1, 0, 2.0 * weight * c1 * centerOfMassVelocityForConstraint.getZ());
      addEquals(zGradientToPack, startIdx + 4, 0, 2.0 * weight * c4 * centerOfMassVelocityForConstraint.getZ());

      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      { // assuming the matrix is initialized as zero, these don't have to be set
         double c2 = getCoMVelocityThirdCoefficientTimeFunction(time);
         double c3 = getCoMVelocityFourthCoefficientTimeFunction(time);

         addEquals(hessianToPack, startIdx, startIdx + 2, weight * c0 * c2);
         addEquals(hessianToPack, startIdx, startIdx + 3, weight * c0 * c3);

         addEquals(hessianToPack, startIdx + 1, startIdx + 2, weight * c1 * c2);
         addEquals(hessianToPack, startIdx + 1, startIdx + 3, weight * c1 * c3);

         addEquals(hessianToPack, startIdx + 2, startIdx, weight * c2 * c0);
         addEquals(hessianToPack, startIdx + 2, startIdx + 1, weight * c2 * c1);
         addEquals(hessianToPack, startIdx + 2, startIdx + 2, weight * c2 * c2);
         addEquals(hessianToPack, startIdx + 2, startIdx + 3, weight * c2 * c3);
         addEquals(hessianToPack, startIdx + 2, startIdx + 4, weight * c2 * c4);

         addEquals(hessianToPack, startIdx + 3, startIdx, weight * c3 * c0);
         addEquals(hessianToPack, startIdx + 3, startIdx + 1, weight * c3 * c1);
         addEquals(hessianToPack, startIdx + 3, startIdx + 2, weight * c3 * c2);
         addEquals(hessianToPack, startIdx + 3, startIdx + 3, weight * c3 * c3);
         addEquals(hessianToPack, startIdx + 3, startIdx + 4, weight * c3 * c4);

         addEquals(hessianToPack, startIdx + 4, startIdx + 2, weight * c4 * c2);
         addEquals(hessianToPack, startIdx + 4, startIdx + 3, weight * c4 * c3);

         addEquals(xGradientToPack, startIdx + 2, 0, 2.0 * weight * c2 * centerOfMassVelocityForConstraint.getX());
         addEquals(xGradientToPack, startIdx + 3, 0, 2.0 * weight * c3 * centerOfMassVelocityForConstraint.getX());

         addEquals(yGradientToPack, startIdx + 2, 0, 2.0 * weight * c2 * centerOfMassVelocityForConstraint.getY());
         addEquals(yGradientToPack, startIdx + 3, 0, 2.0 * weight * c3 * centerOfMassVelocityForConstraint.getY());

         addEquals(zGradientToPack, startIdx + 2, 0, 2.0 * weight * c2 * centerOfMassVelocityForConstraint.getZ());
         addEquals(zGradientToPack, startIdx + 3, 0, 2.0 * weight * c3 * centerOfMassVelocityForConstraint.getZ());
      }
   }

   /**
    * <p> Sets a constraint on the desired DCM position. This constraint is useful for constraining the terminal location of the DCM trajectory. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> and the center of mass velocity is defined by </p>
    * <p>
    * d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    * &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    * 2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>
    * </p>
    * <p>
    * This constraint is then combining these two, saying
    * </p>
    * <p> x<sub>i</sub>(t<sub>i</sub>) + 1 / &omega; d/dt x<sub>i</sub>(t<sub>i</sub>) = &xi;<sub>d</sub>,</p>
    * <p> substituting in the appropriate coefficients. </p>
    *
    * @param sequenceId i in the above equations
    * @param time t<sub>i</sub> in the above equations
    * @param desiredDCMPosition desired DCM location. &xi;<sub>d</sub> in the above equations.
    */
   public static void addDCMPositionConstraint(int sequenceId,
                                               int rowStart,
                                               double time,
                                               double omega,
                                               FramePoint3DReadOnly desiredDCMPosition,
                                               DMatrix constraintMatrixToPack,
                                               DMatrix xObjectiveMatrixToPack,
                                               DMatrix yObjectiveMatrixToPack,
                                               DMatrix zObjectiveMatrixToPack)
   {
      desiredDCMPosition.checkReferenceFrameMatch(worldFrame);

      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      // add constraints on terminal DCM position
      constraintMatrixToPack.set(rowStart, startIndex, getDCMPositionFirstCoefficientTimeFunction(omega, time));
      if (SET_ZERO_VALUES)
      {
         constraintMatrixToPack.set(rowStart, startIndex + 1, getDCMPositionSecondCoefficientTimeFunction());
      }
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         constraintMatrixToPack.set(rowStart, startIndex + 2, getDCMPositionThirdCoefficientTimeFunction(omega, time));
         constraintMatrixToPack.set(rowStart, startIndex + 3, getDCMPositionFourthCoefficientTimeFunction(omega, time));
      }
      constraintMatrixToPack.set(rowStart, startIndex + 4, getDCMPositionFifthCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(rowStart, startIndex + 5, getDCMPositionSixthCoefficientTimeFunction());

      xObjectiveMatrixToPack.set(rowStart, 0, desiredDCMPosition.getX());
      yObjectiveMatrixToPack.set(rowStart, 0, desiredDCMPosition.getY());
      zObjectiveMatrixToPack.set(rowStart, 0, desiredDCMPosition.getZ());
   }

   public static void addDCMPositionObjective(double weight,
                                              int sequenceId,
                                              int rowStart,
                                              double time,
                                              double omega,
                                              FramePoint3DReadOnly desiredDCMPosition,
                                              DMatrix objectiveJacobianToPack,
                                              DMatrix xObjectiveMatrixToPack,
                                              DMatrix yObjectiveMatrixToPack,
                                              DMatrix zObjectiveMatrixToPack)
   {
      desiredDCMPosition.checkReferenceFrameMatch(worldFrame);

      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      // add constraints on terminal DCM position
      addEquals(objectiveJacobianToPack, rowStart, startIndex, weight * getDCMPositionFirstCoefficientTimeFunction(omega, time));
      if (SET_ZERO_VALUES)
      {
         addEquals(objectiveJacobianToPack, rowStart, startIndex + 1, weight * getDCMPositionSecondCoefficientTimeFunction());
      }
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         addEquals(objectiveJacobianToPack, rowStart, startIndex + 2, weight * getDCMPositionThirdCoefficientTimeFunction(omega, time));
         addEquals(objectiveJacobianToPack, rowStart, startIndex + 3, weight * getDCMPositionFourthCoefficientTimeFunction(omega, time));
      }
      addEquals(objectiveJacobianToPack, rowStart, startIndex + 4, weight * getDCMPositionFifthCoefficientTimeFunction(omega, time));
      addEquals(objectiveJacobianToPack, rowStart, startIndex + 5, weight * getDCMPositionSixthCoefficientTimeFunction());

      addEquals(xObjectiveMatrixToPack, rowStart, 0, weight * desiredDCMPosition.getX());
      addEquals(yObjectiveMatrixToPack, rowStart, 0, weight * desiredDCMPosition.getY());
      addEquals(zObjectiveMatrixToPack, rowStart, 0, weight * desiredDCMPosition.getZ());
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
    *
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointPositionIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPPosition reference VRP position, v<sub>r</sub> in the above equations.
    */
   public static void addVRPPositionConstraint(int sequenceId,
                                               int constraintNumber,
                                               int vrpWaypointPositionIndex,
                                               double time,
                                               double omega,
                                               FramePoint3DReadOnly desiredVRPPosition,
                                               DMatrix constraintMatrixToPack,
                                               DMatrix xObjectiveMatrixToPack,
                                               DMatrix yObjectiveMatrixToPack,
                                               DMatrix zObjectiveMatrixToPack,
                                               DMatrix vrpWaypointJacobianToPack)
   {
      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      desiredVRPPosition.checkReferenceFrameMatch(worldFrame);

      if (SET_ZERO_VALUES)
      {
         constraintMatrixToPack.set(constraintNumber, startIndex + 0, CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintNumber, startIndex + 1, CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction());
      }
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         constraintMatrixToPack.set(constraintNumber, startIndex + 2, CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, time));
         constraintMatrixToPack.set(constraintNumber, startIndex + 4, CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(time));
      }
      constraintMatrixToPack.set(constraintNumber, startIndex + 3, CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintNumber, startIndex + 5, CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction());

      vrpWaypointJacobianToPack.set(constraintNumber, vrpWaypointPositionIndex, 1.0);

      xObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getX());
      yObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getY());
      zObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getZ());
   }

   public static void addVRPPositionObjective(double weight,
                                              int sequenceId,
                                              int constraintNumber,
                                              int vrpWaypointPositionIndex,
                                              double time,
                                              double omega,
                                              FramePoint3DReadOnly desiredVRPPosition,
                                              DMatrix objectiveJacobianToPack,
                                              DMatrix xObjectiveMatrixToPack,
                                              DMatrix yObjectiveMatrixToPack,
                                              DMatrix zObjectiveMatrixToPack,
                                              DMatrix vrpWaypointJacobianToPack)
   {
      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      desiredVRPPosition.checkReferenceFrameMatch(worldFrame);

      if (SET_ZERO_VALUES)
      {
         addEquals(objectiveJacobianToPack, constraintNumber, startIndex + 0, weight * CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction());
         addEquals(objectiveJacobianToPack, constraintNumber, startIndex + 1, weight * CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction());
      }
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         addEquals(objectiveJacobianToPack,
                   constraintNumber,
                   startIndex + 2,
                   weight * CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, time));
         addEquals(objectiveJacobianToPack,
                   constraintNumber,
                   startIndex + 4,
                   weight * CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(time));
      }
      addEquals(objectiveJacobianToPack,
                constraintNumber,
                startIndex + 3,
                weight * CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, time));
      addEquals(objectiveJacobianToPack, constraintNumber, startIndex + 5, weight * CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction());

      vrpWaypointJacobianToPack.set(constraintNumber, vrpWaypointPositionIndex, 1.0);

      addEquals(xObjectiveMatrixToPack, vrpWaypointPositionIndex, 0, weight * desiredVRPPosition.getX());
      addEquals(yObjectiveMatrixToPack, vrpWaypointPositionIndex, 0, weight * desiredVRPPosition.getY());
      addEquals(zObjectiveMatrixToPack, vrpWaypointPositionIndex, 0, weight * desiredVRPPosition.getZ());
   }

   public static void addVRPPositionObjective(double weight,
                                              int sequenceId,
                                              double time,
                                              double omega,
                                              FramePoint3DReadOnly desiredVRPPosition,
                                              DMatrix hessianToPack,
                                              DMatrix xGradientToPack,
                                              DMatrix yGradientToPack,
                                              DMatrix zGradientToPack)
   {
      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      desiredVRPPosition.checkReferenceFrameMatch(worldFrame);

      double c2 = CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, time);
      double c3 = CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, time);
      double c5 = CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction();

      addEquals(hessianToPack, startIndex + 2, startIndex + 2, weight * c2 * c2);
      addEquals(hessianToPack, startIndex + 2, startIndex + 3, weight * c2 * c3);
      addEquals(hessianToPack, startIndex + 2, startIndex + 5, weight * c2 * c5);

      addEquals(hessianToPack, startIndex + 3, startIndex + 2, weight * c3 * c2);
      addEquals(hessianToPack, startIndex + 3, startIndex + 3, weight * c3 * c3);
      addEquals(hessianToPack, startIndex + 3, startIndex + 5, weight * c3 * c5);

      addEquals(hessianToPack, startIndex + 5, startIndex + 2, weight * c5 * c2);
      addEquals(hessianToPack, startIndex + 5, startIndex + 3, weight * c5 * c3);
      addEquals(hessianToPack, startIndex + 5, startIndex + 5, weight * c5 * c5);

      addEquals(xGradientToPack, startIndex + 2, startIndex + 2, 2.0 * weight * c2 * desiredVRPPosition.getX());
      addEquals(xGradientToPack, startIndex + 2, startIndex + 3, 2.0 * weight * c3 * desiredVRPPosition.getX());
      addEquals(xGradientToPack, startIndex + 2, startIndex + 5, 2.0 * weight * c5 * desiredVRPPosition.getX());

      addEquals(yGradientToPack, startIndex + 2, startIndex + 2, 2.0 * weight * c2 * desiredVRPPosition.getY());
      addEquals(yGradientToPack, startIndex + 2, startIndex + 3, 2.0 * weight * c3 * desiredVRPPosition.getY());
      addEquals(yGradientToPack, startIndex + 2, startIndex + 5, 2.0 * weight * c5 * desiredVRPPosition.getY());

      addEquals(zGradientToPack, startIndex + 2, startIndex + 2, 2.0 * weight * c2 * desiredVRPPosition.getZ());
      addEquals(zGradientToPack, startIndex + 2, startIndex + 3, 2.0 * weight * c3 * desiredVRPPosition.getZ());
      addEquals(zGradientToPack, startIndex + 2, startIndex + 5, 2.0 * weight * c5 * desiredVRPPosition.getZ());

      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         double c4 = CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(time);

         addEquals(hessianToPack, startIndex + 2, startIndex + 4, weight * c2 * c4);
         addEquals(hessianToPack, startIndex + 3, startIndex + 4, weight * c3 * c4);

         addEquals(hessianToPack, startIndex + 4, startIndex + 2, weight * c4 * c2);
         addEquals(hessianToPack, startIndex + 4, startIndex + 3, weight * c4 * c3);
         addEquals(hessianToPack, startIndex + 4, startIndex + 4, weight * c4 * c4);
         addEquals(hessianToPack, startIndex + 4, startIndex + 5, weight * c4 * c5);

         addEquals(hessianToPack, startIndex + 5, startIndex + 4, weight * c5 * c4);

         addEquals(xGradientToPack, startIndex + 2, startIndex + 4, 2.0 * weight * c4 * desiredVRPPosition.getX());
         addEquals(yGradientToPack, startIndex + 2, startIndex + 4, 2.0 * weight * c4 * desiredVRPPosition.getY());
         addEquals(zGradientToPack, startIndex + 2, startIndex + 4, 2.0 * weight * c4 * desiredVRPPosition.getZ());
      }
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
    *
    * @param sequenceId segment of interest, i in the above equations
    * @param vrpWaypointVelocityIndex current vrp waypoint index, j in the above equations
    * @param time time in the segment, t<sub>i</sub> in the above equations
    * @param desiredVRPVelocity reference VRP veloctiy, d/dt v<sub>r</sub> in the above equations.
    */
   public static void addVRPVelocityConstraint(int sequenceId,
                                               int constraintRow,
                                               int vrpWaypointVelocityIndex,
                                               double omega,
                                               double time,
                                               FrameVector3DReadOnly desiredVRPVelocity,
                                               DMatrix constraintMatrixToPack,
                                               DMatrix xObjectiveMatrixToPack,
                                               DMatrix yObjectiveMatrixToPack,
                                               DMatrix zObjectiveMatrixToPack,
                                               DMatrix vrpWaypointJacobianToPack)
   {
      int startIndex = 6 * sequenceId;

      desiredVRPVelocity.checkReferenceFrameMatch(worldFrame);

      if (SET_ZERO_VALUES)
      {
         constraintMatrixToPack.set(constraintRow, startIndex + 0, CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintRow, startIndex + 1, CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintRow, startIndex + 5, CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction());
      }
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         constraintMatrixToPack.set(constraintRow, startIndex + 3, CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(time));
      }
      constraintMatrixToPack.set(constraintRow, startIndex + 2, CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintRow, startIndex + 4, CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction());

      vrpWaypointJacobianToPack.set(constraintRow, vrpWaypointVelocityIndex, 1.0);

      xObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getX());
      yObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getY());
      zObjectiveMatrixToPack.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getZ());
   }

   public static void addVRPVelocityObjective(double weight,
                                              int sequenceId,
                                              int constraintRow,
                                              int vrpWaypointVelocityIndex,
                                              double omega,
                                              double time,
                                              FrameVector3DReadOnly desiredVRPVelocity,
                                              DMatrix objectiveJacobianToPack,
                                              DMatrix xObjectiveMatrixToPack,
                                              DMatrix yObjectiveMatrixToPack,
                                              DMatrix zObjectiveMatrixToPack,
                                              DMatrix vrpWaypointJacobianToPack)
   {
      int startIndex = 6 * sequenceId;

      desiredVRPVelocity.checkReferenceFrameMatch(worldFrame);

      if (SET_ZERO_VALUES)
      {
         addEquals(objectiveJacobianToPack, constraintRow, startIndex + 0, weight * CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction());
         addEquals(objectiveJacobianToPack, constraintRow, startIndex + 1, weight * CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction());
         addEquals(objectiveJacobianToPack, constraintRow, startIndex + 5, weight * CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction());
      }
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         addEquals(objectiveJacobianToPack,
                   constraintRow,
                   startIndex + 3,
                   weight * CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(time));
      }
      addEquals(objectiveJacobianToPack,
                constraintRow,
                startIndex + 2,
                weight * CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, time));
      addEquals(objectiveJacobianToPack, constraintRow, startIndex + 4, weight * CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction());

      vrpWaypointJacobianToPack.set(constraintRow, vrpWaypointVelocityIndex, 1.0);

      addEquals(xObjectiveMatrixToPack, vrpWaypointVelocityIndex, 0, weight * desiredVRPVelocity.getX());
      addEquals(yObjectiveMatrixToPack, vrpWaypointVelocityIndex, 0, weight * desiredVRPVelocity.getY());
      addEquals(zObjectiveMatrixToPack, vrpWaypointVelocityIndex, 0, weight * desiredVRPVelocity.getZ());
   }

   /**
    * <p> Set a continuity constraint on the CoM position at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> x<sub>i-1</sub>(T<sub>i-1</sub>) = x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   public static void addCoMPositionContinuityConstraint(int previousSequence,
                                                         int nextSequence,
                                                         int constraintRow,
                                                         double omega,
                                                         double previousDuration,
                                                         DMatrix constraintMatrixToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintRow, previousStartIndex, getCoMPositionFirstCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 1, getCoMPositionSecondCoefficientTimeFunction(omega, previousDuration));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(previousDuration, 0.0, minDuration))
      {
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 2, getCoMPositionThirdCoefficientTimeFunction(previousDuration));
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 3, getCoMPositionFourthCoefficientTimeFunction(previousDuration));
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 4, getCoMPositionFifthCoefficientTimeFunction(previousDuration));
      }
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 5, getCoMPositionSixthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, nextStartIndex, -getCoMPositionFirstCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 1, -getCoMPositionSecondCoefficientTimeFunction(omega, 0.0));
      if (SET_ZERO_VALUES)
      {
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 2, -getCoMPositionThirdCoefficientTimeFunction(0.0));
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 3, -getCoMPositionFourthCoefficientTimeFunction(0.0));
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 4, -getCoMPositionFifthCoefficientTimeFunction(0.0));
      }
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 5, -getCoMPositionSixthCoefficientTimeFunction());
   }

   /**
    * <p> Set a continuity constraint on the CoM position at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> x<sub>i-1</sub>(T<sub>i-1</sub>) = x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   public static void addCoMPositionContinuityObjective(double weight,
                                                        int previousSequence,
                                                        int nextSequence,
                                                        int constraintRow,
                                                        double omega,
                                                        double previousDuration,
                                                        DMatrix objectiveJacobianToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      double c00 = getCoMPositionFirstCoefficientTimeFunction(omega, previousDuration);
      double c01 = getCoMPositionFirstCoefficientTimeFunction(omega, previousDuration);
      double c05 = getCoMPositionSixthCoefficientTimeFunction();

      addEquals(objectiveJacobianToPack, constraintRow, previousStartIndex, weight * c00);
      addEquals(objectiveJacobianToPack, constraintRow, previousStartIndex + 1, weight * c01);
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(previousDuration, 0.0, minDuration))
      {
         double c02 = getCoMPositionThirdCoefficientTimeFunction(previousDuration);
         double c03 = getCoMPositionFourthCoefficientTimeFunction(previousDuration);
         double c04 = getCoMPositionFifthCoefficientTimeFunction(previousDuration);
         addEquals(objectiveJacobianToPack, constraintRow, previousStartIndex + 2, weight * c02);
         addEquals(objectiveJacobianToPack, constraintRow, previousStartIndex + 3, weight * c03);
         addEquals(objectiveJacobianToPack, constraintRow, previousStartIndex + 4, weight * c04);
      }
      addEquals(objectiveJacobianToPack, constraintRow, previousStartIndex + 5, weight * c05);

      double c10 = getCoMPositionFirstCoefficientTimeFunction(omega, 0.0);
      double c11 = getCoMPositionSecondCoefficientTimeFunction(omega, 0.0);
      double c15 = getCoMPositionSixthCoefficientTimeFunction();

      addEquals(objectiveJacobianToPack, constraintRow, nextStartIndex, -weight * c10);
      addEquals(objectiveJacobianToPack, constraintRow, nextStartIndex + 1, -weight * c11);
      if (SET_ZERO_VALUES)
      {
         double c12 = getCoMPositionThirdCoefficientTimeFunction(0.0);
         double c13 = getCoMPositionFourthCoefficientTimeFunction(0.0);
         double c14 = getCoMPositionFifthCoefficientTimeFunction(0.0);
         addEquals(objectiveJacobianToPack, constraintRow, nextStartIndex + 2, -weight * c12);
         addEquals(objectiveJacobianToPack, constraintRow, nextStartIndex + 3, -weight * c13);
         addEquals(objectiveJacobianToPack, constraintRow, nextStartIndex + 4, -weight * c14);
      }
      addEquals(objectiveJacobianToPack, constraintRow, nextStartIndex + 5, -weight * c15);
   }

   public static void addCoMPositionContinuityObjective(double weight,
                                                        int previousSequence,
                                                        int nextSequence,
                                                        double omega,
                                                        double previousDuration,
                                                        DMatrix hessianToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      double c00 = getCoMPositionFirstCoefficientTimeFunction(omega, previousDuration);
      double c01 = getCoMPositionFirstCoefficientTimeFunction(omega, previousDuration);
      double c05 = getCoMPositionSixthCoefficientTimeFunction();

      double c10 = -getCoMPositionFirstCoefficientTimeFunction(omega, 0.0);
      double c11 = -getCoMPositionSecondCoefficientTimeFunction(omega, 0.0);
      double c15 = -getCoMPositionSixthCoefficientTimeFunction();

      addEquals(hessianToPack, previousStartIndex, previousStartIndex, weight * c00 * c00);
      addEquals(hessianToPack, previousStartIndex, previousStartIndex + 1, weight * c00 * c01);
      addEquals(hessianToPack, previousStartIndex, previousStartIndex + 5, weight * c00 * c05);
      addEquals(hessianToPack, previousStartIndex, nextStartIndex, weight * c00 * c10);
      addEquals(hessianToPack, previousStartIndex, nextStartIndex + 1, weight * c00 * c11);
      addEquals(hessianToPack, previousStartIndex, nextStartIndex + 5, weight * c00 * c15);

      addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex, weight * c01 * c00);
      addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 1, weight * c01 * c01);
      addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 5, weight * c01 * c05);
      addEquals(hessianToPack, previousStartIndex + 1, nextStartIndex, weight * c01 * c10);
      addEquals(hessianToPack, previousStartIndex + 1, nextStartIndex + 1, weight * c01 * c11);
      addEquals(hessianToPack, previousStartIndex + 1, nextStartIndex + 5, weight * c01 * c15);

      addEquals(hessianToPack, previousStartIndex + 5, previousStartIndex, weight * c05 * c00);
      addEquals(hessianToPack, previousStartIndex + 5, previousStartIndex + 1, weight * c05 * c01);
      addEquals(hessianToPack, previousStartIndex + 5, previousStartIndex + 5, weight * c05 * c05);
      addEquals(hessianToPack, previousStartIndex + 5, nextStartIndex, weight * c05 * c10);
      addEquals(hessianToPack, previousStartIndex + 5, nextStartIndex + 1, weight * c05 * c11);
      addEquals(hessianToPack, previousStartIndex + 5, nextStartIndex + 5, weight * c05 * c15);

      addEquals(hessianToPack, nextStartIndex, previousStartIndex, weight * c10 * c00);
      addEquals(hessianToPack, nextStartIndex, previousStartIndex + 1, weight * c10 * c01);
      addEquals(hessianToPack, nextStartIndex, previousStartIndex + 5, weight * c10 * c05);
      addEquals(hessianToPack, nextStartIndex, nextStartIndex, weight * c10 * c10);
      addEquals(hessianToPack, nextStartIndex, nextStartIndex + 1, weight * c10 * c11);
      addEquals(hessianToPack, nextStartIndex, nextStartIndex + 5, weight * c10 * c15);

      addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex, weight * c11 * c00);
      addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 1, weight * c11 * c01);
      addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 5, weight * c11 * c05);
      addEquals(hessianToPack, nextStartIndex + 1, nextStartIndex, weight * c11 * c10);
      addEquals(hessianToPack, nextStartIndex + 1, nextStartIndex + 1, weight * c11 * c11);
      addEquals(hessianToPack, nextStartIndex + 1, nextStartIndex + 5, weight * c11 * c15);

      addEquals(hessianToPack, nextStartIndex + 5, previousStartIndex, weight * c15 * c00);
      addEquals(hessianToPack, nextStartIndex + 5, previousStartIndex + 1, weight * c15 * c01);
      addEquals(hessianToPack, nextStartIndex + 5, previousStartIndex + 5, weight * c15 * c05);
      addEquals(hessianToPack, nextStartIndex + 5, nextStartIndex, weight * c15 * c10);
      addEquals(hessianToPack, nextStartIndex + 5, nextStartIndex + 1, weight * c15 * c11);
      addEquals(hessianToPack, nextStartIndex + 5, nextStartIndex + 5, weight * c15 * c15);

      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(previousDuration, 0.0, minDuration))
      {
         double c02 = getCoMPositionThirdCoefficientTimeFunction(previousDuration);
         double c03 = getCoMPositionFourthCoefficientTimeFunction(previousDuration);
         double c04 = getCoMPositionFifthCoefficientTimeFunction(previousDuration);

         addEquals(hessianToPack, previousStartIndex, previousStartIndex + 2, weight * c00 * c02);
         addEquals(hessianToPack, previousStartIndex, previousStartIndex + 3, weight * c00 * c03);
         addEquals(hessianToPack, previousStartIndex, previousStartIndex + 4, weight * c00 * c04);

         addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 2, weight * c01 * c02);
         addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 3, weight * c01 * c03);
         addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 4, weight * c01 * c04);

         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex, weight * c02 * c00);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 1, weight * c02 * c01);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 2, weight * c02 * c02);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 3, weight * c02 * c03);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 4, weight * c02 * c04);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 5, weight * c02 * c05);
         addEquals(hessianToPack, previousStartIndex + 2, nextStartIndex, weight * c02 * c10);
         addEquals(hessianToPack, previousStartIndex + 2, nextStartIndex + 1, weight * c02 * c11);
         addEquals(hessianToPack, previousStartIndex + 2, nextStartIndex + 5, weight * c02 * c15);

         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex, weight * c03 * c00);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 1, weight * c03 * c01);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 2, weight * c03 * c02);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 3, weight * c03 * c03);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 4, weight * c03 * c04);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 5, weight * c03 * c05);
         addEquals(hessianToPack, previousStartIndex + 3, nextStartIndex, weight * c03 * c10);
         addEquals(hessianToPack, previousStartIndex + 3, nextStartIndex + 1, weight * c03 * c11);
         addEquals(hessianToPack, previousStartIndex + 3, nextStartIndex + 5, weight * c03 * c15);

         addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex, weight * c04 * c00);
         addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 1, weight * c04 * c01);
         addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 2, weight * c04 * c02);
         addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 3, weight * c04 * c03);
         addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 4, weight * c04 * c04);
         addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 5, weight * c04 * c05);
         addEquals(hessianToPack, previousStartIndex + 4, nextStartIndex, weight * c04 * c10);
         addEquals(hessianToPack, previousStartIndex + 4, nextStartIndex + 1, weight * c04 * c11);
         addEquals(hessianToPack, previousStartIndex + 4, nextStartIndex + 5, weight * c04 * c15);

         addEquals(hessianToPack, previousStartIndex + 5, previousStartIndex + 2, weight * c05 * c02);
         addEquals(hessianToPack, previousStartIndex + 5, previousStartIndex + 3, weight * c05 * c03);
         addEquals(hessianToPack, previousStartIndex + 5, previousStartIndex + 4, weight * c05 * c04);

         addEquals(hessianToPack, nextStartIndex, previousStartIndex + 2, weight * c10 * c02);
         addEquals(hessianToPack, nextStartIndex, previousStartIndex + 3, weight * c10 * c03);
         addEquals(hessianToPack, nextStartIndex, previousStartIndex + 4, weight * c10 * c04);

         addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 2, weight * c11 * c02);
         addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 3, weight * c11 * c03);
         addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 4, weight * c11 * c04);

         addEquals(hessianToPack, nextStartIndex + 5, previousStartIndex + 2, weight * c15 * c02);
         addEquals(hessianToPack, nextStartIndex + 5, previousStartIndex + 3, weight * c15 * c03);
         addEquals(hessianToPack, nextStartIndex + 5, previousStartIndex + 4, weight * c15 * c04);
      }
   }

   /**
    * <p> Set a continuity constraint on the CoM velocity at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    * d/dt x<sub>i</sub>(t<sub>i</sub>) = &omega; c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    * &omega; c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 3 c<sub>2,i</sub> t<sub>i</sub><sup>2</sup> +
    * 2 c<sub>3,i</sub> t<sub>i</sub> + c<sub>4,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> d / dt x<sub>i-1</sub>(T<sub>i-1</sub>) = d / dt x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   public static void addCoMVelocityContinuityConstraint(int previousSequence,
                                                         int nextSequence,
                                                         int constraintRow,
                                                         double omega,
                                                         double previousDuration,
                                                         DMatrix constraintMatrixToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintRow, previousStartIndex, getCoMVelocityFirstCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 1, getCoMVelocitySecondCoefficientTimeFunction(omega, previousDuration));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(previousDuration, 0.0, minDuration))
      {
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 2, getCoMVelocityThirdCoefficientTimeFunction(previousDuration));
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 3, getCoMVelocityFourthCoefficientTimeFunction(previousDuration));
      }
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 4, getCoMVelocityFifthCoefficientTimeFunction());

      constraintMatrixToPack.set(constraintRow, nextStartIndex, -getCoMVelocityFirstCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 1, -getCoMVelocitySecondCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 4, -getCoMVelocityFifthCoefficientTimeFunction());
      if (SET_ZERO_VALUES)
      {
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 5, getCoMVelocitySixthCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 2, -getCoMVelocityThirdCoefficientTimeFunction(0.0));
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 3, -getCoMVelocityFourthCoefficientTimeFunction(0.0));
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 5, -getCoMVelocitySixthCoefficientTimeFunction());
      }
   }

   public static void addCoMVelocityContinuityObjective(double weight,
                                                        int previousSequence,
                                                        int nextSequence,
                                                        int row,
                                                        double omega,
                                                        double previousDuration,
                                                        DMatrix coefficientJacobianToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      addEquals(coefficientJacobianToPack, row, previousStartIndex, weight * getCoMVelocityFirstCoefficientTimeFunction(omega, previousDuration));
      addEquals(coefficientJacobianToPack, row, previousStartIndex + 1, weight * getCoMVelocitySecondCoefficientTimeFunction(omega, previousDuration));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(previousDuration, 0.0, minDuration))
      {
         addEquals(coefficientJacobianToPack, row, previousStartIndex + 2, weight * getCoMVelocityThirdCoefficientTimeFunction(previousDuration));
         addEquals(coefficientJacobianToPack, row, previousStartIndex + 3, weight * getCoMVelocityFourthCoefficientTimeFunction(previousDuration));
      }
      addEquals(coefficientJacobianToPack, row, previousStartIndex + 4, weight * getCoMVelocityFifthCoefficientTimeFunction());

      addEquals(coefficientJacobianToPack, row, nextStartIndex, -weight * getCoMVelocityFirstCoefficientTimeFunction(omega, 0.0));
      addEquals(coefficientJacobianToPack, row, nextStartIndex + 1, -weight * getCoMVelocitySecondCoefficientTimeFunction(omega, 0.0));
      addEquals(coefficientJacobianToPack, row, nextStartIndex + 4, -weight * getCoMVelocityFifthCoefficientTimeFunction());
      if (SET_ZERO_VALUES)
      {
         addEquals(coefficientJacobianToPack, row, previousStartIndex + 5, weight * getCoMVelocitySixthCoefficientTimeFunction());
         addEquals(coefficientJacobianToPack, row, nextStartIndex + 2, -weight * getCoMVelocityThirdCoefficientTimeFunction(0.0));
         addEquals(coefficientJacobianToPack, row, nextStartIndex + 3, -weight * getCoMVelocityFourthCoefficientTimeFunction(0.0));
         addEquals(coefficientJacobianToPack, row, nextStartIndex + 5, -weight * getCoMVelocitySixthCoefficientTimeFunction());
      }
   }

   public static void addCoMVelocityContinuityObjective(double weight,
                                                        int previousSequence,
                                                        int nextSequence,
                                                        double omega,
                                                        double previousDuration,
                                                        DMatrix hessianToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      double c00 = getCoMVelocityFirstCoefficientTimeFunction(omega, previousDuration);
      double c01 = getCoMVelocitySecondCoefficientTimeFunction(omega, previousDuration);
      double c04 = getCoMVelocityFifthCoefficientTimeFunction();

      double c10 = -getCoMVelocityFirstCoefficientTimeFunction(omega, 0.0);
      double c11 = -getCoMVelocitySecondCoefficientTimeFunction(omega, 0.0);
      double c14 = -getCoMVelocityFifthCoefficientTimeFunction();

      addEquals(hessianToPack, previousStartIndex, previousStartIndex, weight * c00 * c00);
      addEquals(hessianToPack, previousStartIndex, previousStartIndex + 1, weight * c00 * c01);
      addEquals(hessianToPack, previousStartIndex, previousStartIndex + 4, weight * c00 * c04);
      addEquals(hessianToPack, previousStartIndex, nextStartIndex, weight * c00 * c10);
      addEquals(hessianToPack, previousStartIndex, nextStartIndex + 1, weight * c00 * c11);
      addEquals(hessianToPack, previousStartIndex, nextStartIndex + 4, weight * c00 * c14);

      addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex, weight * c01 * c00);
      addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 1, weight * c01 * c01);
      addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 4, weight * c01 * c04);
      addEquals(hessianToPack, previousStartIndex + 1, nextStartIndex, weight * c01 * c10);
      addEquals(hessianToPack, previousStartIndex + 1, nextStartIndex + 1, weight * c01 * c11);
      addEquals(hessianToPack, previousStartIndex + 1, nextStartIndex + 4, weight * c01 * c14);

      addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex, weight * c04 * c00);
      addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 1, weight * c04 * c01);
      addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 4, weight * c04 * c04);
      addEquals(hessianToPack, previousStartIndex + 4, nextStartIndex, weight * c04 * c10);
      addEquals(hessianToPack, previousStartIndex + 4, nextStartIndex + 1, weight * c04 * c11);
      addEquals(hessianToPack, previousStartIndex + 4, nextStartIndex + 4, weight * c04 * c14);

      addEquals(hessianToPack, nextStartIndex, previousStartIndex, weight * c10 * c00);
      addEquals(hessianToPack, nextStartIndex, previousStartIndex + 1, weight * c10 * c01);
      addEquals(hessianToPack, nextStartIndex, previousStartIndex + 4, weight * c10 * c04);
      addEquals(hessianToPack, nextStartIndex, nextStartIndex, weight * c10 * c10);
      addEquals(hessianToPack, nextStartIndex, nextStartIndex + 1, weight * c10 * c11);
      addEquals(hessianToPack, nextStartIndex, nextStartIndex + 4, weight * c10 * c14);

      addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex, weight * c11 * c00);
      addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 1, weight * c11 * c01);
      addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 4, weight * c11 * c04);
      addEquals(hessianToPack, nextStartIndex + 1, nextStartIndex, weight * c11 * c10);
      addEquals(hessianToPack, nextStartIndex + 1, nextStartIndex + 1, weight * c11 * c11);
      addEquals(hessianToPack, nextStartIndex + 1, nextStartIndex + 4, weight * c11 * c14);

      addEquals(hessianToPack, nextStartIndex + 4, previousStartIndex, weight * c14 * c00);
      addEquals(hessianToPack, nextStartIndex + 4, previousStartIndex + 1, weight * c14 * c01);
      addEquals(hessianToPack, nextStartIndex + 4, previousStartIndex + 4, weight * c14 * c04);
      addEquals(hessianToPack, nextStartIndex + 4, nextStartIndex, weight * c14 * c10);
      addEquals(hessianToPack, nextStartIndex + 4, nextStartIndex + 1, weight * c14 * c11);
      addEquals(hessianToPack, nextStartIndex + 4, nextStartIndex + 4, weight * c14 * c14);

      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(previousDuration, 0.0, minDuration))
      {
         double c02 = getCoMVelocityThirdCoefficientTimeFunction(previousDuration);
         double c03 = getCoMVelocityFourthCoefficientTimeFunction(previousDuration);

         addEquals(hessianToPack, previousStartIndex, previousStartIndex + 2, weight * c00 * c02);
         addEquals(hessianToPack, previousStartIndex, previousStartIndex + 3, weight * c00 * c03);

         addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 2, weight * c01 * c02);
         addEquals(hessianToPack, previousStartIndex + 1, previousStartIndex + 3, weight * c01 * c03);

         addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 2, weight * c04 * c02);
         addEquals(hessianToPack, previousStartIndex + 4, previousStartIndex + 3, weight * c04 * c03);

         addEquals(hessianToPack, nextStartIndex, previousStartIndex + 2, weight * c10 * c02);
         addEquals(hessianToPack, nextStartIndex, previousStartIndex + 3, weight * c10 * c03);

         addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 2, weight * c11 * c02);
         addEquals(hessianToPack, nextStartIndex + 1, previousStartIndex + 3, weight * c11 * c03);

         addEquals(hessianToPack, nextStartIndex + 4, previousStartIndex + 2, weight * c14 * c02);
         addEquals(hessianToPack, nextStartIndex + 4, previousStartIndex + 3, weight * c14 * c03);

         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex, weight * c02 * c00);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 1, weight * c02 * c01);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 2, weight * c02 * c02);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 3, weight * c02 * c03);
         addEquals(hessianToPack, previousStartIndex + 2, previousStartIndex + 4, weight * c02 * c04);
         addEquals(hessianToPack, previousStartIndex + 2, nextStartIndex, weight * c02 * c10);
         addEquals(hessianToPack, previousStartIndex + 2, nextStartIndex + 1, weight * c02 * c11);
         addEquals(hessianToPack, previousStartIndex + 2, nextStartIndex + 4, weight * c02 * c14);

         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex, weight * c03 * c00);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 1, weight * c03 * c01);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 2, weight * c03 * c02);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 3, weight * c03 * c03);
         addEquals(hessianToPack, previousStartIndex + 3, previousStartIndex + 4, weight * c03 * c04);
         addEquals(hessianToPack, previousStartIndex + 3, nextStartIndex, weight * c03 * c10);
         addEquals(hessianToPack, previousStartIndex + 3, nextStartIndex + 1, weight * c03 * c11);
         addEquals(hessianToPack, previousStartIndex + 3, nextStartIndex + 4, weight * c03 * c14);
      }
   }

   /**
    * <p> Set a continuity constraint on the VRP position at a state change, aka a trajectory knot.. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    * x<sub>i</sub>(t<sub>i</sub>) = c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> + c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> +
    * c<sub>2,i</sub> t<sub>i</sub><sup>3</sup> + c<sub>3,i</sub> t<sub>i</sub><sup>2</sup> +
    * c<sub>4,i</sub> t<sub>i</sub> + c<sub>5,i</sub>.
    * </p>
    * <p> This constraint is then defined as </p>
    * <p> x<sub>i-1</sub>(T<sub>i-1</sub>) = x<sub>i</sub>(0), </p>
    * <p> substituting in the trajectory coefficients. </p>
    *
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   public static void addVRPPositionContinuityConstraint(int previousSequence,
                                                         int nextSequence,
                                                         int constraintRow,
                                                         double omega,
                                                         double previousDuration,
                                                         DMatrix constraintMatrixToPack)
   {
      // move next sequence coefficients to the left hand side
      int previousStartIndex = 6 * previousSequence;
      int nextStartIndex = 6 * nextSequence;

      previousDuration = Math.min(previousDuration, sufficientlyLongTime);

      if (SET_ZERO_VALUES)
      {
         constraintMatrixToPack.set(constraintRow, previousStartIndex, getVRPPositionFirstCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 1, getVRPPositionSecondCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintRow, nextStartIndex, -getVRPPositionFirstCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 1, -getVRPPositionSecondCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 2, -getVRPPositionThirdCoefficientTimeFunction(omega, 0.0));
         constraintMatrixToPack.set(constraintRow, nextStartIndex + 4, -getVRPPositionFifthCoefficientTimeFunction(0.0));
      }
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(previousDuration, 0.0, minDuration))
      {
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 2, getVRPPositionThirdCoefficientTimeFunction(omega, previousDuration));
         constraintMatrixToPack.set(constraintRow, previousStartIndex + 4, getVRPPositionFifthCoefficientTimeFunction(previousDuration));
      }
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 3, getVRPPositionFourthCoefficientTimeFunction(omega, previousDuration));
      constraintMatrixToPack.set(constraintRow, previousStartIndex + 5, getVRPPositionSixthCoefficientTimeFunction());
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 3, -getVRPPositionFourthCoefficientTimeFunction(omega, 0.0));
      constraintMatrixToPack.set(constraintRow, nextStartIndex + 5, -getVRPPositionSixthCoefficientTimeFunction());
   }

   public static void addEquivalentVRPVelocityConstraint(int segmentId1,
                                                         int segmentId2,
                                                         int constraintNumber,
                                                         double timeOfConstraint1,
                                                         double timeOfConstraint2,
                                                         double omega,
                                                         DMatrix constraintMatrixToPack,
                                                         DMatrix xObjectiveMatrixToPack,
                                                         DMatrix yObjectiveMatrixToPack,
                                                         DMatrix zObjectiveMatrixToPack)
   {
      int startIndex1 = 6 * segmentId1;
      int startIndex2 = 6 * segmentId2;

      timeOfConstraint1 = Math.min(timeOfConstraint1, sufficientlyLongTime);
      timeOfConstraint2 = Math.min(timeOfConstraint2, sufficientlyLongTime);

      if (SET_ZERO_VALUES)
      {
         addEquals(constraintMatrixToPack, constraintNumber, startIndex1 + 0, CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction());
         addEquals(constraintMatrixToPack, constraintNumber, startIndex2 + 0, -CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction());
         addEquals(constraintMatrixToPack, constraintNumber, startIndex1 + 1, CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction());
         addEquals(constraintMatrixToPack, constraintNumber, startIndex2 + 1, -CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction());
         addEquals(constraintMatrixToPack, constraintNumber, startIndex1 + 5, CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction());
         addEquals(constraintMatrixToPack, constraintNumber, startIndex2 + 5, -CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction());
      }
      addEquals(constraintMatrixToPack,
                constraintNumber,
                startIndex1 + 2,
                CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, timeOfConstraint1));
      addEquals(constraintMatrixToPack,
                constraintNumber,
                startIndex2 + 2,
                -CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, timeOfConstraint2));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(timeOfConstraint1, 0.0, minDuration))
         addEquals(constraintMatrixToPack,
                   constraintNumber,
                   startIndex1 + 3,
                   CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(timeOfConstraint1));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(timeOfConstraint2, 0.0, minDuration))
         addEquals(constraintMatrixToPack,
                   constraintNumber,
                   startIndex2 + 3,
                   -CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(timeOfConstraint2));
      addEquals(constraintMatrixToPack, constraintNumber, startIndex1 + 4, CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction());
      addEquals(constraintMatrixToPack, constraintNumber, startIndex2 + 4, -CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction());

      xObjectiveMatrixToPack.set(constraintNumber, 0, 0.0);
      yObjectiveMatrixToPack.set(constraintNumber, 0, 0.0);
      zObjectiveMatrixToPack.set(constraintNumber, 0, 0.0);
   }

   public static void addImplicitVRPVelocityConstraint(int sequenceId,
                                                       int constraintNumber,
                                                       int vrpWaypointPositionIndex,
                                                       double timeInSegment,
                                                       double timeOfRelativePosition,
                                                       double omega,
                                                       FramePoint3DReadOnly relativeDesiredVRPPosition,
                                                       DMatrix constraintMatrixToPack,
                                                       DMatrix xObjectiveMatrixToPack,
                                                       DMatrix yObjectiveMatrixToPack,
                                                       DMatrix zObjectiveMatrixToPack,
                                                       DMatrix vrpWaypointJacobianToPack)
   {
      int startIndex = 6 * sequenceId;

      timeInSegment = Math.min(timeInSegment, sufficientlyLongTime);

      relativeDesiredVRPPosition.checkReferenceFrameMatch(worldFrame);

      double duration = Math.min(sufficientlyLarge, timeInSegment - timeOfRelativePosition);
      boolean timeInSegmentIsNonZero = !MathTools.epsilonEquals(timeInSegment, 0.0, minDuration);
      boolean durationIsNonZero = !MathTools.epsilonEquals(duration, 0.0, minDuration);

      if (SET_ZERO_VALUES)
      {
         constraintMatrixToPack.set(constraintNumber,
                                    startIndex + 0,
                                    CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction()
                                    - duration * CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintNumber,
                                    startIndex + 1,
                                    CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction()
                                    - duration * CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction());
      }
      if (SET_ZERO_VALUES || timeInSegmentIsNonZero || durationIsNonZero)
      {
         constraintMatrixToPack.set(constraintNumber,
                                    startIndex + 2,
                                    CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, timeInSegment)
                                    - duration * CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, timeOfRelativePosition));
         constraintMatrixToPack.set(constraintNumber,
                                    startIndex + 4,
                                    CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(timeInSegment)
                                    - duration * CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction());
      }
      constraintMatrixToPack.set(constraintNumber,
                                 startIndex + 3,
                                 CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, timeInSegment)
                                 - duration * CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(timeOfRelativePosition));
      constraintMatrixToPack.set(constraintNumber,
                                 startIndex + 5,
                                 CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction()
                                 - duration * CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction());

      vrpWaypointJacobianToPack.set(constraintNumber, vrpWaypointPositionIndex, 1.0);

      xObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, relativeDesiredVRPPosition.getX());
      yObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, relativeDesiredVRPPosition.getY());
      zObjectiveMatrixToPack.set(vrpWaypointPositionIndex, 0, relativeDesiredVRPPosition.getZ());
   }

   public static void addEquals(DMatrix matrixToPack, int row, int col, double val)
   {
      if (col < 0 || col >= matrixToPack.getNumCols())
         throw new IllegalArgumentException("Specified col is out of bounds");
      if (row < 0 || row >= matrixToPack.getNumRows())
         throw new IllegalArgumentException("Specified row is out of bounds");

      matrixToPack.unsafe_set(row, col, val + matrixToPack.unsafe_get(row, col));
   }

   /**
    * <p> Adds a constraint for the CoM trajectory to have an acceleration equal to gravity at time t.</p>
    * <p> Recall that the CoM acceleration is defined as </p>
    * d<sup>2</sup> / dt<sup>2</sup> x<sub>i</sub>(t<sub>i</sub>) = &omega;<sup>2</sup> c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> +
    * &omega;<sup>2</sup> c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 6 c<sub>2,i</sub> t<sub>i</sub> + 2 c<sub>3,i</sub>
    * <p> This constraint then states that </p>
    * <p> d<sup>2</sup> / dt<sup>2</sup> x<sub>i</sub>(t<sub>i</sub>) = -g, </p>
    * <p> substituting in the appropriate coefficients. </p>
    *
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainCoMAccelerationToGravity(int sequenceId,
                                                        int constraintRow,
                                                        double omega,
                                                        double time,
                                                        double gravityZ,
                                                        DMatrix constraintMatrixToPack,
                                                        DMatrix zObjectiveMatrixToPack)
   {
      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      constraintMatrixToPack.set(constraintRow, startIndex, getCoMAccelerationFirstCoefficientTimeFunction(omega, time));
      constraintMatrixToPack.set(constraintRow, startIndex + 1, getCoMAccelerationSecondCoefficientTimeFunction(omega, time));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         constraintMatrixToPack.set(constraintRow, startIndex + 2, getCoMAccelerationThirdCoefficientTimeFunction(time));
      }
      constraintMatrixToPack.set(constraintRow, startIndex + 3, getCoMAccelerationFourthCoefficientTimeFunction());
      if (SET_ZERO_VALUES)
      {
         constraintMatrixToPack.set(constraintRow, startIndex + 4, getCoMAccelerationFifthCoefficientTimeFunction());
         constraintMatrixToPack.set(constraintRow, startIndex + 5, getCoMAccelerationSixthCoefficientTimeFunction());
      }

      zObjectiveMatrixToPack.set(constraintRow, 0, -Math.abs(gravityZ));
   }

   public static void addCoMAccelerationIsGravityObjective(double weight,
                                                           int sequenceId,
                                                           int constraintRow,
                                                           double omega,
                                                           double time,
                                                           double gravityZ,
                                                           DMatrix constraintMatrixToPack,
                                                           DMatrix zObjectiveMatrixToPack)
   {
      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);

      addEquals(constraintMatrixToPack, constraintRow, startIndex, weight * getCoMAccelerationFirstCoefficientTimeFunction(omega, time));
      addEquals(constraintMatrixToPack, constraintRow, startIndex + 1, weight * getCoMAccelerationSecondCoefficientTimeFunction(omega, time));
      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         addEquals(constraintMatrixToPack, constraintRow, startIndex + 2, weight * getCoMAccelerationThirdCoefficientTimeFunction(time));
      }
      addEquals(constraintMatrixToPack, constraintRow, startIndex + 3, weight * getCoMAccelerationFourthCoefficientTimeFunction());
      if (SET_ZERO_VALUES)
      {
         addEquals(constraintMatrixToPack, constraintRow, startIndex + 4, weight * getCoMAccelerationFifthCoefficientTimeFunction());
         addEquals(constraintMatrixToPack, constraintRow, startIndex + 5, weight * getCoMAccelerationSixthCoefficientTimeFunction());
      }

      addEquals(zObjectiveMatrixToPack, constraintRow, 0, weight * -Math.abs(gravityZ));
   }

   public static void addCoMAccelerationIsGravityObjective(double weight,
                                                           int sequenceId,
                                                           double omega,
                                                           double time,
                                                           double gravityZ,
                                                           DMatrix hessianToPack,
                                                           DMatrix zGradientToPack)
   {
      int startIndex = 6 * sequenceId;

      time = Math.min(time, sufficientlyLongTime);
      gravityZ = -Math.abs(gravityZ);

      double c0 = getCoMAccelerationFirstCoefficientTimeFunction(omega, time);
      double c1 = getCoMAccelerationSecondCoefficientTimeFunction(omega, time);
      double c3 = getCoMAccelerationFourthCoefficientTimeFunction();


      addEquals(hessianToPack, startIndex, startIndex, weight * c0 * c0);
      addEquals(hessianToPack, startIndex, startIndex + 1, weight * c0 * c1);
      addEquals(hessianToPack, startIndex, startIndex + 3, weight * c0 * c3);

      addEquals(hessianToPack, startIndex + 1, startIndex, weight * c1 * c0);
      addEquals(hessianToPack, startIndex + 1, startIndex + 1, weight * c1 * c1);
      addEquals(hessianToPack, startIndex + 1, startIndex + 3, weight * c1 * c3);

      addEquals(hessianToPack, startIndex + 3, startIndex, weight * c3 * c0);
      addEquals(hessianToPack, startIndex + 3, startIndex + 1, weight * c3 * c1);
      addEquals(hessianToPack, startIndex + 3, startIndex + 3, weight * c3 * c3);

      addEquals(zGradientToPack, startIndex, 0, 2.0 * weight * gravityZ * c0);
      addEquals(zGradientToPack, startIndex + 1, 0, 2.0 * weight * gravityZ * c1);
      addEquals(zGradientToPack, startIndex + 3, 0, 2.0 * weight * gravityZ * c3);

      if (SET_ZERO_VALUES || !MathTools.epsilonEquals(time, 0.0, minDuration))
      {
         double c2 = getCoMAccelerationThirdCoefficientTimeFunction(time);

         addEquals(hessianToPack, startIndex, startIndex + 2, weight * c0 * c2);

         addEquals(hessianToPack, startIndex + 1, startIndex + 2, weight * c1 * c2);

         addEquals(hessianToPack, startIndex + 2, startIndex, weight * c2 * c0);
         addEquals(hessianToPack, startIndex + 2, startIndex + 1, weight * c2 * c1);
         addEquals(hessianToPack, startIndex + 2, startIndex + 2, weight * c2 * c2);
         addEquals(hessianToPack, startIndex + 2, startIndex + 3, weight * c2 * c3);

         addEquals(hessianToPack, startIndex + 3, startIndex + 2, weight * c3 * c2);

         addEquals(zGradientToPack, startIndex + 2, 0, 2.0 * weight * gravityZ * c2);
      }
   }

   /**
    * <p> Adds a constraint for the CoM trajectory to have a jerk equal to 0.0 at time t.</p>
    * <p> Recall that the CoM jerk is defined as </p>
    * d<sup>3</sup> / dt<sup>3</sup> x<sub>i</sub>(t<sub>i</sub>) = &omega;<sup>3</sup> c<sub>0,i</sub> e<sup>&omega; t<sub>i</sub></sup> -
    * &omega;<sup>3</sup> c<sub>1,i</sub> e<sup>-&omega; t<sub>i</sub></sup> + 6 c<sub>2,i</sub>
    * <p> This constraint then states that </p>
    * <p> d<sup>3</sup> / dt<sup>3</sup> x<sub>i</sub>(t<sub>i</sub>) = 0.0, </p>
    * <p> substituting in the appropriate coefficients. </p>
    *
    * @param sequenceId segment of interest, i in the above equations.
    * @param time time for the constraint, t<sub>i</sub> in the above equations.
    */
   public static void constrainCoMJerkToZero(double time, double omega, int sequenceId, int rowStart, DMatrix matrixToPack)
   {
      time = Math.min(time, sufficientlyLongTime);

      int colStart = 6 * sequenceId;
      matrixToPack.set(rowStart, colStart, getCoMJerkFirstCoefficientTimeFunction(omega, time));
      matrixToPack.set(rowStart, colStart + 1, getCoMJerkSecondCoefficientTimeFunction(omega, time));
      matrixToPack.set(rowStart, colStart + 2, getCoMJerkThirdCoefficientTimeFunction());
      if (SET_ZERO_VALUES)
      {
         matrixToPack.set(rowStart, colStart + 3, getCoMJerkFourthCoefficientTimeFunction());
         matrixToPack.set(rowStart, colStart + 4, getCoMJerkFifthCoefficientTimeFunction());
         matrixToPack.set(rowStart, colStart + 5, getCoMJerkSixthCoefficientTimeFunction());
      }
   }

   public static void addZeroCoMJerkObjective(double weight, double time, double omega, int sequenceId, int rowStart, DMatrix matrixToPack)
   {
      time = Math.min(time, sufficientlyLongTime);

      int colStart = 6 * sequenceId;
      addEquals(matrixToPack, rowStart, colStart, weight * getCoMJerkFirstCoefficientTimeFunction(omega, time));
      addEquals(matrixToPack, rowStart, colStart + 1, weight * getCoMJerkSecondCoefficientTimeFunction(omega, time));
      addEquals(matrixToPack, rowStart, colStart + 2, weight * getCoMJerkThirdCoefficientTimeFunction());
      if (SET_ZERO_VALUES)
      {
         addEquals(matrixToPack, rowStart, colStart + 3, weight * getCoMJerkFourthCoefficientTimeFunction());
         addEquals(matrixToPack, rowStart, colStart + 4, weight * getCoMJerkFifthCoefficientTimeFunction());
         addEquals(matrixToPack, rowStart, colStart + 5, weight * getCoMJerkSixthCoefficientTimeFunction());
      }
   }

   public static void addZeroCoMJerkObjective(double weight, double time, double omega, int sequenceId,  DMatrix matrixToPack)
   {
      time = Math.min(time, sufficientlyLongTime);

      double c0 = getCoMJerkFirstCoefficientTimeFunction(omega, time);
      double c1 = getCoMJerkSecondCoefficientTimeFunction(omega, time);
      double c2 = getCoMJerkThirdCoefficientTimeFunction();

      int startIdx = 6 * sequenceId;
      addEquals(matrixToPack, startIdx, startIdx, weight * c0 * c0);
      addEquals(matrixToPack, startIdx, startIdx + 1, weight * c0 * c1);
      addEquals(matrixToPack, startIdx, startIdx + 2, weight * c0 * c2);

      addEquals(matrixToPack, startIdx + 1, startIdx, weight * c1 * c0);
      addEquals(matrixToPack, startIdx + 1, startIdx + 1, weight * c1 * c1);
      addEquals(matrixToPack, startIdx + 1, startIdx + 2, weight * c1 * c2);

      addEquals(matrixToPack, startIdx + 2, startIdx, weight * c2 * c0);
      addEquals(matrixToPack, startIdx + 2, startIdx + 1, weight * c2 * c1);
      addEquals(matrixToPack, startIdx + 2, startIdx + 2, weight * c2 * c2);
   }

   public static void addMinimizeCoMAccelerationObjective(double weight, double startTime, double endTime, double omega, int sequenceId, DMatrix hessianToPack)
   {
      int start = 6 * sequenceId;

      double h00End = getIntegralCoMAccelerationCoefficient00(omega, endTime);
      double h01End = getIntegralCoMAccelerationCoefficient01(omega, endTime);
      double h02End = getIntegralCoMAccelerationCoefficient02(omega, endTime);
      double h03End = getIntegralCoMAccelerationCoefficient03(omega, endTime);
      double h11End = getIntegralCoMAccelerationCoefficient11(omega, endTime);
      double h12End = getIntegralCoMAccelerationCoefficient12(omega, endTime);
      double h13End = getIntegralCoMAccelerationCoefficient13(omega, endTime);
      double h22End = getIntegralCoMAccelerationCoefficient22(endTime);
      double h23End = getIntegralCoMAccelerationCoefficient23(endTime);
      double h33End = getIntegralCoMAccelerationCoefficient33(endTime);
      double h00Start = getIntegralCoMAccelerationCoefficient00(omega, startTime);
      double h01Start = getIntegralCoMAccelerationCoefficient01(omega, startTime);
      double h02Start = getIntegralCoMAccelerationCoefficient02(omega, startTime);
      double h03Start = getIntegralCoMAccelerationCoefficient03(omega, startTime);
      double h11Start = getIntegralCoMAccelerationCoefficient11(omega, startTime);
      double h12Start = getIntegralCoMAccelerationCoefficient12(omega, startTime);
      double h13Start = getIntegralCoMAccelerationCoefficient13(omega, startTime);
      double h22Start = getIntegralCoMAccelerationCoefficient22(startTime);
      double h23Start = getIntegralCoMAccelerationCoefficient23(startTime);
      double h33Start = getIntegralCoMAccelerationCoefficient33(startTime);

      addEquals(hessianToPack, start + 0, start + 0, weight * (h00End - h00Start));
      addEquals(hessianToPack, start + 0, start + 1, weight * (h01End - h01Start));
      addEquals(hessianToPack, start + 0, start + 2, weight * (h02End - h02Start));
      addEquals(hessianToPack, start + 0, start + 3, weight * (h03End - h03Start));
      addEquals(hessianToPack, start + 1, start + 0, weight * (h01End - h01Start));
      addEquals(hessianToPack, start + 1, start + 1, weight * (h11End - h11Start));
      addEquals(hessianToPack, start + 1, start + 2, weight * (h12End - h12Start));
      addEquals(hessianToPack, start + 1, start + 3, weight * (h13End - h13Start));
      addEquals(hessianToPack, start + 2, start + 0, weight * (h02End - h02Start));
      addEquals(hessianToPack, start + 2, start + 1, weight * (h12End - h12Start));
      addEquals(hessianToPack, start + 2, start + 2, weight * (h22End - h22Start));
      addEquals(hessianToPack, start + 2, start + 3, weight * (h23End - h23Start));
      addEquals(hessianToPack, start + 3, start + 0, weight * (h03End - h03Start));
      addEquals(hessianToPack, start + 3, start + 1, weight * (h13End - h13Start));
      addEquals(hessianToPack, start + 3, start + 2, weight * (h23End - h23Start));
      addEquals(hessianToPack, start + 3, start + 3, weight * (h33End - h33Start));
   }

   public static void addMinimizeCoMJerkObjective(double weight, double startTime, double endTime, double omega, int sequenceId, DMatrix hessianToPack)
   {
      int start = 6 * sequenceId;

      double h00End = getIntegralCoMJerkCoefficient00(omega, endTime);
      double h01End = getIntegralCoMJerkCoefficient01(omega, endTime);
      double h02End = getIntegralCoMJerkCoefficient02(omega, endTime);
      double h11End = getIntegralCoMJerkCoefficient11(omega, endTime);
      double h12End = getIntegralCoMJerkCoefficient12(omega, endTime);
      double h22End = getIntegralCoMJerkCoefficient22(endTime);
      double h00Start = getIntegralCoMJerkCoefficient00(omega, startTime);
      double h01Start = getIntegralCoMJerkCoefficient01(omega, startTime);
      double h02Start = getIntegralCoMJerkCoefficient02(omega, startTime);
      double h11Start = getIntegralCoMJerkCoefficient11(omega, startTime);
      double h12Start = getIntegralCoMJerkCoefficient12(omega, startTime);
      double h22Start = getIntegralCoMJerkCoefficient22(startTime);

      addEquals(hessianToPack, start + 0, start + 0, weight * (h00End - h00Start));
      addEquals(hessianToPack, start + 0, start + 1, weight * (h01End - h01Start));
      addEquals(hessianToPack, start + 0, start + 2, weight * (h02End - h02Start));
      addEquals(hessianToPack, start + 1, start + 0, weight * (h01End - h01Start));
      addEquals(hessianToPack, start + 1, start + 1, weight * (h11End - h11Start));
      addEquals(hessianToPack, start + 1, start + 2, weight * (h12End - h12Start));
      addEquals(hessianToPack, start + 2, start + 0, weight * (h02End - h02Start));
      addEquals(hessianToPack, start + 2, start + 1, weight * (h12End - h12Start));
      addEquals(hessianToPack, start + 2, start + 2, weight * (h22End - h22Start));
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

   public static double getIntegralCoMAccelerationCoefficient00(double omega, double time)
   {
      return 0.5 * omega * omega * omega * Math.min(sufficientlyLarge, Math.exp(2.0 * omega * time));
   }

   public static double getIntegralCoMAccelerationCoefficient01(double omega, double time)
   {
      return omega * omega * omega * omega * Math.min(sufficientlyLarge, time);
   }

   public static double getIntegralCoMAccelerationCoefficient02(double omega, double time)
   {
      return 6 * Math.min(sufficientlyLarge, Math.exp(omega * time)) * (omega * Math.min(sufficientlyLarge, time) - 1.0);
   }

   public static double getIntegralCoMAccelerationCoefficient03(double omega, double time)
   {
      return 2 * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   public static double getIntegralCoMAccelerationCoefficient11(double omega, double time)
   {
      return -0.5 * omega * omega * omega * Math.min(sufficientlyLarge, Math.exp(-2.0 * omega * time));
   }

   public static double getIntegralCoMAccelerationCoefficient12(double omega, double time)
   {
      return -6 * Math.min(sufficientlyLarge, Math.exp(-omega * time)) * (omega * Math.min(sufficientlyLarge, time) + 1.0);
   }

   public static double getIntegralCoMAccelerationCoefficient13(double omega, double time)
   {
      return -2 * omega * Math.min(sufficientlyLarge, Math.exp(-omega * time));
   }

   public static double getIntegralCoMAccelerationCoefficient22(double time)
   {
      return 12.0 * Math.min(sufficientlyLarge, time * time * time);
   }

   public static double getIntegralCoMAccelerationCoefficient23(double time)
   {
      return 6.0 * Math.min(sufficientlyLarge, time * time);
   }

   public static double getIntegralCoMAccelerationCoefficient33(double time)
   {
      return 4.0 * Math.min(sufficientlyLarge, time);
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

   public static double getIntegralCoMJerkCoefficient00(double omega, double time)
   {
      double omega2 = omega * omega;
      double omega4 = omega2 * omega2;
      return 0.5 * omega2 * omega4 * Math.min(sufficientlyLarge, Math.exp(2.0 * omega * time));
   }

   public static double getIntegralCoMJerkCoefficient01(double omega, double time)
   {
      double omega3 = omega * omega * omega;
      return -omega3 * omega3 * Math.min(sufficientlyLarge, time);
   }

   public static double getIntegralCoMJerkCoefficient02(double omega, double time)
   {
      return 6.0 * omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   public static double getIntegralCoMJerkCoefficient11(double omega, double time)
   {
      double omega2 = omega * omega;
      double omega4 = omega2 * omega2;
      return -0.5 * omega4 * omega2 * Math.min(sufficientlyLarge, Math.exp(-2.0 * omega * time));
   }

   public static double getIntegralCoMJerkCoefficient12(double omega, double time)
   {
      return -6.0 * omega * omega * Math.min(sufficientlyLarge, Math.exp(-omega * time));
   }

   public static double getIntegralCoMJerkCoefficient22(double time)
   {
      return 36.0 * Math.min(sufficientlyLarge, time);
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

   public static void constructDesiredCoMPosition(FixedFramePoint3DBasics comPositionToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
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

   public static void constructDesiredCoMVelocity(FixedFrameVector3DBasics comVelocityToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
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

   public static void constructDesiredCoMAcceleration(FixedFrameVector3DBasics comAccelerationToPack,
                                                      FramePoint3DReadOnly firstCoefficient,
                                                      FramePoint3DReadOnly secondCoefficient,
                                                      FramePoint3DReadOnly thirdCoefficient,
                                                      FramePoint3DReadOnly fourthCoefficient,
                                                      FramePoint3DReadOnly fifthCoefficient,
                                                      FramePoint3DReadOnly sixthCoefficient,
                                                      double timeInPhase,
                                                      double omega)
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

   public static void constructDesiredDCMPosition(FixedFramePoint3DBasics dcmPositionToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
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

   public static void constructDesiredVRPPosition(FixedFramePoint3DBasics vrpPositionToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
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

   public static void constructDesiredVRPVelocity(FixedFrameVector3DBasics vrpVelocityToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
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
