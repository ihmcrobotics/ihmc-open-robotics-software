package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.*;

/**
 * <p>
 *    This is the main class of the trajectory-based CoM Trajectory Planner.
 * </p>
 * <p>
 *    This class assumes that the final phase is always the "stopping" phase, where the CoM is supposed to come to rest.
 *    This means that the final VRP is the terminal DCM location
 *  </p>
 *  <p>
 *     The CoM has the following definitions:
 *     <li>      x(t) = c<sub>0</sub> e<sup>&omega; t</sup> + c<sub>1</sub> e<sup>-&omega; t</sup> + c<sub>2</sub> t<sup>3</sup> + c<sub>3</sub> t<sup>2</sup> +
 *     c<sub>4</sub> t + c<sub>5</sub></li>
 *     <li> d/dt x(t) = &omega; c<sub>0</sub> e<sup>&omega; t</sup> - &omega; c<sub>1</sub> e<sup>-&omega; t</sup> + 3 c<sub>2</sub> t<sup>2</sup> +
 *     2 c<sub>3</sub> t+ c<sub>4</sub>
 *     <li> d<sup>2</sup> / dt<sup>2</sup> x(t) = &omega;<sup>2</sup> c<sub>0</sub> e<sup>&omega; t</sup> + &omega;<sup>2</sup> c<sub>1</sub> e<sup>-&omega; t</sup>
 *     + 6 c<sub>2</sub> t + 2 c<sub>3</sub>  </li>
 *  </p>
 *
 *
 *    <p> From this, it follows that the VRP has the trajectory
 *    <li> v(t) =  c<sub>2</sub> t<sup>3</sup> + c<sub>3</sub> t<sup>2</sup> + (c<sub>4</sub> - 6/&omega;<sup>2</sup> c<sub>2</sub>) t - 2/&omega; c<sub>3</sub> + c<sub>5</sub></li>
 *    </p>
 */
public class CoMTrajectoryPlanner implements CoMTrajectoryPlannerInterface
{
   private static final int maxCapacity = 10;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean VISUALIZE = true;
   private static final double POINT_SIZE = 0.005;

   private static final double sufficientlyLarge = 1.0e10;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DenseMatrix64F coefficientMultipliers = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F coefficientMultipliersInv = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F xEquivalents = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yEquivalents = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F zEquivalents = new DenseMatrix64F(0, 1);

   private final DenseMatrix64F xConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F zConstants = new DenseMatrix64F(0, 1);

   private final DenseMatrix64F vrpWaypointJacobian = new DenseMatrix64F(0, 1);

   private final DenseMatrix64F vrpXWaypoints = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F vrpYWaypoints = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F vrpZWaypoints = new DenseMatrix64F(0, 1);

   private final DenseMatrix64F xCoefficientVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yCoefficientVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F zCoefficientVector = new DenseMatrix64F(0, 1);

   private final FramePoint3D finalDCMPosition = new FramePoint3D();

   private final DoubleProvider omega;
   private final double gravityZ;
   private double nominalCoMHeight;

   private final CoMTrajectoryPlannerIndexHandler indexHandler = new CoMTrajectoryPlannerIndexHandler();

   private final FixedFramePoint3DBasics desiredCoMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMAcceleration = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredDCMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredDCMVelocity = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredVRPPosition = new FramePoint3D(worldFrame);

   private final RecyclingArrayList<FramePoint3D> startVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> endVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);

   private final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition", worldFrame, registry);
   private final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity", worldFrame, registry);

   private final YoFramePoint3D yoFirstCoefficient = new YoFramePoint3D("comFirstCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoSecondCoefficient = new YoFramePoint3D("comSecondCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoThirdCoefficient = new YoFramePoint3D("comThirdCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoFourthCoefficient = new YoFramePoint3D("comFourthCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoFifthCoefficient = new YoFramePoint3D("comFifthCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoSixthCoefficient = new YoFramePoint3D("comSixthCoefficient", worldFrame, registry);

   private final List<YoFramePoint3D> dcmCornerPoints = new ArrayList<>();
   private final List<YoFramePoint3D> comCornerPoints = new ArrayList<>();

   private int numberOfConstraints = 0;

   public CoMTrajectoryPlanner(DoubleProvider omega, double gravityZ, double nominalCoMHeight, YoVariableRegistry parentRegistry)
   {
      this(omega, gravityZ, nominalCoMHeight, parentRegistry, null);
   }

   public CoMTrajectoryPlanner(DoubleProvider omega, double gravityZ, double nominalCoMHeight, YoVariableRegistry parentRegistry,
                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.omega = omega;
      this.nominalCoMHeight = nominalCoMHeight;
      this.gravityZ = Math.abs(gravityZ);

      for (int i = 0; i < maxCapacity + 1; i++)
      {
         dcmCornerPoints.add(new YoFramePoint3D("dcmCornerPoint" + i, worldFrame, registry));
         comCornerPoints.add(new YoFramePoint3D("comCornerPoint" + i, worldFrame, registry));
      }

      String packageName = "dcmPlanner";
      //      YoGraphicsList graphicsList = new YoGraphicsList(packageName);
      ArtifactList artifactList = new ArtifactList(packageName);

      for (int i = 0; i < dcmCornerPoints.size(); i++)
      {
         YoFramePoint3D dcmCornerPoint = dcmCornerPoints.get(i);
         YoFramePoint3D comCornerPoint = comCornerPoints.get(i);
         YoGraphicPosition dcmCornerPointViz = new YoGraphicPosition("DCMCornerPoint" + i, dcmCornerPoint, POINT_SIZE, YoAppearance.Blue(),
                                                                     YoGraphicPosition.GraphicType.BALL);
         YoGraphicPosition comCornerPointViz = new YoGraphicPosition("CoMCornerPoint" + i, comCornerPoint, POINT_SIZE, YoAppearance.Black(),
                                                                     YoGraphicPosition.GraphicType.BALL);
         //         graphicsList.add(dcmCornerPointViz);
         //         graphicsList.add(comCornerPointViz);

         artifactList.add(dcmCornerPointViz.createArtifact());
         artifactList.add(comCornerPointViz.createArtifact());
      }

      artifactList.setVisible(VISUALIZE);
      //      graphicsList.setVisible(VISUALIZE);

      if (yoGraphicsListRegistry != null)
      {
         //         yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      parentRegistry.addChild(registry);
   }

   /** {@inheritDoc} */
   @Override
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.nominalCoMHeight = nominalCoMHeight;
   }

   /** {@inheritDoc} */
   @Override
   public void solveForTrajectory(List<? extends ContactStateProvider> contactSequence)
   {
      if (!ContactStateProviderTools.checkContactSequenceIsValid(contactSequence))
         throw new IllegalArgumentException("The contact sequence is not valid.");

      indexHandler.update(contactSequence);

      resetMatrices();

      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      computeVRPWaypointsFromContactSequence(contactSequence);

      numberOfConstraints = 0;

      // set initial constraint
      setCoMPositionConstraint(currentCoMPosition);
      setDynamicsInitialConstraint(contactSequence, 0);

      // add transition continuity constraints
      for (int transition = 0; transition < numberOfTransitions; transition++)
      {
         int previousSequence = transition;
         int nextSequence = transition + 1;
         setCoMPositionContinuity(contactSequence, previousSequence, nextSequence);
         setCoMVelocityContinuity(contactSequence, previousSequence, nextSequence);
         setDynamicsFinalConstraint(contactSequence, previousSequence);
         setDynamicsInitialConstraint(contactSequence, nextSequence);
      }

      // set terminal constraint
      ContactStateProvider lastContactPhase = contactSequence.get(numberOfPhases - 1);
      finalDCMPosition.set(lastContactPhase.getCopEndPosition());
      finalDCMPosition.addZ(nominalCoMHeight);
      setDCMPositionConstraint(numberOfPhases - 1, lastContactPhase.getTimeInterval().getDuration(), finalDCMPosition);
      setDynamicsFinalConstraint(contactSequence, numberOfPhases - 1);

      // map from VRP waypoints to the value
      xEquivalents.set(xConstants);
      yEquivalents.set(yConstants);
      zEquivalents.set(zConstants);
      CommonOps.multAdd(vrpWaypointJacobian, vrpXWaypoints, xEquivalents);
      CommonOps.multAdd(vrpWaypointJacobian, vrpYWaypoints, yEquivalents);
      CommonOps.multAdd(vrpWaypointJacobian, vrpZWaypoints, zEquivalents);

      // solve for coefficients
      NativeCommonOps.invert(coefficientMultipliers, coefficientMultipliersInv);
      CommonOps.mult(coefficientMultipliersInv, xEquivalents, xCoefficientVector);
      CommonOps.mult(coefficientMultipliersInv, yEquivalents, yCoefficientVector);
      CommonOps.mult(coefficientMultipliersInv, zEquivalents, zCoefficientVector);

      // update coefficient holders
      int firstCoefficientIndex = 0;
      int secondCoefficientIndex = 1;
      int thirdCoefficientIndex = 2;
      int fourthCoefficientIndex = 3;
      int fifthCoefficientIndex = 4;
      int sixthCoefficientIndex = 5;

      yoFirstCoefficient.setX(xCoefficientVector.get(firstCoefficientIndex));
      yoFirstCoefficient.setY(yCoefficientVector.get(firstCoefficientIndex));
      yoFirstCoefficient.setZ(zCoefficientVector.get(firstCoefficientIndex));

      yoSecondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex));
      yoSecondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex));
      yoSecondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex));

      yoThirdCoefficient.setX(xCoefficientVector.get(thirdCoefficientIndex));
      yoThirdCoefficient.setY(yCoefficientVector.get(thirdCoefficientIndex));
      yoThirdCoefficient.setZ(zCoefficientVector.get(thirdCoefficientIndex));

      yoFourthCoefficient.setX(xCoefficientVector.get(fourthCoefficientIndex));
      yoFourthCoefficient.setY(yCoefficientVector.get(fourthCoefficientIndex));
      yoFourthCoefficient.setZ(zCoefficientVector.get(fourthCoefficientIndex));

      yoFifthCoefficient.setX(xCoefficientVector.get(fifthCoefficientIndex));
      yoFifthCoefficient.setY(yCoefficientVector.get(fifthCoefficientIndex));
      yoFifthCoefficient.setZ(zCoefficientVector.get(fifthCoefficientIndex));

      yoSixthCoefficient.setX(xCoefficientVector.get(sixthCoefficientIndex));
      yoSixthCoefficient.setY(yCoefficientVector.get(sixthCoefficientIndex));
      yoSixthCoefficient.setZ(zCoefficientVector.get(sixthCoefficientIndex));

      updateCornerPoints(numberOfPhases);
   }

   private void computeVRPWaypointsFromContactSequence(List<? extends ContactStateProvider> contactSequence)
   {
      startVRPPositions.clear();
      endVRPPositions.clear();

      double initialHeightVelocity = currentCoMVelocity.getZ();
      double finalHeightVelocity;

      for (int i = 0; i < contactSequence.size(); i++)
      {
         ContactStateProvider contactStateProvider = contactSequence.get(i);
         boolean finalContact = i == contactSequence.size() - 1;
         ContactStateProvider nextContactStateProvider = null;
         if (!finalContact)
            nextContactStateProvider = contactSequence.get(i + 1);

         double duration = contactStateProvider.getTimeInterval().getDuration();
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

         FramePoint3D start = startVRPPositions.add();
         FramePoint3D end = endVRPPositions.add();

         start.set(contactStateProvider.getCopStartPosition());
         start.addZ(nominalCoMHeight);
         end.set(contactStateProvider.getCopEndPosition());
         end.addZ(nominalCoMHeight);

         // offset the height VRP waypoint based on the desired velocity change
         double heightVelocityChange = finalHeightVelocity - initialHeightVelocity;
         double offset = heightVelocityChange / (MathTools.square(omega.getValue()) * duration);
         start.subZ(offset);
         end.subZ(offset);

         initialHeightVelocity = finalHeightVelocity;
      }
   }

   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();
   private final FramePoint3D thirdCoefficient = new FramePoint3D();
   private final FramePoint3D fourthCoefficient = new FramePoint3D();
   private final FramePoint3D fifthCoefficient = new FramePoint3D();
   private final FramePoint3D sixthCoefficient = new FramePoint3D();

   private final FrameVector3D comVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D comAccelerationToThrowAway = new FrameVector3D();
   private final FrameVector3D dcmVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D vrpPositionToThrowAway = new FramePoint3D();

   private void updateCornerPoints(int size)
   {
      int segmentId = 0;
      for (; segmentId < Math.min(size, maxCapacity + 1); segmentId++)
      {
         compute(segmentId, 0.0, comCornerPoints.get(segmentId), comVelocityToThrowAway, comAccelerationToThrowAway, dcmCornerPoints.get(segmentId),
                 dcmVelocityToThrowAway, vrpPositionToThrowAway);
      }

      for (; segmentId < maxCapacity + 1; segmentId++)
      {
         comCornerPoints.get(segmentId).setToNaN();
         dcmCornerPoints.get(segmentId).setToNaN();
      }
   }

   /** {@inheritDoc} */
   @Override
   public void compute(int segmentId, double timeInPhase)
   {
      compute(segmentId, timeInPhase, desiredCoMPosition, desiredCoMVelocity, desiredCoMAcceleration, desiredDCMPosition, desiredDCMVelocity,
              desiredVRPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void compute(int segmentId, double timeInPhase, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack)
   {
      int startIndex = indexHandler.getContactSequenceStartIndex(segmentId);
      firstCoefficient.setX(xCoefficientVector.get(startIndex));
      firstCoefficient.setY(yCoefficientVector.get(startIndex));
      firstCoefficient.setZ(zCoefficientVector.get(startIndex));

      int secondCoefficientIndex = startIndex + 1;
      secondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex));
      secondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex));
      secondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex));

      int thirdCoefficientIndex = startIndex + 2;
      thirdCoefficient.setX(xCoefficientVector.get(thirdCoefficientIndex));
      thirdCoefficient.setY(yCoefficientVector.get(thirdCoefficientIndex));
      thirdCoefficient.setZ(zCoefficientVector.get(thirdCoefficientIndex));

      int fourthCoefficientIndex = startIndex + 3;
      fourthCoefficient.setX(xCoefficientVector.get(fourthCoefficientIndex));
      fourthCoefficient.setY(yCoefficientVector.get(fourthCoefficientIndex));
      fourthCoefficient.setZ(zCoefficientVector.get(fourthCoefficientIndex));

      int fifthCoefficientIndex = startIndex + 4;
      fifthCoefficient.setX(xCoefficientVector.get(fifthCoefficientIndex));
      fifthCoefficient.setY(yCoefficientVector.get(fifthCoefficientIndex));
      fifthCoefficient.setZ(zCoefficientVector.get(fifthCoefficientIndex));

      int sixthCoefficientIndex = startIndex + 5;
      sixthCoefficient.setX(xCoefficientVector.get(sixthCoefficientIndex));
      sixthCoefficient.setY(yCoefficientVector.get(sixthCoefficientIndex));
      sixthCoefficient.setZ(zCoefficientVector.get(sixthCoefficientIndex));

      double omega = this.omega.getValue();

      constructDesiredCoMPosition(comPositionToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                  sixthCoefficient, timeInPhase, omega);
      constructDesiredCoMVelocity(comVelocityToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                  sixthCoefficient, timeInPhase, omega);
      constructDesiredCoMAcceleration(comAccelerationToPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                      sixthCoefficient, timeInPhase, omega);

      computeDesiredCapturePointPosition(comPositionToPack, comVelocityToPack, omega, dcmPositionToPack);
      computeDesiredCapturePointVelocity(comVelocityToPack, comAccelerationToPack, omega, dcmVelocityToPack);
      computeDesiredCentroidalMomentumPivot(dcmPositionToPack, desiredDCMVelocity, omega, vrpPositionToPack);
      //      computeDesiredCentroidalMomentumPivot(dcmPositionToPack, desiredDCMVelocity, omega, desiredVRPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      this.currentCoMPosition.setMatchingFrame(centerOfMassPosition);
      this.currentCoMVelocity.setMatchingFrame(centerOfMassVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }

   /**
    * Resets and resizes the internal matrices.
    */
   private void resetMatrices()
   {
      int size = indexHandler.getTotalSize();
      int numberOfVRPWaypoints = indexHandler.getNumberOfVRPWaypoints();

      coefficientMultipliers.reshape(size, size);
      coefficientMultipliersInv.reshape(size, size);
      xEquivalents.reshape(size, 1);
      yEquivalents.reshape(size, 1);
      zEquivalents.reshape(size, 1);
      xConstants.reshape(size, 1);
      yConstants.reshape(size, 1);
      zConstants.reshape(size, 1);
      vrpWaypointJacobian.reshape(size, numberOfVRPWaypoints); // only position
      vrpXWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpYWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpZWaypoints.reshape(numberOfVRPWaypoints, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      coefficientMultipliers.zero();
      coefficientMultipliersInv.zero();
      xEquivalents.zero();
      yEquivalents.zero();
      zEquivalents.zero();
      xConstants.zero();
      yConstants.zero();
      zConstants.zero();
      vrpWaypointJacobian.zero();
      vrpXWaypoints.zero();
      vrpYWaypoints.zero();
      vrpZWaypoints.zero();
      xCoefficientVector.zero();
      yCoefficientVector.zero();
      zCoefficientVector.zero();
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
   private void setCoMPositionConstraint(FramePoint3DReadOnly centerOfMassLocationForConstraint)
   {
      centerOfMassLocationForConstraint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      double omega = this.omega.getValue();

      coefficientMultipliers.set(numberOfConstraints, 0, getCoMPositionFirstCoefficient(omega, 0.0));
      coefficientMultipliers.set(numberOfConstraints, 1, getCoMPositionSecondCoefficient(omega, 0.0));
      coefficientMultipliers.set(numberOfConstraints, 2, getCoMPositionThirdCoefficient(0.0));
      coefficientMultipliers.set(numberOfConstraints, 3, getCoMPositionFourthCoefficient(0.0));
      coefficientMultipliers.set(numberOfConstraints, 4, getCoMPositionFifthCoefficient(0.0));
      coefficientMultipliers.set(numberOfConstraints, 5, getCoMPositionSixthCoefficient());

      xConstants.add(numberOfConstraints, 0, centerOfMassLocationForConstraint.getX());
      yConstants.add(numberOfConstraints, 0, centerOfMassLocationForConstraint.getY());
      zConstants.add(numberOfConstraints, 0, centerOfMassLocationForConstraint.getZ());

      numberOfConstraints++;
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
   private void setDCMPositionConstraint(int sequenceId, double time, FramePoint3DReadOnly desiredDCMPosition)
   {
      desiredDCMPosition.checkReferenceFrameMatch(worldFrame);

      double omega = this.omega.getValue();

      int startIndex = indexHandler.getContactSequenceStartIndex(sequenceId);

      // add constraints on terminal DCM position
      coefficientMultipliers.set(numberOfConstraints, startIndex + 0, getDCMPositionFirstCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 1, getDCMPositionSecondCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 2, getDCMPositionThirdCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 3, getDCMPositionFourthCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 4, getDCMPositionFifthCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 5, getDCMPositionSixthCoefficient());

      xConstants.add(numberOfConstraints, 0, desiredDCMPosition.getX());
      yConstants.add(numberOfConstraints, 0, desiredDCMPosition.getY());
      zConstants.add(numberOfConstraints, 0, desiredDCMPosition.getZ());

      numberOfConstraints++;
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
    * @param contactSequence current contact sequence.
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void setCoMPositionContinuity(List<? extends ContactStateProvider> contactSequence, int previousSequence, int nextSequence)
   {
      ContactStateProvider previousContact = contactSequence.get(previousSequence);
      double omega = this.omega.getValue();
      double previousDuration = previousContact.getTimeInterval().getDuration();

      // move next sequence coefficients to the left hand side, and previous sequence constants to the right
      int previousStartIndex = indexHandler.getContactSequenceStartIndex(previousSequence);
      int nextStartIndex = indexHandler.getContactSequenceStartIndex(nextSequence);

      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 0, getCoMPositionFirstCoefficient(omega, previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 1, getCoMPositionSecondCoefficient(omega, previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 2, getCoMPositionThirdCoefficient(previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 3, getCoMPositionFourthCoefficient(previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 4, getCoMPositionFifthCoefficient(previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 5, getCoMPositionSixthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 0, -getCoMPositionFirstCoefficient(omega, 0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 1, -getCoMPositionSecondCoefficient(omega, 0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 2, -getCoMPositionThirdCoefficient(0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 3, -getCoMPositionFourthCoefficient(0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 4, -getCoMPositionFifthCoefficient(0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 5, -getCoMPositionSixthCoefficient());

      numberOfConstraints++;
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
    * @param contactSequence current contact sequence.
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void setCoMVelocityContinuity(List<? extends ContactStateProvider> contactSequence, int previousSequence, int nextSequence)
   {
      ContactStateProvider previousContact = contactSequence.get(previousSequence);
      double omega = this.omega.getValue();

      double previousDuration = previousContact.getTimeInterval().getDuration();

      int previousStartIndex = indexHandler.getContactSequenceStartIndex(previousSequence);
      int nextStartIndex = indexHandler.getContactSequenceStartIndex(nextSequence);

      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 0, getCoMVelocityFirstCoefficient(omega, previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 1, getCoMVelocitySecondCoefficient(omega, previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 2, getCoMVelocityThirdCoefficient(previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 3, getCoMVelocityFourthCoefficient(previousDuration));
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 4, getCoMVelocityFifthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, previousStartIndex + 5, getCoMVelocitySixthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 0, -getCoMVelocityFirstCoefficient(omega, 0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 1, -getCoMVelocitySecondCoefficient(omega, 0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 2, -getCoMVelocityThirdCoefficient(0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 3, -getCoMVelocityFourthCoefficient(0.0));
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 4, -getCoMVelocityFifthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, nextStartIndex + 5, -getCoMVelocitySixthCoefficient());

      numberOfConstraints++;
   }

   private final FrameVector3D desiredVelocity = new FrameVector3D();

   /**
    * Used to enforce the dynamics at the beginning of the trajectory segment {@param sequenceId}.
    *
    * @param contactSequence current contact sequence.
    * @param sequenceId desired trajectory segment.
    */
   private void setDynamicsInitialConstraint(List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      ContactState contactState = contactStateProvider.getContactState();
      if (contactState.isLoadBearing())
      {
         desiredVelocity.sub(endVRPPositions.get(sequenceId), startVRPPositions.get(sequenceId));
         desiredVelocity.scale(1.0 / contactStateProvider.getTimeInterval().getDuration());
         constrainVRPPosition(sequenceId, indexHandler.getVRPWaypointStartPositionIndex(sequenceId), 0.0, startVRPPositions.get(sequenceId));
         constrainVRPVelocity(sequenceId, indexHandler.getVRPWaypointStartVelocityIndex(sequenceId), 0.0, desiredVelocity);
      }
      else
      {
         constrainCoMAccelerationToGravity(sequenceId, 0.0);
         constraintCoMJerkToZero(sequenceId, 0.0);
      }
   }

   /**
    * Used to enforce the dynamics at the end of the trajectory segment {@param sequenceId}.
    *
    * @param contactSequence current contact sequence.
    * @param sequenceId desired trajectory segment.
    */
   private void setDynamicsFinalConstraint(List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      ContactState contactState = contactStateProvider.getContactState();
      double duration = contactStateProvider.getTimeInterval().getDuration();
      if (contactState.isLoadBearing())
      {
         desiredVelocity.sub(endVRPPositions.get(sequenceId), startVRPPositions.get(sequenceId));
         desiredVelocity.scale(1.0 / contactStateProvider.getTimeInterval().getDuration());
         constrainVRPPosition(sequenceId, indexHandler.getVRPWaypointFinalPositionIndex(sequenceId), duration, endVRPPositions.get(sequenceId));
         constrainVRPVelocity(sequenceId, indexHandler.getVRPWaypointFinalVelocityIndex(sequenceId), duration, desiredVelocity);
      }
      else
      {
         constrainCoMAccelerationToGravity(sequenceId, duration);
         constraintCoMJerkToZero(sequenceId, duration);
      }
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
   private void constrainVRPPosition(int sequenceId, int vrpWaypointPositionIndex, double time, FramePoint3DReadOnly desiredVRPPosition)
   {
      double omega = this.omega.getValue();

      int startIndex = indexHandler.getContactSequenceStartIndex(sequenceId);

      desiredVRPPosition.checkReferenceFrameMatch(worldFrame);

      coefficientMultipliers.set(numberOfConstraints, startIndex + 0, getVRPPositionFirstCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 1, getVRPPositionSecondCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 2, getVRPPositionThirdCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 3, getVRPPositionFourthCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 4, getVRPPositionFifthCoefficient(time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 5, getVRPPositionSixthCoefficient());

      vrpWaypointJacobian.set(numberOfConstraints, vrpWaypointPositionIndex, 1.0);

      vrpXWaypoints.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getX());
      vrpYWaypoints.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getY());
      vrpZWaypoints.set(vrpWaypointPositionIndex, 0, desiredVRPPosition.getZ());

      numberOfConstraints++;
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
   private void constrainVRPVelocity(int sequenceId, int vrpWaypointVelocityIndex, double time, FrameVector3DReadOnly desiredVRPVelocity)
   {
      double omega = this.omega.getValue();

      int startIndex = indexHandler.getContactSequenceStartIndex(sequenceId);

      desiredVRPVelocity.checkReferenceFrameMatch(worldFrame);

      coefficientMultipliers.set(numberOfConstraints, startIndex + 0, getVRPVelocityFirstCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 1, getVRPVelocitySecondCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 2, getVRPVelocityThirdCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 3, getVRPVelocityFourthCoefficient(time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 4, getVRPVelocityFifthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 5, getVRPVelocitySixthCoefficient());

      vrpWaypointJacobian.set(numberOfConstraints, vrpWaypointVelocityIndex, 1.0);

      vrpXWaypoints.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getX());
      vrpYWaypoints.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getY());
      vrpZWaypoints.set(vrpWaypointVelocityIndex, 0, desiredVRPVelocity.getZ());

      numberOfConstraints++;
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
   private void constrainCoMAccelerationToGravity(int sequenceId, double time)
   {
      double omega = this.omega.getValue();

      int startIndex = indexHandler.getContactSequenceStartIndex(sequenceId);

      coefficientMultipliers.set(numberOfConstraints, startIndex + 0, getCoMAccelerationFirstCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 1, getCoMAccelerationSecondCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 2, getCoMAccelerationThirdCoefficient(time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 3, getCoMAccelerationFourthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 4, getCoMAccelerationFifthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 5, getCoMAccelerationSixthCoefficient());

      zConstants.set(numberOfConstraints, 0, -Math.abs(gravityZ));

      numberOfConstraints++;
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
   private void constraintCoMJerkToZero(int sequenceId, double time)
   {
      double omega = this.omega.getValue();

      int startIndex = indexHandler.getContactSequenceStartIndex(sequenceId);

      coefficientMultipliers.set(numberOfConstraints, startIndex + 0, getCoMJerkFirstCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 1, getCoMJerkSecondCoefficient(omega, time));
      coefficientMultipliers.set(numberOfConstraints, startIndex + 2, getCoMJerkThirdCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 3, getCoMJerkFourthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 4, getCoMJerkFifthCoefficient());
      coefficientMultipliers.set(numberOfConstraints, startIndex + 5, getCoMJerkSixthCoefficient());

      numberOfConstraints++;
   }

   /**
    * e<sup>&omega; t</sup>
    */
   static double getCoMPositionFirstCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * e<sup>-&omega; t</sup>
    */
   static double getCoMPositionSecondCoefficient(double omega, double time)
   {
      return Math.exp(-omega * time);
   }

   /**
    * t<sup>3</sup>
    */
   static double getCoMPositionThirdCoefficient(double time)
   {
      return Math.min(sufficientlyLarge, time * time * time);
   }

   /**
    * t<sup>2</sup>
    */
   static double getCoMPositionFourthCoefficient(double time)
   {
      return Math.min(sufficientlyLarge, time * time);
   }

   /**
    * t
    */
   static double getCoMPositionFifthCoefficient(double time)
   {
      return Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   static double getCoMPositionSixthCoefficient()
   {
      return 1.0;
   }

   /**
    * &omega; e<sup>&omega; t</sup>
    */
   static double getCoMVelocityFirstCoefficient(double omega, double time)
   {
      return omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * -&omega; e<sup>-&omega; t</sup>
    */
   static double getCoMVelocitySecondCoefficient(double omega, double time)
   {
      return -omega * Math.exp(-omega * time);
   }

   /**
    * 3 t<sup>2</sup>
    */
   static double getCoMVelocityThirdCoefficient(double time)
   {
      return 3.0 * Math.min(sufficientlyLarge, time * time);
   }

   /**
    * 2 t
    */
   static double getCoMVelocityFourthCoefficient(double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   static double getCoMVelocityFifthCoefficient()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   static double getCoMVelocitySixthCoefficient()
   {
      return 0.0;
   }

   /**
    * &omega;<sup>2</sup> e<sup>&omega; t</sup>
    */
   static double getCoMAccelerationFirstCoefficient(double omega, double time)
   {
      return omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * &omega;<sup>2</sup> e<sup>-&omega; t</sup>
    */
   static double getCoMAccelerationSecondCoefficient(double omega, double time)
   {
      return omega * omega * Math.exp(-omega * time);
   }

   /**
    * 6 t
    */
   static double getCoMAccelerationThirdCoefficient(double time)
   {
      return 6.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 2
    */
   static double getCoMAccelerationFourthCoefficient()
   {
      return 2.0;
   }

   /**
    * 0.0
    */
   static double getCoMAccelerationFifthCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getCoMAccelerationSixthCoefficient()
   {
      return 0.0;
   }

   /**
    * &omega;<sup>3</sup> e<sup>&omega; t</sup>
    */
   static double getCoMJerkFirstCoefficient(double omega, double time)
   {
      return omega * omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * -&omega;<sup>3</sup> e<sup>-&omega; t</sup>
    */
   static double getCoMJerkSecondCoefficient(double omega, double time)
   {
      return -omega * omega * omega * Math.exp(-omega * time);
   }

   /**
    * 6.0
    */
   static double getCoMJerkThirdCoefficient()
   {
      return 6.0;
   }

   /**
    * 0.0
    */
   static double getCoMJerkFourthCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getCoMJerkFifthCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getCoMJerkSixthCoefficient()
   {
      return 0.0;
   }

   /**
    * 2 e<sup>&omega; t</sup>
    */
   static double getDCMPositionFirstCoefficient(double omega, double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * 0.0
    */
   static double getDCMPositionSecondCoefficient()
   {
      return 0.0;
   }

   /**
    * t<sup>3</sup> + 3.0 / &omega; t<sup>2</sup>
    */
   static double getDCMPositionThirdCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time * time) + 3.0 / omega * Math.min(sufficientlyLarge, time * time);
   }

   /**
    * t<sup>2</sup> + 2.0 / &omega; t
    */
   static double getDCMPositionFourthCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time) + 2.0 / omega * Math.min(sufficientlyLarge, time);
   }

   /**
    * t + 1/ &omega;
    */
   static double getDCMPositionFifthCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time) + 1.0 / omega;
   }

   /**
    * 1.0
    */
   static double getDCMPositionSixthCoefficient()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   static double getVRPPositionFirstCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getVRPPositionSecondCoefficient()
   {
      return 0.0;
   }

   /**
    * t<sup>3</sup> - 6.0 t / &omega;<sup>2</sup>
    */
   static double getVRPPositionThirdCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time * time) - 6.0 * Math.min(sufficientlyLarge, time) / (omega * omega);
   }

   /**
    * t<sup>2</sup> - 2.0 / &omega;<sup>2</sup>
    */
   static double getVRPPositionFourthCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time) - 2.0 / (omega * omega);
   }

   /**
    * t
    */
   static double getVRPPositionFifthCoefficient(double time)
   {
      return Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   static double getVRPPositionSixthCoefficient()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   static double getVRPVelocityFirstCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getVRPVelocitySecondCoefficient()
   {
      return 0.0;
   }

   /**
    * 3 t<sup>2</sup> - 6 / &omega;<sup>2</sup>
    */
   static double getVRPVelocityThirdCoefficient(double omega, double time)
   {
      return 3.0 * Math.min(sufficientlyLarge, time * time) - 6.0 / (omega * omega);
   }

   /**
    * 2 t
    */
   static double getVRPVelocityFourthCoefficient(double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   static double getVRPVelocityFifthCoefficient()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   static double getVRPVelocitySixthCoefficient()
   {
      return 0.0;
   }

   static void constructDesiredCoMPosition(FixedFramePoint3DBasics comPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                           FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                           FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient, FramePoint3DReadOnly sixthCoefficient,
                                           double timeInPhase, double omega)
   {
      comPositionToPack.checkReferenceFrameMatch(worldFrame);
      comPositionToPack.setToZero();
      comPositionToPack.scaleAdd(getCoMPositionFirstCoefficient(omega, timeInPhase), firstCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionSecondCoefficient(omega, timeInPhase), secondCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionThirdCoefficient(timeInPhase), thirdCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionFourthCoefficient(timeInPhase), fourthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionFifthCoefficient(timeInPhase), fifthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionSixthCoefficient(), sixthCoefficient, comPositionToPack);
   }

   static void constructDesiredCoMVelocity(FixedFrameVector3DBasics comVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                           FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                           FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient, FramePoint3DReadOnly sixthCoefficient,
                                           double timeInPhase, double omega)
   {
      comVelocityToPack.checkReferenceFrameMatch(worldFrame);
      comVelocityToPack.setToZero();
      comVelocityToPack.scaleAdd(getCoMVelocityFirstCoefficient(omega, timeInPhase), firstCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocitySecondCoefficient(omega, timeInPhase), secondCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityThirdCoefficient(timeInPhase), thirdCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityFourthCoefficient(timeInPhase), fourthCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityFifthCoefficient(), fifthCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocitySixthCoefficient(), sixthCoefficient, comVelocityToPack);
   }

   static void constructDesiredCoMAcceleration(FixedFrameVector3DBasics comAccelerationToPack, FramePoint3DReadOnly firstCoefficient,
                                               FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                               FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                               FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      comAccelerationToPack.checkReferenceFrameMatch(worldFrame);
      comAccelerationToPack.setToZero();
      comAccelerationToPack.scaleAdd(getCoMAccelerationFirstCoefficient(omega, timeInPhase), firstCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationSecondCoefficient(omega, timeInPhase), secondCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationThirdCoefficient(timeInPhase), thirdCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationFourthCoefficient(), fourthCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationFifthCoefficient(), fifthCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationSixthCoefficient(), sixthCoefficient, comAccelerationToPack);
   }
}
