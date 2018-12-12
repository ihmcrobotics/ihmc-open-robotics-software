package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.*;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.*;

// This guy assumes that the final phase is always the "stopping" phase, where the CoM is supposed to come to rest.
// This means that the final CoP is the terminal ICP location
public class CoMTrajectoryPlanner implements CoMTrajectoryPlannerInterface
{
   private static final int maxCapacity = 10;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean VISUALIZE = true;
   private static final double POINT_SIZE = 0.005;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DenseMatrix64F coefficientMultipliers = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F coefficientMultipliersInv = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F xCoefficientConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yCoefficientConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F zCoefficientConstants = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F xCoefficientVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F yCoefficientVector = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F zCoefficientVector = new DenseMatrix64F(0, 1);

   private final FramePoint3D currentCoMPosition = new FramePoint3D();
   private final FramePoint3D finalDCMPosition = new FramePoint3D();

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final DoubleProvider omega;
   private final double gravityZ;
   private double nominalCoMHeight;

   private final List<? extends ContactStateProvider> contactSequence;

   private final FixedFramePoint3DBasics desiredCoMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMAcceleration = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredDCMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredDCMVelocity = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredVRPPosition = new FramePoint3D(worldFrame);

   private final YoFramePoint3D yoFirstCoefficient = new YoFramePoint3D("comFirstCoefficient", worldFrame, registry);
   private final YoFramePoint3D yoSecondCoefficient = new YoFramePoint3D("comSecondCoefficient", worldFrame, registry);

   private final List<YoFramePoint3D> dcmCornerPoints = new ArrayList<>();
   private final List<YoFramePoint3D> comCornerPoints = new ArrayList<>();
   private final List<YoFramePoint3D> vrpCornerPoints = new ArrayList<>();

   private int numberOfConstraints = 0;

   public CoMTrajectoryPlanner(List<? extends ContactStateProvider> contactSequence, DoubleProvider omega, double gravityZ, double nominalCoMHeight,
                               YoVariableRegistry parentRegistry)
   {
      this(contactSequence, omega, gravityZ, nominalCoMHeight, parentRegistry, null);
   }

   public CoMTrajectoryPlanner(List<? extends ContactStateProvider> contactSequence, DoubleProvider omega, double gravityZ, double nominalCoMHeight,
                               YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.contactSequence = contactSequence;
      this.omega = omega;
      this.nominalCoMHeight = nominalCoMHeight;
      this.gravityZ = Math.abs(gravityZ);

      for (int i = 0; i < maxCapacity + 1; i++)
      {
         dcmCornerPoints.add(new YoFramePoint3D("dcmCornerPoint" + i, worldFrame, registry));
         comCornerPoints.add(new YoFramePoint3D("comCornerPoint" + i, worldFrame, registry));
         vrpCornerPoints.add(new YoFramePoint3D("vrpCornerPoint" + i, worldFrame, registry));
      }

      String packageName = "dcmPlanner";
      YoGraphicsList graphicsList = new YoGraphicsList(packageName);
      ArtifactList artifactList = new ArtifactList(packageName);

      for (int i = 0; i < dcmCornerPoints.size(); i++)
      {
         YoFramePoint3D dcmCornerPoint = dcmCornerPoints.get(i);
         YoFramePoint3D comCornerPoint = comCornerPoints.get(i);
         YoFramePoint3D vrpCornerPoint = vrpCornerPoints.get(i);
         YoGraphicPosition dcmCornerPointViz = new YoGraphicPosition("DCMCornerPoint" + i, dcmCornerPoint, POINT_SIZE, YoAppearance.Blue(),
                                                                     YoGraphicPosition.GraphicType.BALL);
         YoGraphicPosition comCornerPointViz = new YoGraphicPosition("CoMCornerPoint" + i, comCornerPoint, POINT_SIZE, YoAppearance.Black(),
                                                                     YoGraphicPosition.GraphicType.BALL);
         YoGraphicPosition vrpCornerPointViz = new YoGraphicPosition("VRPCornerPoint" + i, vrpCornerPoint, POINT_SIZE, YoAppearance.Green(),
                                                                     YoGraphicPosition.GraphicType.SOLID_BALL);
         graphicsList.add(dcmCornerPointViz);
         graphicsList.add(comCornerPointViz);
         graphicsList.add(vrpCornerPointViz);

         artifactList.add(dcmCornerPointViz.createArtifact());
         artifactList.add(comCornerPointViz.createArtifact());
         artifactList.add(vrpCornerPointViz.createArtifact());
      }

      artifactList.setVisible(VISUALIZE);
      graphicsList.setVisible(VISUALIZE);

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
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
   public void solveForTrajectory()
   {
      if (!ContactStateProviderTools.checkContactSequenceIsValid(contactSequence))
         throw new IllegalArgumentException("The contact sequence is not valid.");

      int size = 2 * contactSequence.size();
      coefficientMultipliers.reshape(size, size);
      coefficientMultipliersInv.reshape(size, size);
      xCoefficientConstants.reshape(size, 1);
      yCoefficientConstants.reshape(size, 1);
      zCoefficientConstants.reshape(size, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      coefficientMultipliers.zero();
      coefficientMultipliersInv.zero();
      xCoefficientConstants.zero();
      yCoefficientConstants.zero();
      zCoefficientConstants.zero();
      xCoefficientVector.zero();
      yCoefficientVector.zero();
      zCoefficientVector.zero();

      int numberOfPhases = contactSequence.size();
      int numberOfTransitions = numberOfPhases - 1;

      numberOfConstraints = 0;

      // set initial constraint
      setCoMPositionConstraint(0, 0.0, currentCoMPosition);

      // add transition continuity constraints
      for (int transition = 0; transition < numberOfTransitions; transition++)
      {
         int previousSequence = transition;
         int nextSequence = transition + 1;
         setPositionContinuity(previousSequence, nextSequence);
         setVelocityContinuity(previousSequence, nextSequence);
      }

      // set terminal constraint
      ContactStateProvider lastContactPhase = contactSequence.get(numberOfPhases - 1);
      finalDCMPosition.set(lastContactPhase.getCopStartPosition());
      finalDCMPosition.addZ(nominalCoMHeight);
      setDCMTerminalConstraint(numberOfPhases - 1, finalDCMPosition);

      // solve for coefficients
      solver.setA(coefficientMultipliers);
      solver.invert(coefficientMultipliersInv);
      CommonOps.mult(coefficientMultipliersInv, xCoefficientConstants, xCoefficientVector);
      CommonOps.mult(coefficientMultipliersInv, yCoefficientConstants, yCoefficientVector);
      CommonOps.mult(coefficientMultipliersInv, zCoefficientConstants, zCoefficientVector);

      int firstCoefficientIndex = getFirstCoefficientIndex(0);
      int secondCoefficientIndex = getSecondCoefficientIndex(0);
      yoFirstCoefficient.setX(xCoefficientVector.get(firstCoefficientIndex));
      yoFirstCoefficient.setY(yCoefficientVector.get(firstCoefficientIndex));
      yoFirstCoefficient.setZ(zCoefficientVector.get(firstCoefficientIndex));

      yoSecondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex));
      yoSecondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex));
      yoSecondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex));

      updateCornerPoints(numberOfPhases);
   }

   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();

   private final FrameVector3D comVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D comAccelerationToThrowAway = new FrameVector3D();
   private final FrameVector3D dcmVelocityToThrowAway = new FrameVector3D();

   private void updateCornerPoints(int size)
   {
      int segmentId = 0;
      for (; segmentId < Math.min(size, maxCapacity + 1); segmentId++)
      {
         compute(segmentId, 0.0, comCornerPoints.get(segmentId), comVelocityToThrowAway, comAccelerationToThrowAway, dcmCornerPoints.get(segmentId), dcmVelocityToThrowAway,
                 vrpCornerPoints.get(segmentId));
      }

      for (; segmentId < maxCapacity + 1; segmentId++)
      {
         comCornerPoints.get(segmentId).setToNaN();
         dcmCornerPoints.get(segmentId).setToNaN();
         vrpCornerPoints.get(segmentId).setToNaN();
      }
   }

   /** {@inheritDoc} */
   @Override
   public void compute(int segmentId, double timeInPhase)
   {
      compute(segmentId, timeInPhase, desiredCoMPosition, desiredCoMVelocity, desiredCoMAcceleration, desiredDCMPosition, desiredDCMVelocity, desiredVRPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void compute(int segmentId, double timeInPhase, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(segmentId);

      int firstCoefficientIndex = getFirstCoefficientIndex(segmentId);
      int secondCoefficientIndex = getSecondCoefficientIndex(segmentId);
      firstCoefficient.setX(xCoefficientVector.get(firstCoefficientIndex));
      firstCoefficient.setY(yCoefficientVector.get(firstCoefficientIndex));
      firstCoefficient.setZ(zCoefficientVector.get(firstCoefficientIndex));

      secondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex));
      secondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex));
      secondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex));

      double omega = this.omega.getValue();

      ContactState contactState = contactStateProvider.getContactState();

      constructDesiredCoMPosition(comPositionToPack, firstCoefficient, secondCoefficient, contactStateProvider.getCopStartPosition(), contactState, timeInPhase,
                                  omega, gravityZ, nominalCoMHeight);
      constructDesiredCoMVelocity(comVelocityToPack, firstCoefficient, secondCoefficient, contactState, timeInPhase, omega, gravityZ);
      constructDesiredCoMAcceleration(comAccelerationToPack, firstCoefficient, secondCoefficient, contactState, timeInPhase, omega, gravityZ);

      computeDesiredCapturePointPosition(comPositionToPack, comVelocityToPack, omega, dcmPositionToPack);
      computeDesiredCapturePointVelocity(comVelocityToPack, comAccelerationToPack, omega, dcmVelocityToPack);
      computeDesiredCentroidalMomentumPivot(dcmPositionToPack, desiredDCMVelocity, omega, vrpPositionToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      this.currentCoMPosition.setIncludingFrame(centerOfMassPosition);
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
    * <p> Sets the continuity constraint on the initial position. This DOES result in a initial discontinuity on the desired DCM location. </p>
    * <p> This constraint should be used for the initial position of the center of mass to properly initialize the trajectory. </p><
    * <p> Recall that the equation for the center of mass is defined by </p>
    * <p>
    *    x<sub>i</sub>(t) = c<sub>0,i</sub> e<sup>&omega; t</sup> + c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub> t + c<sub>3,i</sub>
    * </p>
    * <p>
    *    This results in the following constraint:
    * </p>
    * <p>
    *    c<sub>0,i</sub> e<sup>&omega; t</sup> + c<sub>1,i</sub> e<sup>-&omega; t</sup> + r<sub>vrp,i</sub>= x<sub>0</sub>
    * </p>
    * <p>
    *    c<sub>2,i</sub> = c<sub>3,i</sub> = 0
    * </p>
    * @param sequenceId i in the above equations
    * @param timeInPhaseForConstraint t in the above equations
    * @param centerOfMassLocationForConstraint x<sub>0</sub> in the above equations
    */
   private void setCoMPositionConstraint(int sequenceId, double timeInPhaseForConstraint, FramePoint3DReadOnly centerOfMassLocationForConstraint)
   {
      centerOfMassLocationForConstraint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      double omega = this.omega.getValue();
      ContactState contactState = contactStateProvider.getContactState();

      double c0 = getFirstCoefficientPositionMultiplier(contactState, timeInPhaseForConstraint, omega);
      double c1 = getSecondCoefficientPositionMultiplier(contactState, timeInPhaseForConstraint, omega);

      double bX = centerOfMassLocationForConstraint.getX();
      double bY = centerOfMassLocationForConstraint.getY();
      double bZ = centerOfMassLocationForConstraint.getZ();

      if (contactState == ContactState.IN_CONTACT)
      {
         FramePoint3DReadOnly copPosition = contactStateProvider.getCopStartPosition();
         copPosition.checkReferenceFrameMatch(worldFrame);
         bX -= copPosition.getX();
         bY -= copPosition.getY();
         bZ -= (copPosition.getZ() + nominalCoMHeight);
      }
      bZ -= getGravityPositionEffect(contactState, gravityZ, timeInPhaseForConstraint);

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficientIndex(sequenceId), c0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficientIndex(sequenceId), c1);
      xCoefficientConstants.add(numberOfConstraints, 0, bX);
      yCoefficientConstants.add(numberOfConstraints, 0, bY);
      zCoefficientConstants.add(numberOfConstraints, 0, bZ);

      numberOfConstraints++;
   }

   /**
    * <p> Sets the terminal constraint on the center of mass trajectory. This constraint states that the final position should be equal to
    * {@param terminalPosition} and the desired velocity should be equal to 0. </p>
    * <p> Recall that the equation for the center of mass position is defined by </p>
    * <p>
    *    x<sub>i</sub>(t) = c<sub>0,i</sub> e<sup>&omega; t</sup> + c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub> t + c<sub>3,i</sub> + r<sub>vrp,i</sub>
    * </p>
    * <p> and the center of mass velocity is defined by </p>
    * <p>
    *    v<sub>i</sub>(t) = &omega; c<sub>0,i</sub> e<sup>&omega; t</sup> - &omega; c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub>
    * </p>
    * <p> When combined, this makes the DCM trajectory </p>
    * <p>
    *    &xi;<sub>i</sub>(t) = c<sub>0,i</sub> 2.0 e<sup>&omega; t</sup> + r<sub>vrp,i</sub>
    * </p>
    * <p> The number of variables can be reduced because of this knowledge, making the constraint:</p>
    * <p>
    *    c<sub>0,i</sub> 2.0 e<sup>&omega; T<sub>i</sub></sup> + r<sub>vrp,i</sub> = x<sub>f</sub>
    * </p>
    * <p>
    *    c<sub>2,i</sub> = c<sub>3,i</sub> = 0
    * </p>
    * @param sequenceId i in the above equations
    * @param terminalDCMPosition desired final location. x<sub>f</sub> in the above equations.
    */
   private void setDCMTerminalConstraint(int sequenceId, FramePoint3DReadOnly terminalDCMPosition)
   {
      terminalDCMPosition.checkReferenceFrameMatch(worldFrame);
      double omega = this.omega.getValue();

      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      double duration = contactStateProvider.getTimeInterval().getDuration();
      FramePoint3DReadOnly copPosition = contactStateProvider.getCopStartPosition();
      copPosition.checkReferenceFrameMatch(worldFrame);

      double c0 = 2.0 * Math.exp(omega * duration);
      double bX = terminalDCMPosition.getX() - copPosition.getX();
      double bY = terminalDCMPosition.getY() - copPosition.getY();
      double bZ = terminalDCMPosition.getZ() - (copPosition.getZ() + nominalCoMHeight);

      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficientIndex(sequenceId), c0);
      xCoefficientConstants.add(numberOfConstraints, 0, bX);
      yCoefficientConstants.add(numberOfConstraints, 0, bY);
      zCoefficientConstants.add(numberOfConstraints, 0, bZ);

      numberOfConstraints++;
   }

   /**
    * Sets up the continuity constraint on the CoM position at a state change.
    * <p>
    *    The CoM position equation is as follows:
    * </p>
    * <p>
    *    x<sub>i</sub>(t) = c<sub>0,i</sub> e<sup>&omega; t</sup> + c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub> t + c<sub>3,i</sub> + r<sub>vrp,i</sub>
    * </p>
    * <p> If both the previous state and the next state are in contact, the constraint used is </p>
    * <p>
    *    c<sub>0,i-1</sub> e<sup>&omega; T<sub>i-1</sub></sup> + c<sub>1,i-1</sub> e<sup>-&omega; T<sub>i-1</sub></sup> + r<sub>vrp,i-1</sub>
    *    = c<sub>0,i</sub> e<sup>&omega; 0</sup> + c<sub>1,i</sub> e<sup>-&omega; 0</sup> + r<sub>vrp,i</sub>
    * </p>
    * <p>
    *    c<sub>2,i-1</sub> = c<sub>3,i-1</sub> = c<sub>2,i</sub> = c<sub>3,i</sub> = 0
    * </p>
    * <p> If the previous state is in contact and the next state is in flight, the constraint used is </p>
    * <p>
    *    c<sub>0,i-1</sub> e<sup>&omega; T<sub>i-1</sub></sup> + c<sub>1,i-1</sub> e<sup>-&omega; T<sub>i-1</sub></sup> + r<sub>vrp,i-1</sub>
    *    = c<sub>2,i</sub> 0 + c<sub>3,i</sub>
    * </p>
    * <p>
    *    c<sub>2,i-1</sub> = c<sub>3,i-1</sub> = c<sub>0,i</sub> = c<sub>1,i</sub> = 0
    * </p>
    * <p> If the previous state is in flight and the next state is in contact, the constraint used is </p>
    * <p>
    *    c<sub>2,i-1</sub> 0 + c<sub>3,i-1</sub>
    *       = c<sub>0,i</sub> e<sup>&omega; 0</sup> + c<sub>1,i</sub> e<sup>-&omega; 0</sup> + r<sub>vrp,i</sub>
    * </p>
    * <p>
    *    c<sub>0,i-1</sub> = c<sub>1,i-1</sub> = c<sub>2,i</sub> = c<sub>3,i</sub> = 0
    * </p>
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void setPositionContinuity(int previousSequence, int nextSequence)
   {
      ContactStateProvider previousContact = contactSequence.get(previousSequence);
      ContactStateProvider nextContact = contactSequence.get(nextSequence);
      double omega = this.omega.getValue();

      double previousDuration = previousContact.getTimeInterval().getDuration();
      double previousBX, previousBY, previousBZ;
      if (previousContact.getContactState() == ContactState.IN_CONTACT)
      {
         FramePoint3DReadOnly previousCopPosition = previousContact.getCopStartPosition();
         previousCopPosition.checkReferenceFrameMatch(worldFrame);
         previousBX = previousCopPosition.getX();
         previousBY = previousCopPosition.getY();
         previousBZ = previousCopPosition.getZ() + nominalCoMHeight;
      }
      else
      {
         previousBX = 0.0;
         previousBY = 0.0;
         previousBZ = 0.0;
      }

      ContactState previousContactState = previousContact.getContactState();
      double previousC0 = getFirstCoefficientPositionMultiplier(previousContactState, previousDuration, omega);
      double previousC1 = getSecondCoefficientPositionMultiplier(previousContactState, previousDuration, omega);
      double previousGravityEffect = getGravityPositionEffect(previousContactState, previousDuration, gravityZ);

      double nextBX, nextBY, nextBZ;
      if (nextContact.getContactState() == ContactState.IN_CONTACT)
      {
         FramePoint3DReadOnly nextCopPosition = nextContact.getCopStartPosition();
         nextCopPosition.checkReferenceFrameMatch(worldFrame);
         nextBX = nextCopPosition.getX();
         nextBY = nextCopPosition.getY();
         nextBZ = nextCopPosition.getZ() + nominalCoMHeight;
      }
      else
      {
         nextBX = 0.0;
         nextBY = 0.0;
         nextBZ = 0.0;
      }

      ContactState nextContactState = nextContact.getContactState();
      double nextC0 = getFirstCoefficientPositionMultiplier(nextContactState, 0.0, omega);
      double nextC1 = getSecondCoefficientPositionMultiplier(nextContactState, 0.0, omega);
      double nextGravityEffort = getGravityPositionEffect(nextContactState, 0.0, gravityZ);

      // move next sequence coefficients to the left hand side, and previous sequence constants to the right
      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficientIndex(previousSequence), previousC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficientIndex(previousSequence), previousC1);
      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficientIndex(nextSequence), -nextC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficientIndex(nextSequence), -nextC1);
      xCoefficientConstants.add(numberOfConstraints, 0, nextBX - previousBX);
      yCoefficientConstants.add(numberOfConstraints, 0, nextBY - previousBY);
      zCoefficientConstants.add(numberOfConstraints, 0, nextBZ + nextGravityEffort - previousBZ - previousGravityEffect);

      numberOfConstraints++;
   }

   /**
    * Sets up the continuity constraint on the CoM velocity at a state change.
    * <p>
    *    The CoM velocity equation is as follows:
    * </p>
    * <p>
    *    v<sub>i</sub>(t) = &omega; c<sub>0,i</sub> e<sup>&omega; t</sup> - &omega; c<sub>1,i</sub> e<sup>-&omega; t</sup> + c<sub>2,i</sub>
    * </p>
    * <p> If both the previous state and the next state are in contact, the constraint used is </p>
    * <p>
    *    &omega; c<sub>0,i-1</sub> e<sup>&omega; T<sub>i-1</sub></sup> - &omega; c<sub>1,i-1</sub> e<sup>-&omega; T<sub>i-1</sub></sup>
    *    = &omega; c<sub>0,i</sub> e<sup>&omega; 0</sup> - &omega; c<sub>1,i</sub> e<sup>-&omega; 0</sup>
    * </p>
    * <p> If the previous state is in contact and the next state is in flight, the constraint used is </p>
    * <p>
    *    &omega; c<sub>0,i-1</sub> e<sup>&omega; T<sub>i-1</sub></sup> - &omega; c<sub>1,i-1</sub> e<sup>-&omega; T<sub>i-1</sub></sup>
    *    = c<sub>2,i</sub>
    * </p>
    * <p> If the previous state is in flight and the next state is in contact, the constraint used is </p>
    * <p>
    *    c<sub>2,i-1</sub>
    *       = &omega; c<sub>0,i</sub> e<sup>&omega; 0</sup> - &omega; c<sub>1,i</sub> e<sup>-&omega; 0</sup>
    * </p>
    * @param previousSequence i-1 in the above equations.
    * @param nextSequence i in the above equations.
    */
   private void setVelocityContinuity(int previousSequence, int nextSequence)
   {
      ContactStateProvider previousContact = contactSequence.get(previousSequence);
      ContactStateProvider nextContact = contactSequence.get(nextSequence);
      double omega = this.omega.getValue();

      double previousDuration = previousContact.getTimeInterval().getDuration();

      ContactState previousContactState = previousContact.getContactState();
      double previousC0 = getFirstCoefficientVelocityMultiplier(previousContactState, previousDuration, omega);
      double previousC1 = getSecondCoefficientVelocityMultiplier(previousContactState, previousDuration, omega);
      double previousGravityEffect = getGravityVelocityEffect(previousContactState, previousDuration, gravityZ);

      ContactState nextContactState = nextContact.getContactState();
      double nextC0 = getFirstCoefficientVelocityMultiplier(nextContactState, 0.0, omega);
      double nextC1 = getSecondCoefficientVelocityMultiplier(nextContactState, 0.0, omega);
      double nextGravityEffect = getGravityVelocityEffect(nextContactState, 0.0, gravityZ);

      // move next sequence coefficients to the left hand side, and previous sequence constants to the right
      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficientIndex(previousSequence), previousC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficientIndex(previousSequence), previousC1);
      coefficientMultipliers.set(numberOfConstraints, getFirstCoefficientIndex(nextSequence), -nextC0);
      coefficientMultipliers.set(numberOfConstraints, getSecondCoefficientIndex(nextSequence), -nextC1);
      xCoefficientConstants.add(numberOfConstraints, 0, 0.0);
      yCoefficientConstants.add(numberOfConstraints, 0, 0.0);
      zCoefficientConstants.add(numberOfConstraints, 0, nextGravityEffect - previousGravityEffect);

      numberOfConstraints++;
   }
}
