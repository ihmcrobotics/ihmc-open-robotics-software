package us.ihmc.quadrupedRobotics.planning.icp;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
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
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.*;
import static us.ihmc.quadrupedRobotics.planning.icp.CoMTrajectoryPlannerTools.*;

// This guy assumes that the final phase is always the "stopping" phase, where the CoM is supposed to come to rest.
// This means that the final CoP is the terminal ICP location
public class CoMTrajectoryPlanner
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
   private final FixedFramePoint3DBasics desiredECMPPosition = new FramePoint3D(worldFrame);

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

   /**
    * Sets the nominal CoM height to be used by the planner.
    */
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.nominalCoMHeight = nominalCoMHeight;
   }

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
      finalDCMPosition.set(lastContactPhase.getCopPosition());
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

   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();
   private final FrameVector3D dcmVelocity = new FrameVector3D();

   private void updateCornerPoints(int size)
   {
      double omega = this.omega.getValue();

      int i = 0;
      for (; i < Math.min(size, maxCapacity + 1); i++)
      {
         ContactState contactState = contactSequence.get(i).getContactState();

         int firstCoefficientIndex = getFirstCoefficientIndex(i);
         int secondCoefficientIndex = getSecondCoefficientIndex(i);
         firstCoefficient.setX(xCoefficientVector.get(firstCoefficientIndex));
         firstCoefficient.setY(yCoefficientVector.get(firstCoefficientIndex));
         firstCoefficient.setZ(zCoefficientVector.get(firstCoefficientIndex));

         secondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex));
         secondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex));
         secondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex));

         constructDesiredCoMPosition(comCornerPoints.get(i), firstCoefficient, secondCoefficient, contactSequence.get(i).getCopPosition(), contactState, 0.0,
                                     omega, gravityZ, nominalCoMHeight);
         constructDesiredCoMVelocity(comVelocity, firstCoefficient, secondCoefficient, contactState, 0.0, omega, gravityZ);
         constructDesiredCoMAcceleration(comAcceleration, firstCoefficient, secondCoefficient, contactState, 0.0, omega, gravityZ);

         computeDesiredCapturePointPosition(comCornerPoints.get(i), comVelocity, omega, dcmCornerPoints.get(i));
         computeDesiredCapturePointVelocity(comVelocity, comAcceleration, omega, dcmVelocity);
         computeDesiredCentroidalMomentumPivot(dcmCornerPoints.get(i), dcmVelocity, omega, vrpCornerPoints.get(i));
      }

      for (; i < maxCapacity + 1; i++)
      {
         comCornerPoints.get(i).setToNaN();
         dcmCornerPoints.get(i).setToNaN();
         vrpCornerPoints.get(i).setToNaN();
      }
   }

   /**
    * Computes the desired values.
    * @param timeInPhase time in the current phase. Note that this assumes that the phase starts at 0.0.
    */
   public void compute(double timeInPhase)
   {
      ContactStateProvider currentContactPhase = contactSequence.get(0);

      double omega = this.omega.getValue();

      ContactState contactState = currentContactPhase.getContactState();

      constructDesiredCoMPosition(desiredCoMPosition, yoFirstCoefficient, yoSecondCoefficient, currentContactPhase.getCopPosition(), contactState, timeInPhase,
                                  omega, gravityZ, nominalCoMHeight);
      constructDesiredCoMVelocity(desiredCoMVelocity, yoFirstCoefficient, yoSecondCoefficient, contactState, timeInPhase, omega, gravityZ);
      constructDesiredCoMAcceleration(desiredCoMAcceleration, yoFirstCoefficient, yoSecondCoefficient, contactState, timeInPhase, omega, gravityZ);

      computeDesiredCapturePointPosition(desiredCoMPosition, desiredCoMVelocity, omega, desiredDCMPosition);
      computeDesiredCapturePointVelocity(desiredCoMVelocity, desiredCoMAcceleration, omega, desiredDCMVelocity);
      computeDesiredCentroidalMomentumPivot(desiredDCMPosition, desiredDCMVelocity, omega, desiredVRPPosition);

      desiredECMPPosition.set(desiredVRPPosition);
      desiredECMPPosition.subZ(gravityZ / MathTools.square(omega));
   }

   public void setCurrentCoMPosition(FramePoint3DReadOnly currentCoMPosition)
   {
      this.currentCoMPosition.setIncludingFrame(currentCoMPosition);
   }

   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }

   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return desiredECMPPosition;
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
      double omega = this.omega.getValue();

      double c0, c1;
      if (timeInPhaseForConstraint > 0)
      {
         c0 = Math.exp(omega * timeInPhaseForConstraint);
         c1 = Math.exp(-omega * timeInPhaseForConstraint);
      }
      else
      {
         c0 = 1.0;
         c1 = 1.0;
      }

      double bX = centerOfMassLocationForConstraint.getX();
      double bY = centerOfMassLocationForConstraint.getY();
      double bZ = centerOfMassLocationForConstraint.getZ();

      if (contactSequence.get(sequenceId).getContactState() == ContactState.IN_CONTACT)
      {
         bX -= contactSequence.get(sequenceId).getCopPosition().getX();
         bY -= contactSequence.get(sequenceId).getCopPosition().getY();
         bZ -= (contactSequence.get(sequenceId).getCopPosition().getZ() + nominalCoMHeight);
      }

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
      terminalDCMPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      double omega = this.omega.getValue();

      double duration = contactSequence.get(sequenceId).getTimeInterval().getDuration();

      double c0 = 2.0 * Math.exp(omega * duration);
      double bX = terminalDCMPosition.getX() - contactSequence.get(sequenceId).getCopPosition().getX();
      double bY = terminalDCMPosition.getY() - contactSequence.get(sequenceId).getCopPosition().getY();
      double bZ = terminalDCMPosition.getZ() - (contactSequence.get(sequenceId).getCopPosition().getZ() + nominalCoMHeight);

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
         FramePoint3DReadOnly previousCopPosition = previousContact.getCopPosition();
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
         FramePoint3DReadOnly nextCopPosition = nextContact.getCopPosition();
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
