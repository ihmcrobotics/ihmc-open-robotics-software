package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

import us.fed.fs.fpl.optimization.Lmdif_fcn;
import us.fed.fs.fpl.optimization.Minpack_f77;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFeetUpdater;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.GoOnToesDuringDoubleSupportBipedFeetUpdater;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ResizableBipedFoot;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturePointCalculatorInterface;
import us.ihmc.commonWalkingControlModules.captureRegion.CommonCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfMassHeightControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.GeometricVirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.TeeterTotterLegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.pelvisOrientation.AxisAnglePelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SimpleDesiredCenterOfPressureFilter;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SpeedControllingDesiredCoPCalculator;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.utilities.screwTheory.DesiredJointAccelerationCalculator;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.mathworks.jama.Matrix;
import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.AxisAngleOrientationController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public class MomentumBasedController implements RobotController
{
   private static final long serialVersionUID = -744968203951905486L;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ProcessedSensorsInterface processedSensors;
   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final ProcessedOutputsInterface processedOutputs;
   private final TwistCalculator twistCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final SideDependentList<GeometricJacobian> legJacobians = new SideDependentList<GeometricJacobian>();
   private final SideDependentList<DesiredJointAccelerationCalculator> desiredJointAccelerationCalculators =
      new SideDependentList<DesiredJointAccelerationCalculator>();
   private final FramePoint initialPelvisPosition = new FramePoint(worldFrame);
   private final AxisAnglePelvisOrientationControlModule orientationControlModule;
   private final Matrix jointVelocitiesMatrix;
   private final double controlDT;
   private final InverseDynamicsJoint[] jointsInOrder;
   private final InverseDynamicsJoint[] upperBodyJointsInOrder;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final CapturePointCalculatorInterface capturePointCalculator;
   private final VirtualToePointCalculator virtualToePointCalculator;
   private final SpeedControllingDesiredCoPCalculator desiredCapturePointToDesiredCoPControlModule;
   private final SimpleDesiredCenterOfPressureFilter desiredCenterOfPressureFilter;
   private final CenterOfMassHeightControlModule centerOfMassHeightControlModule;
   private final LegStrengthCalculator legStrengthCalculator;

   private final ResizableBipedFoot rightFoot;
   private final ResizableBipedFoot leftFoot;
   private final BipedFeetUpdater bipedFeetUpdater;
   private final ReferenceFrame midFeetZUp;
   private final double totalMass;
   private final int nDegreesOfFreedom;

   // optimization stuff:
   private final Lmdif_fcn nlls;
   private final int m;
   private final int n;
   private final double[] x;
   private final double[] fvec;
   private final double tol = 1e-9;
   private final int[] info = new int[2];
   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final Matrix previousCentroidalMomentumMatrix;
   private final Matrix centroidalMomentumMatrixDerivative;
   private final YoFrameVector desiredLinearCentroidalMomentumRate;
   private final YoFrameVector desiredAngularCentroidalMomentumRate;
   private final YoFrameVector linearCentroidalMomentumRateError;
   private final YoFrameVector angularCentroidalMomentumRateError;
   private final YoFrameVector desiredPelvisLinearAcceleration;
   private final YoFrameVector desiredPelvisAngularAcceleration;
   private final YoFrameVector desiredPelvisForce;
   private final YoFrameVector desiredPelvisTorque;
   private final ReferenceFrame centerOfMassFrame;

   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);

   private final DoubleYoVariable kAngularMomentumXY = new DoubleYoVariable("kAngularMomentumXY", registry);
   private final DoubleYoVariable kPelvisAxisAngle = new DoubleYoVariable("kPelvisAxisAngle", registry);
   private final DoubleYoVariable kUpperBody = new DoubleYoVariable("kUpperBody", registry);
   private final DoubleYoVariable zetaUpperBody = new DoubleYoVariable("zetaUpperBody", registry);
   private final DoubleYoVariable kAngularMomentumZ = new DoubleYoVariable("kAngularMomentumZ", registry);
   private final DoubleYoVariable alphaDesiredJointAcceleration = new DoubleYoVariable("alphaDesiredJointAcceleration", registry);

   private final YoFrameVector2d desiredDeltaCMP = new YoFrameVector2d("desiredDeltaCMP", "", worldFrame, registry);
   private final SideDependentList<HashMap<FramePoint, BooleanYoVariable>> contactMap = SideDependentList.createListOfHashMaps();
   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   private final SideDependentList<AxisAngleOrientationController> footOrientationControllers = new SideDependentList<AxisAngleOrientationController>();
   private final YoFrameVector swingFootPositionErrorInWorld = new YoFrameVector("swingFootPositionErrorInWorld", "", worldFrame, registry);

   public MomentumBasedController(ProcessedSensorsInterface processedSensors, ProcessedOutputsInterface processedOutputs, CommonWalkingReferenceFrames referenceFrames, double controlDT,
                                      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, double footForward, double footBack, double footWidth, double footHeight)
   {
      this.processedSensors = processedSensors;
      this.fullRobotModel = processedSensors.getFullRobotModel();
      this.referenceFrames = referenceFrames;
      this.processedOutputs = processedOutputs;
      this.controlDT = controlDT;

      RigidBody elevator = fullRobotModel.getElevator();
      this.twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, -processedSensors.getGravityInWorldFrame().getZ());

      RigidBody pelvis = fullRobotModel.getPelvis();
      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         GeometricJacobian jacobian = new GeometricJacobian(foot, pelvis, fullRobotModel.getRootJoint().getFrameAfterJoint());
         legJacobians.put(robotSide, jacobian);
         desiredJointAccelerationCalculators.put(robotSide, new DesiredJointAccelerationCalculator(foot, pelvis, jacobian));
         AxisAngleOrientationController footOrientationController = new AxisAngleOrientationController(robotSide.getCamelCaseNameForStartOfExpression()
                                                                       + "Foot", referenceFrames.getAnkleZUpFrame(robotSide), registry);
         footOrientationController.setProportionalGains(0.0, 0.0, 0.0);
         footOrientationController.setDerivativeGains(0.0, 0.0, 0.0);
         footOrientationControllers.put(robotSide, footOrientationController);
      }

      this.orientationControlModule = new AxisAnglePelvisOrientationControlModule(processedSensors, referenceFrames, null, registry, false);
      this.jointsInOrder = ScrewTools.computeJointsInOrder(elevator);
      this.nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);

      ArrayList<InverseDynamicsJoint> upperBodyJointsArrayList = new ArrayList<InverseDynamicsJoint>();
      for (RobotSide robotSide : RobotSide.values())        
         for (InverseDynamicsJoint joint : fullRobotModel.getArmJointList(robotSide))
            upperBodyJointsArrayList.add(joint);
      for (InverseDynamicsJoint joint : fullRobotModel.getSpineJointList())
         upperBodyJointsArrayList.add(joint);
      for (InverseDynamicsJoint joint : fullRobotModel.getNeckJointList())
         upperBodyJointsArrayList.add(joint);

      this.upperBodyJointsInOrder = new InverseDynamicsJoint[upperBodyJointsArrayList.size()];
      upperBodyJointsArrayList.toArray(upperBodyJointsInOrder);

      this.jointVelocitiesMatrix = new Matrix(nDegreesOfFreedom, 1);

      SideDependentList<ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      this.bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUp, registry, dynamicGraphicObjectsListRegistry);

      double preventRotationFactorLength = 0.9;    // 0.8;    // 0.9; // 1.0;
      double preventRotationFactorWidth = 0.8;    // 0.7;    // 0.9;


      rightFoot = ResizableBipedFoot.createRectangularRightFoot(preventRotationFactorLength, preventRotationFactorWidth, footForward, footBack, footWidth,
              footHeight, referenceFrames, processedSensors.getYoTime(), registry, dynamicGraphicObjectsListRegistry);
      leftFoot = rightFoot.createLeftFootAsMirrorImage(referenceFrames, processedSensors.getYoTime(), registry, dynamicGraphicObjectsListRegistry);
      bipedFeetUpdater = new GoOnToesDuringDoubleSupportBipedFeetUpdater(referenceFrames, footForward, footBack, registry, dynamicGraphicObjectsListRegistry);

      this.capturePointCalculator = new CommonCapturePointCalculator(processedSensors, referenceFrames, registry, dynamicGraphicObjectsListRegistry);
      this.virtualToePointCalculator = new GeometricVirtualToePointCalculator(referenceFrames, registry, dynamicGraphicObjectsListRegistry);

      this.desiredCapturePointToDesiredCoPControlModule = new SpeedControllingDesiredCoPCalculator(processedSensors, referenceFrames, registry,
              dynamicGraphicObjectsListRegistry);
      desiredCapturePointToDesiredCoPControlModule.setParametersForR2InverseDynamics();
      desiredCenterOfPressureFilter = new SimpleDesiredCenterOfPressureFilter(bipedSupportPolygons, referenceFrames, controlDT, registry);
      desiredCenterOfPressureFilter.setParametersForR2();
      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(registry);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      this.centerOfMassHeightControlModule = new CenterOfMassHeightControlModule(processedSensors, referenceFrames, footHeight, registry);
      centerOfMassHeightControlModule.setParametersForR2InverseDynamics();
      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      orientationControlModule.setupParametersForR2();

      nlls = new InverseDynamicsOptimizationFunction(nDegreesOfFreedom);
      m = Momentum.SIZE;    // nDegreesOfFreedom + Momentum.SIZE;
      n = 6;    // nDegreesOfFreedomUpperBody;
      x = new double[n + 1];
      fvec = new double[m + 1];

      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, referenceFrames.getCenterOfMassFrame());
      this.previousCentroidalMomentumMatrix = new Matrix(centroidalMomentumMatrix.getMatrix().getRowDimension(),
              centroidalMomentumMatrix.getMatrix().getColumnDimension());
      this.centroidalMomentumMatrixDerivative = new Matrix(centroidalMomentumMatrix.getMatrix().getRowDimension(),
              centroidalMomentumMatrix.getMatrix().getColumnDimension());

      this.desiredLinearCentroidalMomentumRate = new YoFrameVector("desiredLinearCentroidalMomentumRate", "", centerOfMassFrame, registry);
      this.desiredAngularCentroidalMomentumRate = new YoFrameVector("desiredAngularCentroidalMomentumRate", "", centerOfMassFrame, registry);
      this.linearCentroidalMomentumRateError = new YoFrameVector("linearCentroidalMomentumRateError", "", centerOfMassFrame, registry);
      this.angularCentroidalMomentumRateError = new YoFrameVector("angularCentroidalMomentumRateError", "", centerOfMassFrame, registry);
      this.desiredPelvisLinearAcceleration = new YoFrameVector("desiredPelvisLinearAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisForce = new YoFrameVector("desiredPelvisForce", "", centerOfMassFrame, registry);
      this.desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", "", centerOfMassFrame, registry);

      kAngularMomentumXY.set(3e-2);
      kPelvisAxisAngle.set(5e-1);
      kAngularMomentumZ.set(10.0);
      kUpperBody.set(100.0);
      zetaUpperBody.set(1.0);
      supportLeg.set(null);

      desiredCoMHeight.set(1.25);
      alphaDesiredJointAcceleration.set(0.05);
   }

   public void initialize()
   {
      initialPelvisPosition.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      initialPelvisPosition.changeFrame(worldFrame);

      centroidalMomentumMatrix.compute();
      MatrixTools.set(previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix());

      doControl();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      updateStuff();
      doMomentumBasedControl();
      inverseDynamicsCalculator.compute();
      fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }

   private void updateStuff()
   {
      bipedFeetUpdater.updateBipedFeet(leftFoot, rightFoot, supportLeg.getEnumValue(), capturePointCalculator.getCapturePointInFrame(midFeetZUp), false);
      bipedSupportPolygons.update(leftFoot, rightFoot);
      centroidalMomentumMatrix.compute();
      MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
              controlDT);
      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, jointVelocitiesMatrix);

      for (RobotSide robotSide : RobotSide.values())
      {
         legJacobians.get(robotSide).compute();
      }
      twistCalculator.compute();
   }

   private void doMomentumBasedControl()
   {
      FramePoint2d desiredCoP = determineDesiredCoP();
      FrameVector desiredDeltaCMP = determineDesiredDeltaCMP();
      FramePoint2d desiredCMP = new FramePoint2d(desiredCoP);
      FrameVector2d desiredDeltaCMP2d = desiredDeltaCMP.toFrameVector2d();
      desiredCMP.add(desiredDeltaCMP2d);

      desiredDeltaCMP2d.changeFrame(this.desiredDeltaCMP.getReferenceFrame());
      this.desiredDeltaCMP.set(desiredDeltaCMP2d);

      SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();
      SideDependentList<Double> legStrengths = new SideDependentList<Double>();
      RobotSide supportLeg = this.supportLeg.getEnumValue();
      if (supportLeg == null)
      {
         virtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, desiredCoP);
         legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, desiredCoP);
      }
      else
      {
         virtualToePoints.put(supportLeg, desiredCoP);
         legStrengths.put(supportLeg, 1.0);

         virtualToePoints.put(supportLeg.getOppositeSide(), new FramePoint2d(midFeetZUp));
         legStrengths.put(supportLeg.getOppositeSide(), 0.0);
      }

      double fZ = centerOfMassHeightControlModule.doCenterOfMassHeightControl(desiredCoMHeight.getDoubleValue(), null);
      FrameVector totalgroundReactionMoment = determineGroundReactionMoment();
      Wrench totalGroundReactionWrench = new Wrench(centerOfMassFrame, centerOfMassFrame);

      for (RobotSide robotSide : RobotSide.values())
      {
         FramePoint groundReactionForceTerminalPoint = new FramePoint(centerOfMassFrame);
         groundReactionForceTerminalPoint.changeFrame(midFeetZUp);
         desiredDeltaCMP.changeFrame(midFeetZUp);
         groundReactionForceTerminalPoint.sub(desiredDeltaCMP);
         FrameVector groundReactionForce = new FrameVector(groundReactionForceTerminalPoint);
         FramePoint virtualToePoint = virtualToePoints.get(robotSide).toFramePoint();
         groundReactionForce.sub(virtualToePoint);
         groundReactionForce.scale(legStrengths.get(robotSide) * fZ / groundReactionForce.getZ());

         FrameVector groundReactionMoment = new FrameVector(totalgroundReactionMoment);
         groundReactionMoment.scale(legStrengths.get(robotSide));

         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame footCoMFrame = foot.getBodyFixedFrame();
         FrameVector torque = new FrameVector(midFeetZUp);
         torque.cross(virtualToePoint, groundReactionForce);
         Wrench groundReactionWrench = new Wrench(footCoMFrame, midFeetZUp, groundReactionForce.getVector(), torque.getVector());
         groundReactionWrench.addAngularPart(groundReactionMoment.getVector());
         groundReactionWrench.changeFrame(footCoMFrame);
         inverseDynamicsCalculator.setExternalWrench(foot, groundReactionWrench);
         groundReactionWrench.changeFrame(centerOfMassFrame);
         groundReactionWrench.changeBodyFrameAttachedToSameBody(centerOfMassFrame);
         totalGroundReactionWrench.add(groundReactionWrench);
      }

      FrameVector linearCentroidalMomentumRate = new FrameVector(totalGroundReactionWrench.getExpressedInFrame(),
                                                    totalGroundReactionWrench.getLinearPartCopy());
      linearCentroidalMomentumRate.setZ(linearCentroidalMomentumRate.getZ() + processedSensors.getGravityInWorldFrame().getZ() * totalMass);

      desiredLinearCentroidalMomentumRate.set(linearCentroidalMomentumRate);
      desiredAngularCentroidalMomentumRate.set(totalGroundReactionWrench.getAngularPartCopy());

      double kUpperBody = this.kUpperBody.getDoubleValue();
      double dUpperBody = 2.0 * zetaUpperBody.getDoubleValue() * Math.sqrt(kUpperBody);
      for (RobotSide robotSide : RobotSide.values())
         doPDControl(kUpperBody, dUpperBody, fullRobotModel.getArmJointList(robotSide));
      doPDControl(kUpperBody, dUpperBody, fullRobotModel.getSpineJointList());
      doPDControl(kUpperBody, dUpperBody, fullRobotModel.getNeckJointList());

      solveForPelvisJointAcceleration();
   }

   private FramePoint2d determineDesiredCoP()
   {
      capturePointCalculator.computeCapturePoint(null);

      FramePoint2d capturePoint = capturePointCalculator.getCapturePoint2dInFrame(midFeetZUp);

      FramePoint2d desiredCapturePoint = new FramePoint2d(midFeetZUp, 0.04, 0.0);
//      FramePoint2d desiredCapturePoint = bipedSupportPolygons.getSupportPolygonInMidFeetZUp().getCentroidCopy();
//      FramePoint2d desiredCapturePoint = bipedSupportPolygons.getSweetSpotCopy(RobotSide.RIGHT);
      desiredCapturePoint.changeFrame(midFeetZUp);
      FramePoint2d desiredCoP = desiredCapturePointToDesiredCoPControlModule.computeDesiredCoPDoubleSupport(bipedSupportPolygons, capturePoint,
                                   new FrameVector2d(midFeetZUp), desiredCapturePoint);
      FramePoint2d filteredDesiredCoP = desiredCenterOfPressureFilter.filter(desiredCoP, null);

      return filteredDesiredCoP;
   }

   private FrameVector determineDesiredDeltaCMP()
   {
      FrameVector zUnitVector = new FrameVector(midFeetZUp, 0.0, 0.0, 1.0);
      FrameVector angularMomentum = processedSensors.getAngularMomentumInFrame(midFeetZUp);
      FrameVector desiredDeltaCMP = new FrameVector(midFeetZUp);
      desiredDeltaCMP.cross(angularMomentum, zUnitVector);
      desiredDeltaCMP.scale(-kAngularMomentumXY.getDoubleValue());

      Transform3D pelvisToMidFeetZUp = fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(midFeetZUp);
      Matrix3d pelvisToMidFeetZUpRotation = new Matrix3d();
      pelvisToMidFeetZUp.get(pelvisToMidFeetZUpRotation);
      AxisAngle4d pelvisToCenterOfMassAxisAngle = new AxisAngle4d();
      pelvisToCenterOfMassAxisAngle.set(pelvisToMidFeetZUpRotation);
      FrameVector proportionalPart = new FrameVector(midFeetZUp, pelvisToCenterOfMassAxisAngle.getX(), pelvisToCenterOfMassAxisAngle.getY(),
                                        pelvisToCenterOfMassAxisAngle.getZ());
      proportionalPart.scale(pelvisToCenterOfMassAxisAngle.getAngle());
      proportionalPart.cross(proportionalPart, zUnitVector);
      proportionalPart.scale(-kPelvisAxisAngle.getDoubleValue());
      desiredDeltaCMP.add(proportionalPart);

      return desiredDeltaCMP;
   }

   private FrameVector determineGroundReactionMoment()
   {
      FrameVector ret = new FrameVector(midFeetZUp);
      FrameVector angularMomentum = processedSensors.getAngularMomentumInFrame(midFeetZUp);
      ret.setZ(-kAngularMomentumZ.getDoubleValue() * angularMomentum.getZ());

      return ret;
   }

   private void updateYoVariables()
   {
      SpatialAccelerationVector pelvisAcceleration = new SpatialAccelerationVector();
      fullRobotModel.getRootJoint().packDesiredJointAcceleration(pelvisAcceleration);
      desiredPelvisLinearAcceleration.set(pelvisAcceleration.getLinearPartCopy());
      desiredPelvisAngularAcceleration.set(pelvisAcceleration.getAngularPartCopy());

      Wrench pelvisJointWrench = new Wrench();
      fullRobotModel.getRootJoint().packWrench(pelvisJointWrench);
      pelvisJointWrench.changeFrame(centerOfMassFrame);
      desiredPelvisForce.set(pelvisJointWrench.getLinearPartCopy());
      desiredPelvisTorque.set(pelvisJointWrench.getAngularPartCopy());
   }

   private void solveForPelvisJointAcceleration()
   {
      info[1] = 0;
      Minpack_f77.lmdif1_f77(nlls, m, n, x, fvec, tol, info);    // also sets the result in the full robot model
      nlls.fcn(m, n, x, fvec, info);
   }

   public class InverseDynamicsOptimizationFunction implements Lmdif_fcn
   {
      private final Matrix sixDoFJointAccelerations = new Matrix(6, 1);
      private final Matrix jointAccelerations;
      private final Matrix centroidalMomentumErrorMatrix = new Matrix(Momentum.SIZE, 1);
      private final SpatialAccelerationVector accelerationOfPelvisWithRespectToWorld = new SpatialAccelerationVector();
      private final double w = 0.999999999;

      public InverseDynamicsOptimizationFunction(int nDegreesOfFreedom)
      {
         jointAccelerations = new Matrix(nDegreesOfFreedom, 1);
      }

      public void fcn(int m, int n, double[] x, double[] fvec, int[] iflag)
      {
         setMatrixFromOneBasedArray(sixDoFJointAccelerations, x);
         fullRobotModel.getRootJoint().setDesiredAcceleration(sixDoFJointAccelerations, 0);
         fullRobotModel.getRootJoint().packDesiredSuccessorAcceleration(accelerationOfPelvisWithRespectToWorld);

         for (RobotSide robotSide : RobotSide.values())
         {
            SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBase = computeDesiredSpatialAccelerationOfPelvisWithRespectToFoot(robotSide);
            desiredJointAccelerationCalculators.get(robotSide).compute(accelerationOfEndEffectorWithRespectToBase,
                    alphaDesiredJointAcceleration.getDoubleValue());
         }


         int index = 1;
         ScrewTools.packDesiredJointAccelerationsMatrix(jointsInOrder, jointAccelerations);

         centroidalMomentumErrorMatrix.set(0, 0, desiredAngularCentroidalMomentumRate.getX());
         centroidalMomentumErrorMatrix.set(1, 0, desiredAngularCentroidalMomentumRate.getY());
         centroidalMomentumErrorMatrix.set(2, 0, desiredAngularCentroidalMomentumRate.getZ());
         centroidalMomentumErrorMatrix.set(3, 0, desiredLinearCentroidalMomentumRate.getX());
         centroidalMomentumErrorMatrix.set(4, 0, desiredLinearCentroidalMomentumRate.getY());
         centroidalMomentumErrorMatrix.set(5, 0, desiredLinearCentroidalMomentumRate.getZ());

         Matrix aQdd = centroidalMomentumMatrix.getMatrix().times(jointAccelerations);
         Matrix adQd = centroidalMomentumMatrixDerivative.times(jointVelocitiesMatrix);
         centroidalMomentumErrorMatrix.minusEquals(aQdd);
         centroidalMomentumErrorMatrix.minusEquals(adQd);

         angularCentroidalMomentumRateError.set(centroidalMomentumErrorMatrix.get(0, 0), centroidalMomentumErrorMatrix.get(1, 0),
                 centroidalMomentumErrorMatrix.get(2, 0));
         linearCentroidalMomentumRateError.set(centroidalMomentumErrorMatrix.get(3, 0), centroidalMomentumErrorMatrix.get(4, 0),
                 centroidalMomentumErrorMatrix.get(5, 0));

         for (int i = 0; i < centroidalMomentumErrorMatrix.getRowDimension(); i++)
         {
            fvec[index++] = w * (centroidalMomentumErrorMatrix.get(i, 0));
         }
      }

      private SpatialAccelerationVector computeDesiredSpatialAccelerationOfPelvisWithRespectToFoot(RobotSide robotSide)
      {
         SpatialAccelerationVector accelerationOfPelvisWithRespectToFoot;
         ReferenceFrame footFrame = fullRobotModel.getFoot(robotSide).getBodyFixedFrame();
         ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
         ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();
         RobotSide supportSide = supportLeg.getEnumValue();
         if ((supportSide == null) || (supportSide == robotSide))
         {
            accelerationOfPelvisWithRespectToFoot = new SpatialAccelerationVector(footFrame, elevatorFrame, pelvisFrame);    // acceleration of foot with respect to world
         }
         else
         {
            ReferenceFrame swingAnkleZUp = referenceFrames.getAnkleZUpFrame(robotSide);
            Twist twistOfFootWithRespectToElevator = new Twist();
            twistCalculator.packTwistOfBody(twistOfFootWithRespectToElevator, fullRobotModel.getFoot(robotSide));

            double kSwingFootPosition = 0.0;
            double bSwingFootPosition = 0.1;
            FramePoint desiredPosition = new FramePoint(referenceFrames.getAnkleZUpFrame(supportSide), 0.0, supportSide.negateIfLeftSide(0.38), 0.01);
            desiredPosition.changeFrame(swingAnkleZUp);
            FrameVector swingFootPositionError = new FrameVector(desiredPosition);
            swingFootPositionError.changeFrame(worldFrame);
            swingFootPositionErrorInWorld.set(swingFootPositionError);
            FrameVector currentVelocity = twistOfFootWithRespectToElevator.getBodyOriginLinearVelocityInBaseFrame();
            FrameVector velocityTerm = new FrameVector(currentVelocity.getReferenceFrame()); // desired velocity at this point
            velocityTerm.sub(currentVelocity); // velocity error at this point
            velocityTerm.changeFrame(swingAnkleZUp);
            velocityTerm.scale(bSwingFootPosition); // velocity term
            
            FrameVector linearAcceleration = new FrameVector(desiredPosition);
            linearAcceleration.scale(kSwingFootPosition);
            linearAcceleration.add(velocityTerm);

            FrameVector angularAcceleration = new FrameVector(referenceFrames.getAnkleZUpFrame(robotSide));
            Orientation desiredOrientation = new Orientation(referenceFrames.getAnkleZUpFrame(supportSide), 0.0, 0.0, 0.0);
            FrameVector desiredAngularVelocity = new FrameVector(referenceFrames.getAnkleZUpFrame(robotSide));    // TODO
            FrameVector currentAngularVelocity = new FrameVector(referenceFrames.getAnkleZUpFrame(robotSide));    // TODO
            footOrientationControllers.get(robotSide).compute(angularAcceleration, desiredOrientation, desiredAngularVelocity, currentAngularVelocity);

            accelerationOfPelvisWithRespectToFoot = new SpatialAccelerationVector(footFrame, elevatorFrame, footFrame);
            twistOfFootWithRespectToElevator.changeBaseFrameNoRelativeTwist(elevatorFrame);
            accelerationOfPelvisWithRespectToFoot.setBasedOnOriginAcceleration(angularAcceleration, linearAcceleration, twistOfFootWithRespectToElevator);
            
            Twist twistOfFootWithRespectToPelvis = new Twist();
            twistCalculator.packTwistOfBody(twistOfFootWithRespectToPelvis, fullRobotModel.getPelvis());
            twistOfFootWithRespectToPelvis.changeBaseFrameNoRelativeTwist(elevatorFrame);
            twistOfFootWithRespectToPelvis.invert();
            twistOfFootWithRespectToPelvis.changeFrame(footFrame);
            twistOfFootWithRespectToPelvis.add(twistOfFootWithRespectToElevator);
            accelerationOfPelvisWithRespectToFoot.changeFrame(pelvisFrame, twistOfFootWithRespectToPelvis, twistOfFootWithRespectToElevator);
         }

         accelerationOfPelvisWithRespectToFoot.invert();    // acceleration of world with respect to foot
         accelerationOfPelvisWithRespectToFoot.add(accelerationOfPelvisWithRespectToWorld);    // acceleration of pelvis with respect to foot


         return accelerationOfPelvisWithRespectToFoot;
      }
   }

   private static void doPDControl(double k, double d, RevoluteJoint[] joints)
   {
      for (RevoluteJoint joint : joints)
         joint.setQddDesired(computeDesiredAcceleration(k, d, joint));
   }

   private static double computeDesiredAcceleration(double k, double d, RevoluteJoint joint)
   {
      return -k * joint.getQ() - d * joint.getQd();
   }

   private static void setMatrixFromOneBasedArray(Matrix ret, double[] oneBasedArray)
   {
      for (int i = 0; i < oneBasedArray.length - 1; i++)
      {
         ret.set(i, 0, oneBasedArray[i + 1]);
      }
   }
}
