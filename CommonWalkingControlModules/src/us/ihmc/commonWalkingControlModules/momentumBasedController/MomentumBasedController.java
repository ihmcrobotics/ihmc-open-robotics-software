package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

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
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

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

   private final ProcessedSensorsInterface processedSensors;

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final AxisAnglePelvisOrientationControlModule orientationControlModule;
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

   private final MomentumBasedPelvisAccelerationOptimizer optimizer;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;

   private final EnumYoVariable<RobotSide> supportLeg = EnumYoVariable.create("supportLeg", RobotSide.class, registry);
   private final SideDependentList<AxisAngleOrientationController> footOrientationControllers = new SideDependentList<AxisAngleOrientationController>();
   private final SideDependentList<SpatialAccelerationVector> desiredFootAccelerationsInWorld = new SideDependentList<SpatialAccelerationVector>();
   private final YoFrameVector swingFootPositionErrorInWorld = new YoFrameVector("swingFootPositionErrorInWorld", "", worldFrame, registry);


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

   private final YoFrameVector2d desiredDeltaCMP = new YoFrameVector2d("desiredDeltaCMP", "", worldFrame, registry);
   private final SideDependentList<HashMap<FramePoint, BooleanYoVariable>> contactMap = SideDependentList.createListOfHashMaps();

   public MomentumBasedController(ProcessedSensorsInterface processedSensors, ProcessedOutputsInterface processedOutputs,
                                  CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator, double controlDT,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, double footForward, double footBack, double footWidth,
                                  double footHeight)
   {
      this.processedSensors = processedSensors;
      this.fullRobotModel = processedSensors.getFullRobotModel();
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.processedOutputs = processedOutputs;

      RigidBody elevator = fullRobotModel.getElevator();
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, -processedSensors.getGravityInWorldFrame().getZ());
      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();

      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame footFrame = foot.getBodyFixedFrame();
         AxisAngleOrientationController footOrientationController = new AxisAngleOrientationController(robotSide.getCamelCaseNameForStartOfExpression()
                                                                       + "Foot", footFrame, registry);
//         footOrientationController.setProportionalGains(100.0, 100.0, 100.0);
//         footOrientationController.setDerivativeGains(20.0, 20.0, 20.0);
         footOrientationControllers.put(robotSide, footOrientationController);
         desiredFootAccelerationsInWorld.put(robotSide, new SpatialAccelerationVector(footFrame, elevatorFrame, footFrame));
      }

      this.orientationControlModule = new AxisAnglePelvisOrientationControlModule(processedSensors, referenceFrames, null, registry, false);

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

      optimizer = new MomentumBasedPelvisAccelerationOptimizer(fullRobotModel, twistCalculator, referenceFrames.getCenterOfMassFrame(), registry, controlDT);

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

      desiredCoMHeight.set(1.2);
   }

   public void initialize()
   {
      optimizer.initialize();

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
   }

   private void doMomentumBasedControl()
   {
      // TODO: calculate CMP first, need to make sure that CoP is inside BoS, CMP may be outside
      FramePoint2d desiredCoP = determineDesiredCoP();
      FrameVector2d desiredDeltaCMP = determineDesiredDeltaCMP();
      FramePoint2d desiredCMP = new FramePoint2d(desiredCoP);
      desiredCMP.add(desiredDeltaCMP);
      
      desiredDeltaCMP.changeFrame(this.desiredDeltaCMP.getReferenceFrame());
      this.desiredDeltaCMP.set(desiredDeltaCMP);

      SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();
      SideDependentList<Double> legStrengths = new SideDependentList<Double>();
      RobotSide supportLeg = this.supportLeg.getEnumValue();
      ReferenceFrame frame;
      if (supportLeg == null)
      {
         frame = midFeetZUp;
         virtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, desiredCoP);
         legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, desiredCoP);
      }
      else
      {
         frame = referenceFrames.getAnkleZUpFrame(supportLeg);
         virtualToePoints.put(supportLeg, desiredCoP);
         legStrengths.put(supportLeg, 1.0);

         virtualToePoints.put(supportLeg.getOppositeSide(), new FramePoint2d(frame));
         legStrengths.put(supportLeg.getOppositeSide(), 0.0);
      }

      double fZ = centerOfMassHeightControlModule.doCenterOfMassHeightControl(desiredCoMHeight.getDoubleValue(), supportLeg);
      FrameVector totalgroundReactionMoment = determineGroundReactionMoment();
      Wrench totalGroundReactionWrench = new Wrench(centerOfMassFrame, centerOfMassFrame);

      for (RobotSide robotSide : RobotSide.values())
      {
         FramePoint groundReactionForceTerminalPoint = new FramePoint(centerOfMassFrame);
         groundReactionForceTerminalPoint.changeFrame(frame);
         desiredDeltaCMP.changeFrame(frame);
         groundReactionForceTerminalPoint.setX(groundReactionForceTerminalPoint.getX() - desiredDeltaCMP.getX());
         groundReactionForceTerminalPoint.setY(groundReactionForceTerminalPoint.getY() - desiredDeltaCMP.getY());
         FrameVector groundReactionForce = new FrameVector(groundReactionForceTerminalPoint);
         FramePoint virtualToePoint = virtualToePoints.get(robotSide).toFramePoint();
         groundReactionForce.sub(virtualToePoint);
         groundReactionForce.scale(legStrengths.get(robotSide) * fZ / groundReactionForce.getZ());

         FrameVector groundReactionMoment = new FrameVector(totalgroundReactionMoment);
         groundReactionMoment.scale(legStrengths.get(robotSide));

         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame footCoMFrame = foot.getBodyFixedFrame();
         FrameVector torque = new FrameVector(frame);
         torque.cross(virtualToePoint, groundReactionForce);
         Wrench groundReactionWrench = new Wrench(footCoMFrame, frame, groundReactionForce.getVector(), torque.getVector());
         groundReactionWrench.addAngularPart(groundReactionMoment.getVector());
         groundReactionWrench.changeFrame(footCoMFrame);
         inverseDynamicsCalculator.setExternalWrench(foot, groundReactionWrench);
         groundReactionWrench.changeFrame(centerOfMassFrame);
         groundReactionWrench.changeBodyFrameAttachedToSameBody(centerOfMassFrame);
         totalGroundReactionWrench.add(groundReactionWrench);
      }


      double kUpperBody = this.kUpperBody.getDoubleValue();
      double dUpperBody = 2.0 * zetaUpperBody.getDoubleValue() * Math.sqrt(kUpperBody);
      for (RobotSide robotSide : RobotSide.values())
      {
         doPDControl(kUpperBody, dUpperBody, fullRobotModel.getArmJointList(robotSide));
      }

      doPDControl(kUpperBody, dUpperBody, fullRobotModel.getSpineJointList());
      doPDControl(kUpperBody, dUpperBody, fullRobotModel.getNeckJointList());

      FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(totalGroundReactionWrench.getExpressedInFrame(),
                                                            totalGroundReactionWrench.getAngularPartCopy());
      FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(totalGroundReactionWrench.getExpressedInFrame(),
                                                           totalGroundReactionWrench.getLinearPartCopy());
      desiredLinearCentroidalMomentumRate.setZ(desiredLinearCentroidalMomentumRate.getZ() + processedSensors.getGravityInWorldFrame().getZ() * totalMass);

      computeDesiredFootAccelerations();

      optimizer.solveForPelvisJointAcceleration(desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, desiredFootAccelerationsInWorld);
   }

   private FramePoint2d determineDesiredCoP()
   {
//    FramePoint2d desiredCapturePoint = new FramePoint2d(midFeetZUp, 0.04, 0.0);
//    FramePoint2d desiredCapturePoint = bipedSupportPolygons.getSupportPolygonInMidFeetZUp().getCentroidCopy();
      FramePoint2d desiredCapturePoint = bipedSupportPolygons.getSweetSpotCopy(RobotSide.RIGHT);

      RobotSide supportLeg = this.supportLeg.getEnumValue();
      capturePointCalculator.computeCapturePoint(supportLeg);

      ReferenceFrame frame = supportLeg == null ? midFeetZUp : referenceFrames.getAnkleZUpFrame(supportLeg);
      FrameVector2d desiredVelocity = new FrameVector2d(frame);
      desiredCapturePoint.changeFrame(frame);
      FramePoint2d capturePoint = capturePointCalculator.getCapturePoint2dInFrame(frame);

      FramePoint2d desiredCoP;
      if (supportLeg == null)
      {
         desiredCoP = desiredCapturePointToDesiredCoPControlModule.computeDesiredCoPDoubleSupport(bipedSupportPolygons, capturePoint, desiredVelocity,
                 desiredCapturePoint);
      }
      else
      {
         desiredCoP = desiredCapturePointToDesiredCoPControlModule.computeDesiredCoPSingleSupport(supportLeg, bipedSupportPolygons,
                 capturePoint, desiredVelocity, desiredCapturePoint);
      }

      FramePoint2d filteredDesiredCoP = desiredCenterOfPressureFilter.filter(desiredCoP, supportLeg);

      return filteredDesiredCoP;
   }

   private FrameVector2d determineDesiredDeltaCMP()
   {
      RobotSide supportLeg = this.supportLeg.getEnumValue();
      ReferenceFrame frame = supportLeg == null ? midFeetZUp : referenceFrames.getAnkleZUpFrame(supportLeg);

      FrameVector zUnitVector = new FrameVector(frame, 0.0, 0.0, 1.0);
      FrameVector angularMomentum = processedSensors.getAngularMomentumInFrame(frame);
      FrameVector desiredDeltaCMP = new FrameVector(frame);
      desiredDeltaCMP.cross(angularMomentum, zUnitVector);
      desiredDeltaCMP.scale(-kAngularMomentumXY.getDoubleValue());

      Transform3D pelvisToMidFeetZUp = fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(frame);
      Matrix3d pelvisToMidFeetZUpRotation = new Matrix3d();
      pelvisToMidFeetZUp.get(pelvisToMidFeetZUpRotation);
      AxisAngle4d pelvisToCenterOfMassAxisAngle = new AxisAngle4d();
      pelvisToCenterOfMassAxisAngle.set(pelvisToMidFeetZUpRotation);
      FrameVector proportionalPart = new FrameVector(frame, pelvisToCenterOfMassAxisAngle.getX(), pelvisToCenterOfMassAxisAngle.getY(),
                                        pelvisToCenterOfMassAxisAngle.getZ());
      proportionalPart.scale(pelvisToCenterOfMassAxisAngle.getAngle());
      proportionalPart.cross(proportionalPart, zUnitVector);
      proportionalPart.scale(-kPelvisAxisAngle.getDoubleValue());
      desiredDeltaCMP.add(proportionalPart);

      return desiredDeltaCMP.toFrameVector2d();
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

   private void computeDesiredFootAccelerations()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame footFrame = foot.getBodyFixedFrame();
         ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();

         RobotSide supportSide = supportLeg.getEnumValue();
         if ((supportSide == null) || (supportSide == robotSide))
         {
            desiredFootAccelerationsInWorld.get(robotSide).setToZero(footFrame, elevatorFrame, footFrame);
         }
         else
         {
            ReferenceFrame supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(supportSide);
            Twist twistOfFootWithRespectToElevator = new Twist();
            twistCalculator.packTwistOfBody(twistOfFootWithRespectToElevator, fullRobotModel.getFoot(robotSide));

            double kSwingFootPosition = 100.0;
            double bSwingFootPosition = 20.0;
            FramePoint desiredPosition = new FramePoint(supportAnkleZUpFrame, 0.08, supportSide.negateIfLeftSide(0.38), 0.03);
            desiredPosition.changeFrame(footFrame);
            FrameVector swingFootPositionError = new FrameVector(desiredPosition);
            swingFootPositionError.changeFrame(worldFrame);
            swingFootPositionErrorInWorld.set(swingFootPositionError);
            FrameVector linearAcceleration = new FrameVector(swingFootPositionError);
            linearAcceleration.scale(kSwingFootPosition);

            FrameVector currentVelocity = twistOfFootWithRespectToElevator.getBodyOriginLinearVelocityInBaseFrame();
            FrameVector velocityTerm = new FrameVector(currentVelocity.getReferenceFrame());    // desired velocity at this point
            velocityTerm.sub(currentVelocity);    // velocity error at this point
            velocityTerm.changeFrame(worldFrame);
            velocityTerm.scale(bSwingFootPosition);    // velocity term

            linearAcceleration.add(velocityTerm);



//          FrameVector linearAcceleration = new FrameVector(worldFrame, 0.0, 0.0, -1.0);

            linearAcceleration.changeFrame(footFrame);
            linearAcceleration.scale(-1.0);

            FrameVector angularAcceleration = new FrameVector(footFrame);
            Orientation desiredOrientation = new Orientation(supportAnkleZUpFrame, 0.0, 0.0, 0.0);
            FrameVector desiredAngularVelocity = new FrameVector(footFrame);    // TODO
            FrameVector currentAngularVelocity = new FrameVector(footFrame);    // TODO
            footOrientationControllers.get(robotSide).compute(angularAcceleration, desiredOrientation, desiredAngularVelocity, currentAngularVelocity);
            angularAcceleration.scale(-1.0);

//          desiredFootAccelerationsInWorld.get(robotSide).setToZero(footFrame, elevatorFrame, footFrame);
            twistOfFootWithRespectToElevator.changeBaseFrameNoRelativeTwist(elevatorFrame);
            twistOfFootWithRespectToElevator.setToZero();
            desiredFootAccelerationsInWorld.get(robotSide).setBasedOnOriginAcceleration(angularAcceleration, linearAcceleration,
                    twistOfFootWithRespectToElevator);

//          desiredFootAccelerationsInWorld.get(robotSide).setAngularPart(angularAcceleration.getVector());
//          desiredFootAccelerationsInWorld.get(robotSide).setLinearPart(linearAcceleration.getVector());

//          desiredFootAccelerationsInWorld.get(robotSide).set(footFrame, elevatorFrame, footFrame, new Vector3d(0.0, 0.0, -0.1), new Vector3d());
         }
      }
   }

   private static void doPDControl(double k, double d, RevoluteJoint[] joints)
   {
      for (RevoluteJoint joint : joints)
      {
         joint.setQddDesired(computeDesiredAcceleration(k, d, joint));
      }
   }

   private static double computeDesiredAcceleration(double k, double d, RevoluteJoint joint)
   {
      return -k * joint.getQ() - d * joint.getQd();
   }
}
