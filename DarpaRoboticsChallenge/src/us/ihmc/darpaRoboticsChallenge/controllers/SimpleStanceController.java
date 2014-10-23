package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.controllers.AxisAngleOrientationController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

import com.yobotics.simulationconstructionset.robotController.RobotController;

/**
 * @author twan
 *         Date: 5/8/13
 */
public class SimpleStanceController implements RobotController
{
   private final String name = SimpleStanceController.class.getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SDFPerfectSimulatedSensorReader sensorReader;
   private final SDFPerfectSimulatedOutputWriter outputWriter;
   private final OptimizationMomentumControlModule momentumControlModule;
   private final SideDependentList<GeometricJacobian> footJacobians = new SideDependentList<GeometricJacobian>();
   private final GeometricJacobian pelvisJacobian;
   private final DenseMatrix64F orientationSelectionMatrix;
   private final LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates;
   private final TwistCalculator twistCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final AxisAngleOrientationController pelvisOrientationController;
   private final CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final FullRobotModel fullRobotModel;
   private final ReferenceFrame centerOfMassFrame;
   private final YoFrameVector desiredPelvisAngularAcceleration;
   private final Map<OneDoFJoint, DoubleYoVariable> desiredJointAccelerationYoVariables = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
   private final YoFrameVector desiredPelvisTorque;
   private final YoFrameVector desiredPelvisForce;
   private final OneDoFJoint[] oneDoFJoints;
   private final SixDoFJoint rootJoint;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   public SimpleStanceController(SDFRobot robot, SDFFullRobotModel fullRobotModel, ReferenceFrames referenceFrames, double controlDT,
                                 InverseDynamicsJoint[] jointsToOptimize, double gravityZ, double footForward, double footBack, double footWidth)
   {
      this.sensorReader = new SDFPerfectSimulatedSensorReader(robot, fullRobotModel, referenceFrames);
      this.outputWriter = new SDFPerfectSimulatedOutputWriter(robot, fullRobotModel);
      MomentumOptimizationSettings momentumOptimizationSettings = createOptimizationSettings(jointsToOptimize, 1.0, 5e-2, 1e-5, 0.0);
      rootJoint = fullRobotModel.getRootJoint();
      twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullRobotModel.getPelvis());
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         GeometricJacobian jacobian = new GeometricJacobian(fullRobotModel.getElevator(), foot, foot.getBodyFixedFrame());
         footJacobians.put(robotSide, jacobian);
      }

      RigidBody pelvis = fullRobotModel.getPelvis();
      pelvisJacobian = new GeometricJacobian(fullRobotModel.getElevator(), pelvis, pelvis.getBodyFixedFrame());

      orientationSelectionMatrix = new DenseMatrix64F(3, Momentum.SIZE);
      CommonOps.setIdentity(orientationSelectionMatrix);

      SideDependentList<ContactablePlaneBody> feet = createFeet(fullRobotModel, referenceFrames, footForward, footBack, footWidth);
      double coefficientOfFriction = 1.0;
      contactStates = createContactStates(feet, registry, coefficientOfFriction);
      
      this.momentumControlModule = new OptimizationMomentumControlModule(rootJoint, referenceFrames.getCenterOfMassFrame(), controlDT, gravityZ,
            momentumOptimizationSettings, twistCalculator, null, contactStates.values(), null, registry);

      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      this.pelvisOrientationController = new AxisAngleOrientationController("pelvis", pelvis.getBodyFixedFrame(), controlDT, registry);
      pelvisOrientationController.setProportionalGains(5.0, 5.0, 5.0);
      pelvisOrientationController.setDerivativeGains(1.0, 1.0, 1.0);

      centerOfMassJacobian = new CenterOfMassJacobian(rootJoint.getSuccessor());
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      momentumRateOfChangeControlModule = new CoMBasedMomentumRateOfChangeControlModule(controlDT, centerOfMassFrame, centerOfMassJacobian, registry);
      momentumRateOfChangeControlModule.setProportionalGains(100.0, 100.0, 100.0);
      momentumRateOfChangeControlModule.setDerivativeGains(20.0, 20.0, 20.0);

      desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", rootJoint.getSuccessor().getBodyFixedFrame(), registry);

      this.fullRobotModel = fullRobotModel;

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());
      oneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         DoubleYoVariable yoVariable = new DoubleYoVariable("qdd_d_" + oneDoFJoint.getName(), registry);
         desiredJointAccelerationYoVariables.put(oneDoFJoint, yoVariable);
      }

      desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", rootJoint.getSuccessor().getBodyFixedFrame(), registry);
      desiredPelvisForce = new YoFrameVector("desiredPelvisForce", rootJoint.getSuccessor().getBodyFixedFrame(), registry);

      spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootJoint.getPredecessor(), twistCalculator, gravityZ, true);
   }

   private static MomentumOptimizationSettings createOptimizationSettings(InverseDynamicsJoint[] jointsToOptimizeFor, double momentumWeight, double lambda, double wRho, double rhoMin)
   {
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(jointsToOptimizeFor, new YoVariableRegistry("test1"));
      momentumOptimizationSettings.setMomentumWeight(momentumWeight, momentumWeight, momentumWeight, momentumWeight);
      momentumOptimizationSettings.setDampedLeastSquaresFactor(lambda);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(wRho);
      momentumOptimizationSettings.setRhoMin(rhoMin);

      return momentumOptimizationSettings;
   }

   public void doControl()
   {
      sensorReader.read();

      twistCalculator.compute();

      centerOfMassJacobian.compute();

      momentumControlModule.reset();

      inverseDynamicsCalculator.reset();

      constrainFeet();

      FrameVector desiredPelvisAngularAcceleration = constrainPelvis();

      controlLinearMomentum();

      setJointAccelerationsAndWrenches();

      inverseDynamicsCalculator.compute();

      outputWriter.write();

      SpatialAccelerationVector pelvisAcceleration = new SpatialAccelerationVector();

      spatialAccelerationCalculator.compute();
      spatialAccelerationCalculator.packAccelerationOfBody(pelvisAcceleration, fullRobotModel.getPelvis());
      FrameVector angularAccelerationBack = new FrameVector(pelvisAcceleration.getExpressedInFrame());

      pelvisAcceleration.packAngularPart(angularAccelerationBack);

//    angularAccelerationBack.changeFrame(desiredPelvisAngularAcceleration.getReferenceFrame());
//    if (!desiredPelvisAngularAcceleration.epsilonEquals(angularAccelerationBack, 1e-12))
//       throw new RuntimeException();

      this.desiredPelvisAngularAcceleration.set(desiredPelvisAngularAcceleration);

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         desiredJointAccelerationYoVariables.get(oneDoFJoint).set(oneDoFJoint.getQddDesired());
      }

      Wrench wrench = new Wrench();
      rootJoint.packWrench(wrench);

      FrameVector pelvisTorque = new FrameVector(rootJoint.getSuccessor().getBodyFixedFrame());
      wrench.packAngularPart(pelvisTorque);
      desiredPelvisTorque.set(pelvisTorque);

      FrameVector pelvisForce = new FrameVector(rootJoint.getSuccessor().getBodyFixedFrame());
      wrench.packLinearPart(pelvisForce);
      desiredPelvisForce.set(pelvisForce);

   }

   private void controlLinearMomentum()
   {
      momentumRateOfChangeControlModule.startComputation();
      MomentumRateOfChangeData momentumRateOfChangeData = momentumRateOfChangeControlModule.getMomentumRateOfChangeOutputPort().getData();
      
      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      momentumControlModule.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);
   }

   private void setJointAccelerationsAndWrenches()
   {
      MomentumModuleSolution momentumModuleSolution;
      try
      {
         momentumModuleSolution = momentumControlModule.compute(contactStates, null);
      }
      catch (MomentumControlModuleException momentumControlModuleException)
      {
         momentumControlModuleException.printStackTrace();
         momentumModuleSolution = momentumControlModuleException.getMomentumModuleSolution();
      }

      Map<RigidBody, Wrench> externalWrenches = momentumModuleSolution.getExternalWrenchSolution();

      for (RigidBody rigidBody : externalWrenches.keySet())
      {
         inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrenches.get(rigidBody));
      }
   }

   private void constrainFeet()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         GeometricJacobian jacobian = footJacobians.get(robotSide);
         TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
         taskspaceConstraintData.set(new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), jacobian.getBaseFrame(), jacobian.getEndEffectorFrame()));
         
         DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskspaceConstraintData);
         momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
      }
   }

   private FrameVector constrainPelvis()
   {
      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();

      ReferenceFrame pelvisFrame = pelvisJacobian.getEndEffectorFrame();
      FrameVector output = new FrameVector(pelvisFrame);
      FrameOrientation desiredOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
      desiredOrientation.changeFrame(pelvisFrame);
      FrameVector desiredAngularVelocity = new FrameVector(pelvisFrame);
      Twist twist = new Twist();
      twistCalculator.packRelativeTwist(twist, pelvisJacobian.getBase(), pelvisJacobian.getEndEffector());
      FrameVector currentAngularVelocity = new FrameVector(pelvisFrame);
      twist.packAngularPart(currentAngularVelocity);

      FrameVector feedForwardAngularAcceleration = new FrameVector(pelvisFrame);
      pelvisOrientationController.compute(output, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAcceleration);

      DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);

      taskspaceConstraintData.setAngularAcceleration(pelvisFrame, pelvisJacobian.getBaseFrame(), output, nullspaceMultipliers);

      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(pelvisJacobian, taskspaceConstraintData, 1.0);
      momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);


//    DenseMatrix64F jointAcceleration = new DenseMatrix64F(3, 1);
//    MatrixTools.setDenseMatrixFromTuple3d(jointAcceleration, output.getVector(), 0, 0);
//    momentumControlModule.setDesiredJointAcceleration(fullRobotModel.getRootJoint(), jointAcceleration);

      return output;
   }

   public void initialize()
   {
      sensorReader.read();

      twistCalculator.compute();

      momentumControlModule.initialize();

      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(fullRobotModel.getRootJoint().getSuccessor(), ReferenceFrame.getWorldFrame());
      centerOfMassCalculator.compute();
      FramePoint centerOfMass = centerOfMassCalculator.getCenterOfMass();
      ControlFlowInputPort<FramePoint> desiredCoMPositionInputPort = momentumRateOfChangeControlModule.getDesiredCoMPositionInputPort();
      desiredCoMPositionInputPort.setData(centerOfMass);
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

   private SideDependentList<ContactablePlaneBody> createFeet(SDFFullRobotModel fullRobotModel, ReferenceFrames referenceFrames, double footForward,
           double footBack, double footWidth)
   {
      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      return bipedFeet;
   }

   private LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> createContactStates(SideDependentList<ContactablePlaneBody> feet,
           YoVariableRegistry registry, double coefficientOfFriction)
   {
      LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();
      for (ContactablePlaneBody contactablePlaneBody : feet)
      {
         YoPlaneContactState contactState = new YoPlaneContactState(contactablePlaneBody.getName() + "ContactState", contactablePlaneBody.getRigidBody(),
                                               contactablePlaneBody.getSoleFrame(), contactablePlaneBody.getContactPoints2d(), coefficientOfFriction, registry);
         contactStates.put(contactablePlaneBody, contactState);
      }

      return contactStates;
   }

   public static InverseDynamicsJoint[] createJointsToOptimize(SDFFullRobotModel fullRobotModel)
   {
      List<InverseDynamicsJoint> joints = new ArrayList<InverseDynamicsJoint>();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      joints.addAll(Arrays.asList(allJoints));

      for (RobotSide robotSide : RobotSide.values)
      {
         List<InverseDynamicsJoint> fingerJoints = Arrays.asList(ScrewTools.computeSubtreeJoints(fullRobotModel.getHand(robotSide)));
         joints.removeAll(fingerJoints);
      }

//      joints.remove(fullRobotModel.getLidarJoint());

      return joints.toArray(new InverseDynamicsJoint[joints.size()]);
   }

   
}
