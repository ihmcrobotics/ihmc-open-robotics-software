package us.ihmc.commonWalkingControlModules.controlModules.head;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.robotics.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.OriginAndPointFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


public class HeadOrientationControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameQuaternion orientationToTrack;
   private final YoFramePoint pointToTrack;
   private final ReferenceFrame headFrame;
   private final OriginAndPointFrame pointTrackingFrame;
   private final YoGraphicReferenceFrame pointTrackingFrameFiz;
   private final FramePoint positionToPointAt = new FramePoint();

   private enum HeadTrackingMode {ORIENTATION, POINT}

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final EnumYoVariable<HeadTrackingMode> headTrackingMode = EnumYoVariable.create("headTrackingMode", HeadTrackingMode.class, registry);

   private final DoubleYoVariable yawLimit = new DoubleYoVariable("yawLimit", registry);
   private final DoubleYoVariable pitchLowerLimit = new DoubleYoVariable("pitchLowerLimit", registry);
   private final DoubleYoVariable pitchUpperLimit = new DoubleYoVariable("pitchUpperLimit", registry);
   private final DoubleYoVariable rollLimit = new DoubleYoVariable("rollLimit", registry);

   private final RigidBody head;
   private final RigidBody elevator;
   private final ReferenceFrame baseFrame;

   private final MomentumBasedController momentumBasedController;
   private final RigidBodyOrientationControlModule controlModule;

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector desiredAngularAcceleration = new FrameVector();
   private final FrameVector controlledLinearAcceleration = new FrameVector();
   private final FrameVector controlledAngularAcceleration = new FrameVector();
   private final SpatialAccelerationVector controlledSpatialAcceleration = new SpatialAccelerationVector();

   private final RigidBodyTransform desiredHeadTransform = new RigidBodyTransform();

   private final DenseMatrix64F selectionMatrix;
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);

   private final int jacobianId;
   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();

   private final OneDoFJoint[] headOrientationControlJoints;
   private final NumericalInverseKinematicsCalculator numericalInverseKinematicsCalculator;
   private final DenseMatrix64F desiredJointAngles;
   private final DoubleYoVariable[] yoDesiredJointAngles;

   private final BooleanYoVariable doPositionControl = new BooleanYoVariable("doPositionControlForNeck", registry);
   private final boolean[] doIntegrateDesiredAccerations;

   /*
    * TODO Sylvain. In walking, the head in controlled with respect to the
    * elevator (see WalkingHighLevelHumanoidController.setupManagers()). This
    * causes the head to have a little lag when rotating the pelvis (around z
    * only). The best option would be to control the head with respect to two
    * bases: for the pitch and roll with respect to the elevator, and for the
    * yaw with respect to the pelvis.
    */

   public HeadOrientationControlModule(MomentumBasedController momentumBasedController, ReferenceFrame headOrientationExpressedInFrame,
         HeadOrientationControllerParameters headOrientationControllerParameters, YoOrientationPIDGains gains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      head = fullRobotModel.getHead();
      headFrame = head.getBodyFixedFrame();
      elevator = fullRobotModel.getElevator();
      taskspaceConstraintData.set(elevator, head);

      doPositionControl.set(headOrientationControllerParameters.isNeckPositionControlled());

      String[] headOrientationControlJointNames = headOrientationControllerParameters.getDefaultHeadOrientationControlJointNames();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      headOrientationControlJoints = ScrewTools.filterJoints(ScrewTools.findJointsWithNames(allJoints, headOrientationControlJointNames), OneDoFJoint.class);

      jacobianId = momentumBasedController.getOrCreateGeometricJacobian(headOrientationControlJoints, headFrame);

      yoDesiredJointAngles = new DoubleYoVariable[headOrientationControlJoints.length];

      for (int i = 0; i < headOrientationControlJoints.length; i++)
      {
         String jointName = headOrientationControlJoints[i].getName();
         yoDesiredJointAngles[i] = new DoubleYoVariable(jointName + "QDesired", parentRegistry);
      }

      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      double dt = momentumBasedController.getControlDT();
      pointTrackingFrame = new OriginAndPointFrame("headPointTrackingFrame", worldFrame);

      orientationToTrack = new YoFrameQuaternion("headOrientationToTrack", headOrientationExpressedInFrame, registry);
      pointToTrack = new YoFramePoint("headPointToTrack", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         pointTrackingFrameFiz = new YoGraphicReferenceFrame(pointTrackingFrame, registry, 0.3);
         YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
         yoGraphicsList.add(pointTrackingFrameFiz);
         yoGraphicsList.hideYoGraphics();
      }
      else
      {
         pointTrackingFrameFiz = null;
      }

      if (jacobianId != -1)
      {
         controlModule = new RigidBodyOrientationControlModule("head", elevator, head, twistCalculator, dt, gains, registry);

         baseFrame = momentumBasedController.getJacobian(jacobianId).getBaseFrame();
         InverseDynamicsJoint[] cloneOfControlledJoints = ScrewTools.cloneJointPath(headOrientationControlJoints);
         int numberOfDoFs = cloneOfControlledJoints.length;
         RigidBody cloneOfHead = cloneOfControlledJoints[numberOfDoFs - 1].getSuccessor();
         ReferenceFrame cloneOfHeadFrame = cloneOfHead.getBodyFixedFrame();
         GeometricJacobian jacobian = new GeometricJacobian(cloneOfControlledJoints, cloneOfHeadFrame);
         
         int maxIterations = 2;
         double lambdaLeastSquares = 0.0009;
         double tolerance = 1e-12;
         double maxStepSize = 0.01;
         double minRandomSearchScalar = 1.0;
         double maxRandomSearchScalar = 1.0;
         numericalInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(jacobian, lambdaLeastSquares, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
         desiredJointAngles = new DenseMatrix64F(numberOfDoFs, 1);

         selectionMatrix = new DenseMatrix64F(numberOfDoFs, SpatialMotionVector.SIZE);
         for (int i = 0; i < numberOfDoFs; i++)
         {
            FrameVector jointAxis = headOrientationControlJoints[i].getJointAxis();
            jointAxis.changeFrame(baseFrame);
            selectionMatrix.set(i, 0, jointAxis.getX());
            selectionMatrix.set(i, 1, jointAxis.getY());
            selectionMatrix.set(i, 2, jointAxis.getZ());
         }
      }
      else
      {
         controlModule = null;

         baseFrame = null;
         numericalInverseKinematicsCalculator = null;
         desiredJointAngles = null;
         selectionMatrix = null;
      }

      setHeadOrientationLimits(headOrientationControllerParameters);

      doIntegrateDesiredAccerations = new boolean[headOrientationControlJoints.length];
      saveDoAccelerationIntegration();

      parentRegistry.addChild(registry);
   }

   public RigidBody getHead()
   {
      return head;
   }

   private final FramePoint headPosition = new FramePoint();

   public void compute()
   {
      if (jacobianId == -1) // Nothing to control there
         return;

      if (doPositionControl.getBooleanValue())
      {
         enablePositionControl();
      }
      else
      {
         disablePositionControl();
      }

      packDesiredFrameOrientation(desiredOrientation);
      packDesiredAngularVelocity(desiredAngularVelocity);
      packDesiredAngularAccelerationFeedForward(desiredAngularAcceleration);

      if (doPositionControl.getBooleanValue())
         computeJointsDesiredOrientationAndAngularVelocity();

      computeJointsDesiredAcceleration();
   }

   private void computeJointsDesiredOrientationAndAngularVelocity()
   {
      numericalInverseKinematicsCalculator.setSelectionMatrix(selectionMatrix);
      desiredOrientation.changeFrame(baseFrame);
      desiredOrientation.getTransform3D(desiredHeadTransform);
      numericalInverseKinematicsCalculator.solve(desiredHeadTransform);
      numericalInverseKinematicsCalculator.getBest(desiredJointAngles);

      for (int i = 0; i < headOrientationControlJoints.length; i++)
      {
         OneDoFJoint joint = headOrientationControlJoints[i];
         double qDesired = desiredJointAngles.get(i, 0);
         joint.setqDesired(qDesired);
         yoDesiredJointAngles[i].set(qDesired);
      }
   }

   private void computeJointsDesiredAcceleration()
   {
      controlModule.compute(controlledAngularAcceleration, desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      controlledLinearAcceleration.setToZero(headFrame);
      
      controlledSpatialAcceleration.set(headFrame, elevator.getBodyFixedFrame(), headFrame, controlledLinearAcceleration, controlledAngularAcceleration);
      
      taskspaceConstraintData.set(controlledSpatialAcceleration, nullspaceMultipliers, selectionMatrix);
      momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
   }

   private void saveDoAccelerationIntegration()
   {
      for (int i = 0; i < headOrientationControlJoints.length; i++)
      {
         OneDoFJoint joint = headOrientationControlJoints[i];
         doIntegrateDesiredAccerations[i] = joint.getIntegrateDesiredAccelerations();
      }
   }

   private void enablePositionControl()
   {
      for (int i = 0; i < headOrientationControlJoints.length; i++)
      {
         OneDoFJoint joint = headOrientationControlJoints[i];
         joint.setIntegrateDesiredAccelerations(false);
         joint.setUnderPositionControl(true);
      }
   }

   private void disablePositionControl()
   {
      for (int i = 0; i < headOrientationControlJoints.length; i++)
      {
         OneDoFJoint joint = headOrientationControlJoints[i];
         joint.setIntegrateDesiredAccelerations(doIntegrateDesiredAccerations[i]);
         joint.setUnderPositionControl(false);
         yoDesiredJointAngles[i].set(Double.NaN);
      }
   }

   protected void packDesiredFrameOrientation(FrameOrientation orientationToPack)
   {
      pointToTrack.getFrameTupleIncludingFrame(positionToPointAt);
      headPosition.setToZero(headFrame);
      pointTrackingFrame.setOriginAndPositionToPointAt(headPosition, positionToPointAt);

      switch (headTrackingMode.getEnumValue())
      {
         case ORIENTATION :
         {
            orientationToPack.setToZero(orientationToTrack.getReferenceFrame());
            orientationToTrack.getFrameOrientationIncludingFrame(orientationToPack);

            break;
         }

         case POINT :
         {
            pointTrackingFrame.update();
            

            orientationToPack.setToZero(pointTrackingFrame);

            break;
         }

         default :
            throw new RuntimeException("Case " + headTrackingMode.getEnumValue() + " not handled.");
      }

      orientationToPack.changeFrame(baseFrame);
   }

   private void setHeadOrientationLimits(HeadOrientationControllerParameters headOrientationControllerParameters)
   {
      yawLimit.set(headOrientationControllerParameters.getHeadYawLimit());
      pitchUpperLimit.set(headOrientationControllerParameters.getNeckPitchUpperLimit());
      pitchLowerLimit.set(headOrientationControllerParameters.getNeckPitchLowerLimit());
      rollLimit.set(headOrientationControllerParameters.getHeadRollLimit());
   }

   protected void packDesiredAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(worldFrame);
   }

   protected void packDesiredAngularAccelerationFeedForward(FrameVector angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(worldFrame);
   }

   public void setOrientationToTrack(FrameOrientation orientation)
   {
      this.headTrackingMode.set(HeadTrackingMode.ORIENTATION);
      this.orientationToTrack.set(orientation);
   }

   public void setPointToTrack(FramePoint point)
   {
      this.headTrackingMode.set(HeadTrackingMode.POINT);
      this.pointToTrack.set(point);
   }
}
