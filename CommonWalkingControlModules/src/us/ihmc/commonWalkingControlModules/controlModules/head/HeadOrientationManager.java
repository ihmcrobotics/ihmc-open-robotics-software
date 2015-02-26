package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoQuaternionProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;


public class HeadOrientationManager
{
   private final YoVariableRegistry registry;
   private final HeadOrientationControlModule headOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private final HeadOrientationProvider desiredHeadOrientationProvider;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewHeadOrientationTime;
   private final DoubleYoVariable headOrientationTrajectoryTime;
   private final OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;

   private final BooleanYoVariable isTrackingOrientation;

   private final ReferenceFrame headOrientationExpressedInFrame;
   private final YoQuaternionProvider initialOrientationProvider;
   private final YoQuaternionProvider finalOrientationProvider;
   private int jacobianId = -1;

   private final BooleanYoVariable hasBeenInitialized;
   private final BooleanYoVariable doPositionControl;

   // In progress
//   /** Call this constructor to do position control on the neck joints */
//   public HeadOrientationManager(MomentumBasedController momentumBasedController, HeadOrientationProvider desiredHeadOrientationProvider,
//         double trajectoryTime, double[] initialDesiredHeadYawPitchRoll, YoVariableRegistry parentRegistry)
//   {
//      this(momentumBasedController, null, desiredHeadOrientationProvider, trajectoryTime, initialDesiredHeadYawPitchRoll, parentRegistry);
//   }

   /** Call this constructor to do force control on the neck */
   public HeadOrientationManager(MomentumBasedController momentumBasedController, HeadOrientationControlModule headOrientationControlModule,
         HeadOrientationProvider desiredHeadOrientationProvider, double trajectoryTime, double[] initialDesiredHeadYawPitchRoll, YoVariableRegistry parentRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      this.yoTime = momentumBasedController.getYoTime();
      this.headOrientationControlModule = headOrientationControlModule;
      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);
      doPositionControl = new BooleanYoVariable(getClass().getSimpleName() + "DoPositionControl", registry);
      doPositionControl.set(headOrientationControlModule == null);

      if (desiredHeadOrientationProvider != null)
      {
         headOrientationExpressedInFrame = desiredHeadOrientationProvider.getHeadOrientationExpressedInFrame();

         receivedNewHeadOrientationTime = new DoubleYoVariable("receivedNewHeadOrientationTime", registry);
         isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);
         headOrientationTrajectoryTime = new DoubleYoVariable("headOrientationTrajectoryTime", registry);

         hasBeenInitialized = new BooleanYoVariable("hasHeadOrientationManagerBeenInitialized", registry);
         hasBeenInitialized.set(false);

         headOrientationTrajectoryTime.set(trajectoryTime);
         DoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider(headOrientationTrajectoryTime);
         //         initialOrientationProvider = new CurrentOrientationProvider(headOrientationExpressedInFrame, headOrientationControlModule.getHead().getBodyFixedFrame());
         initialOrientationProvider = new YoQuaternionProvider("headInitialOrientation", headOrientationExpressedInFrame, registry);
         finalOrientationProvider = new YoQuaternionProvider("headFinalOrientation", headOrientationExpressedInFrame, registry);
         desiredOrientation.setIncludingFrame(headOrientationExpressedInFrame, initialDesiredHeadYawPitchRoll);
         finalOrientationProvider.setOrientation(desiredOrientation);
         orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("headOrientation", headOrientationExpressedInFrame,
               trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);
         orientationTrajectoryGenerator.setContinuouslyUpdateFinalOrientation(true);
      }
      else
      {
         headOrientationExpressedInFrame = null;
         receivedNewHeadOrientationTime = null;
         headOrientationTrajectoryTime = null;
         isTrackingOrientation = null;
         hasBeenInitialized = null;
         initialOrientationProvider = null;
         finalOrientationProvider = null;
         orientationTrajectoryGenerator = null;
      }
   }
   
   private void initialize()
   {
      if (hasBeenInitialized != null && hasBeenInitialized.getBooleanValue())
         return;

      hasBeenInitialized.set(true);

      ReferenceFrame headFrame = momentumBasedController.getFullRobotModel().getHead().getBodyFixedFrame();
      desiredOrientation.setToZero(headFrame);
      desiredOrientation.changeFrame(headOrientationExpressedInFrame);
      initialOrientationProvider.setOrientation(desiredOrientation);
      receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
      orientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
   }

   private final FrameOrientation desiredOrientation = new FrameOrientation();

   public void compute()
   {
      // Just to make sure it is not changed from SCS
      doPositionControl.set(headOrientationControlModule == null);

      initialize();

      checkForNewDesiredOrientationInformation();
      
      if (desiredHeadOrientationProvider != null && isTrackingOrientation.getBooleanValue())
      {
         double deltaTime = yoTime.getDoubleValue() - receivedNewHeadOrientationTime.getDoubleValue();
         orientationTrajectoryGenerator.compute(deltaTime);
         orientationTrajectoryGenerator.get(desiredOrientation);
         headOrientationControlModule.setOrientationToTrack(desiredOrientation);
         isTrackingOrientation.set(!orientationTrajectoryGenerator.isDone());
      }

      if (jacobianId >= 0)
      {
         headOrientationControlModule.compute();

         TaskspaceConstraintData taskspaceConstraintData = headOrientationControlModule.getTaskspaceConstraintData();
         momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
      }
   }

   private void checkForNewDesiredOrientationInformation()
   {
      if (desiredHeadOrientationProvider == null)
         return;

      if (desiredHeadOrientationProvider.isNewHeadOrientationInformationAvailable())
      {
         orientationTrajectoryGenerator.get(desiredOrientation);
         initialOrientationProvider.setOrientation(desiredOrientation);
         finalOrientationProvider.setOrientation(desiredHeadOrientationProvider.getDesiredHeadOrientation());
         receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
         orientationTrajectoryGenerator.initialize();
         isTrackingOrientation.set(true);
      }
      else if (desiredHeadOrientationProvider.isNewLookAtInformationAvailable())
      {
         orientationTrajectoryGenerator.get(desiredOrientation);
         initialOrientationProvider.setOrientation(desiredOrientation);
         headOrientationControlModule.setPointToTrack(desiredHeadOrientationProvider.getLookAtPoint());
         headOrientationControlModule.packDesiredFrameOrientation(desiredOrientation);
         desiredOrientation.changeFrame(headOrientationExpressedInFrame);
         finalOrientationProvider.setOrientation(desiredOrientation);
         receivedNewHeadOrientationTime.set(yoTime.getDoubleValue());
         orientationTrajectoryGenerator.initialize();
         isTrackingOrientation.set(true);
      }
      
   }

   public int createJacobian(String[] headOrientationControlJointNames)
   {
      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] headOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, headOrientationControlJointNames);

      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(headOrientationControlJoints, headOrientationControlModule.getHead()
            .getBodyFixedFrame());
      return jacobianId;
   }

   public void setUp(RigidBody base, int jacobianId)
   {
      this.jacobianId = jacobianId;
      headOrientationControlModule.setBase(base);
      headOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
   }

   public void setUp(RigidBody base, int jacobianId, double proportionalGainX, double proportionalGainY, double proportionalGainZ, double derivativeGainX,
         double derivativeGainY, double derivativeGainZ)
   {
      this.jacobianId = jacobianId;
      headOrientationControlModule.setBase(base);
      headOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
      headOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
      headOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      headOrientationControlModule.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void setControlGains(double proportionalGain, double derivativeGain)
   {
      headOrientationControlModule.setProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      headOrientationControlModule.setDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
   }

   public void turnOff()
   {
      setUp(null, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }
}
