package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoQuaternionProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;


public class ChestOrientationManager
{
   private final YoVariableRegistry registry;
   private final ChestOrientationControlModule chestOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private int jacobianId = -1;

   private final ChestOrientationProvider chestOrientationProvider;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewChestOrientationTime;
   private final DoubleYoVariable chestOrientationTrajectoryTime;
   private final OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator;
   private final YoQuaternionProvider initialOrientationProvider;
   private final YoQuaternionProvider finalOrientationProvider;
   private final ReferenceFrame chestOrientationExpressedInFrame;

   private final BooleanYoVariable isTrackingOrientation;
   private final YoFrameVector yoControlledAngularAcceleration;
   
   public ChestOrientationManager(MomentumBasedController momentumBasedController, ChestOrientationControlModule chestOrientationControlModule,
         ChestOrientationProvider chestOrientationProvider, double trajectoryTime, YoVariableRegistry parentRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      this.yoTime = momentumBasedController.getYoTime();
      this.chestOrientationControlModule = chestOrientationControlModule;
      this.chestOrientationProvider = chestOrientationProvider;

      if (chestOrientationProvider != null)
      {
         chestOrientationExpressedInFrame = chestOrientationProvider.getChestOrientationExpressedInFrame();

         registry = new YoVariableRegistry(getClass().getSimpleName());
         isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);
         receivedNewChestOrientationTime = new DoubleYoVariable("receivedNewChestOrientationTime", registry);
         chestOrientationTrajectoryTime = new DoubleYoVariable("chestOrientationTrajectoryTime", registry);

         chestOrientationTrajectoryTime.set(trajectoryTime);
         DoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider(chestOrientationTrajectoryTime);
         initialOrientationProvider = new YoQuaternionProvider("chestInitialOrientation", chestOrientationExpressedInFrame, registry);
         finalOrientationProvider = new YoQuaternionProvider("chestFinalOrientation", chestOrientationExpressedInFrame, registry);
         orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("chestOrientation", chestOrientationExpressedInFrame,
               trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);
         orientationTrajectoryGenerator.setContinuouslyUpdateFinalOrientation(true);
         orientationTrajectoryGenerator.initialize();
         
         ReferenceFrame chestCoMFrame = chestOrientationControlModule.getChest().getBodyFixedFrame();
         yoControlledAngularAcceleration = new YoFrameVector("controlledChestAngularAcceleration", chestCoMFrame, registry);

         parentRegistry.addChild(registry);
      }
      else
      {
         chestOrientationExpressedInFrame = null;
         registry = null;
         isTrackingOrientation = null;
         receivedNewChestOrientationTime = null;
         chestOrientationTrajectoryTime = null;
         initialOrientationProvider = null;
         finalOrientationProvider = null;
         orientationTrajectoryGenerator = null;
         yoControlledAngularAcceleration = null;
      }
      
   }

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector feedForwardAngularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

   private final FrameVector controlledAngularAcceleration = new FrameVector();
   
   public void compute()
   {
      checkForNewDesiredOrientationInformation();

      if (chestOrientationProvider != null && isTrackingOrientation.getBooleanValue())
      {
         double deltaTime = yoTime.getDoubleValue() - receivedNewChestOrientationTime.getDoubleValue();
         orientationTrajectoryGenerator.compute(deltaTime);
         orientationTrajectoryGenerator.get(desiredOrientation);
         chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
         isTrackingOrientation.set(!orientationTrajectoryGenerator.isDone());
      }

      if (jacobianId >= 0)
      {
         chestOrientationControlModule.compute();
         TaskspaceConstraintData taskspaceConstraintData = chestOrientationControlModule.getTaskspaceConstraintData();
         
         if (yoControlledAngularAcceleration != null)
         {
            SpatialAccelerationVector spatialAcceleration = taskspaceConstraintData.getSpatialAcceleration();
            if (spatialAcceleration.getExpressedInFrame() != null) // That happens when there is no joint to control.
            {
               spatialAcceleration.packAngularPart(controlledAngularAcceleration);
               yoControlledAngularAcceleration.set(controlledAngularAcceleration);
            }
         }

         momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
      }
   }

   private void checkForNewDesiredOrientationInformation()
   {
      if (chestOrientationProvider == null)
         return;

      if (chestOrientationProvider.isNewChestOrientationInformationAvailable())
      {
         orientationTrajectoryGenerator.get(desiredOrientation);
         initialOrientationProvider.setOrientation(desiredOrientation);
         finalOrientationProvider.setOrientation(chestOrientationProvider.getDesiredChestOrientation());
         chestOrientationTrajectoryTime.set(chestOrientationProvider.getTrajectoryTime());
         receivedNewChestOrientationTime.set(yoTime.getDoubleValue());
         orientationTrajectoryGenerator.initialize();
         isTrackingOrientation.set(true);
      }
   }  

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector desiredAngularAcceleration)
   {
      chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }

   public int createJacobian(FullRobotModel fullRobotModel, String[] chestOrientationControlJointNames)
   {
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chestOrientationControlJoints, chestOrientationControlModule.getChest()
            .getBodyFixedFrame());
      return jacobianId;
   }

   public void setUp(RigidBody base, int jacobianId)
   {
      this.jacobianId = jacobianId;
      chestOrientationControlModule.setBase(base);
      chestOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
   }

   public void setUp(RigidBody base, int jacobianId, double proportionalGainX, double proportionalGainY, double proportionalGainZ, double derivativeGainX,
         double derivativeGainY, double derivativeGainZ)
   {
      this.jacobianId = jacobianId;
      chestOrientationControlModule.setBase(base);
      chestOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
      chestOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
      chestOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setControlGains(double proportionalGain, double derivativeGain)
   {
      chestOrientationControlModule.setProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      chestOrientationControlModule.setDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      chestOrientationControlModule.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void turnOff()
   {
      setUp(null, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }
}
