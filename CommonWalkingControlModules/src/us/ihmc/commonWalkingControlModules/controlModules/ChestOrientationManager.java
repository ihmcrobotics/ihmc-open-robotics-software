package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameOrientationWaypoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.CurrentOrientationProvider;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.SimpleOrientationTrajectoryGenerator;


public class ChestOrientationManager
{
   private final YoVariableRegistry registry;
   private final ChestOrientationControlModule chestOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private int jacobianId = -1;

   private final ChestOrientationProvider chestOrientationProvider;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewChestOrientationTime;
   private final ReferenceFrame chestOrientationExpressedInFrame;

   private final BooleanYoVariable isTrackingOrientation;
   private final YoFrameVector yoControlledAngularAcceleration;

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator simpleOrientationTrajectoryGenerator;
   private final ReferenceFrame pelvisZUpFrame;

   private double defaultTrajectoryTime = 0;
   
   private final boolean USE_NEW_IMPLEMENTATION = true;

   public ChestOrientationManager(MomentumBasedController momentumBasedController, 
         ChestOrientationControlModule chestOrientationControlModule,
         ChestOrientationProvider chestOrientationProvider, 
         double trajectoryTime,
         YoVariableRegistry parentRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      this.yoTime = momentumBasedController.getYoTime();
      this.chestOrientationControlModule = chestOrientationControlModule;
      this.chestOrientationProvider = chestOrientationProvider;
      this.pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();
      
      defaultTrajectoryTime = trajectoryTime;

      if (chestOrientationProvider != null)
      {
         chestOrientationExpressedInFrame = chestOrientationProvider.getChestOrientationExpressedInFrame();

         registry = new YoVariableRegistry(getClass().getSimpleName());
         isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);
         receivedNewChestOrientationTime = new DoubleYoVariable("receivedNewChestOrientationTime", registry);

         ReferenceFrame chestCoMFrame = chestOrientationControlModule.getChest().getBodyFixedFrame();
         yoControlledAngularAcceleration = new YoFrameVector("controlledChestAngularAcceleration", chestCoMFrame, registry);

         if( USE_NEW_IMPLEMENTATION ) {
            CurrentOrientationProvider currentOrientation = new CurrentOrientationProvider( chestOrientationProvider.getChestOrientationExpressedInFrame() );
            
            orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("chest", currentOrientation, parentRegistry);  
            orientationTrajectoryGenerator.initialize();
            simpleOrientationTrajectoryGenerator = null;
         }
         else{
            simpleOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("chest", true, chestOrientationExpressedInFrame, parentRegistry);
            simpleOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
            simpleOrientationTrajectoryGenerator.registerAndSwitchFrame(pelvisZUpFrame);
            simpleOrientationTrajectoryGenerator.initialize();
            orientationTrajectoryGenerator = null;
         }

         parentRegistry.addChild(registry);
      }
      else {
         chestOrientationExpressedInFrame = null;
         registry = null;
         isTrackingOrientation = null;
         receivedNewChestOrientationTime = null;
         orientationTrajectoryGenerator = null;
         simpleOrientationTrajectoryGenerator = null;
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
         
         if( USE_NEW_IMPLEMENTATION )
         {
            orientationTrajectoryGenerator.compute(deltaTime);  
            orientationTrajectoryGenerator.get(desiredOrientation);
         }
         else{
            simpleOrientationTrajectoryGenerator.compute(deltaTime);
            simpleOrientationTrajectoryGenerator.get(desiredOrientation);
            
            if (simpleOrientationTrajectoryGenerator.isDone() )
            {
               simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
               isTrackingOrientation.set( false );
            }
         }    
         chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
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

      if( USE_NEW_IMPLEMENTATION ) 
      {
         if (chestOrientationProvider.checkForHomeOrientation())
         {
            FrameOrientationWaypoint pelvisHome = new FrameOrientationWaypoint(
                  defaultTrajectoryTime,
                  new FrameOrientation( pelvisZUpFrame ) );

            orientationTrajectoryGenerator.initializeTrajectory(pelvisHome);
            receivedNewChestOrientationTime.set( yoTime.getDoubleValue() );
            isTrackingOrientation.set(true);
         }
         else if (chestOrientationProvider.checkForNewChestOrientation())
         {
            //TODO for trajectories            
            receivedNewChestOrientationTime.set(yoTime.getDoubleValue());
            isTrackingOrientation.set(true);

            orientationTrajectoryGenerator.initializeTrajectory( chestOrientationProvider.getDesiredChestOrientations() );
         }
      }
      else{
         if (chestOrientationProvider.checkForHomeOrientation())
         {
            simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
            simpleOrientationTrajectoryGenerator.get(desiredOrientation);
            simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);
            desiredOrientation.setToZero(pelvisZUpFrame);
            simpleOrientationTrajectoryGenerator.setFinalOrientation(desiredOrientation);
            simpleOrientationTrajectoryGenerator.setTrajectoryTime( defaultTrajectoryTime );
            simpleOrientationTrajectoryGenerator.initialize();
            isTrackingOrientation.set(true);
         }
         else if (chestOrientationProvider.checkForNewChestOrientation())
         {
            receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

            simpleOrientationTrajectoryGenerator.changeFrame(chestOrientationProvider.getChestOrientationExpressedInFrame());
            simpleOrientationTrajectoryGenerator.get(desiredOrientation);
            simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);
            
            FrameOrientationWaypoint[] desiredChestOrientations = chestOrientationProvider.getDesiredChestOrientations();
            
            simpleOrientationTrajectoryGenerator.setFinalOrientation( desiredChestOrientations[0].orientation );
            simpleOrientationTrajectoryGenerator.setTrajectoryTime(  desiredChestOrientations[0].timeSincePreviousWaypoint );
            simpleOrientationTrajectoryGenerator.initialize();
            isTrackingOrientation.set(true);
         }
      }
   }  

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector desiredAngularAcceleration)
   {
      chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }

   public int createJacobian(FullRobotModel fullRobotModel, String[] chestOrientationControlJointNames)
   {
      RigidBody rootBody = fullRobotModel.getRootJoint().getSuccessor();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      ReferenceFrame chestFrame = chestOrientationControlModule.getChest().getBodyFixedFrame();
      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chestOrientationControlJoints, chestFrame);
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
