package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.pathGeneration.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.utilities.humanoidRobot.footstep.footsepGenerator.SimplePathParameters;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class WalkToLocationBehavior extends BehaviorInterface
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullRobotModel fullRobotModel;
   private final ReferenceFrames referenceFrames;

   private final FramePose robotPose = new FramePose();
   private final Point3d robotLocation = new Point3d();
   private final Quat4d robotOrientation = new Quat4d();

   private final YoFramePose robotYoPose = new YoFramePose("robotYoPose", worldFrame, registry);

   private final BooleanYoVariable hasTargetBeenProvided = new BooleanYoVariable("hasTargetBeenProvided", registry);
   private final BooleanYoVariable haveFootstepsBeenGenerated = new BooleanYoVariable("haveFootstepsBeenGenerated", registry);

   private final YoFramePoint targetLocation = new YoFramePoint(getName() + "TargetLocation", worldFrame, registry);
   private final YoFrameOrientation targetOrientation = new YoFrameOrientation(getName() + "TargetOrientation", worldFrame, registry);

   private SimplePathParameters pathType;// = new SimplePathParameters(0.4, 0.30, 0.0, Math.toRadians(10.0), Math.toRadians(5.0), 0.4);

   private TurnStraightTurnFootstepGenerator footstepGenerator;

   private ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
   private FootstepListBehavior footstepListBehavior;

   private final SideDependentList<RigidBody> feet = new SideDependentList<RigidBody>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>();
   
   private double minDistanceThresholdForWalking, minYawThresholdForWalking;

   public WalkToLocationBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);

//      this.targetLocation.set(robotLocation);
//      this.targetOrientation.set(robotOrientation);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.pathType = new SimplePathParameters(walkingControllerParameters.getMaxStepLength(), walkingControllerParameters.getInPlaceWidth(), 0.0,
            Math.toRadians(20.0), Math.toRadians(10.0), 0.4); // 10 5 0.4
      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge);

      for (RobotSide robotSide : RobotSide.values)
      {
         feet.put(robotSide, fullRobotModel.getFoot(robotSide));
         soleFrames.put(robotSide, fullRobotModel.getSoleFrame(robotSide));
      }
   }

   public WalkToLocationBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames, double walkingYawOrientationAngle, WalkingControllerParameters walkingControllerParameters)
   {
      this(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      this.pathType = new SimplePathParameters(walkingControllerParameters.getMaxStepLength(), walkingControllerParameters.getInPlaceWidth(),
            walkingYawOrientationAngle, Math.toRadians(10.0), Math.toRadians(5.0), 0.4);
   }

   public void setTarget(Point3d targetLocation, YoFrameOrientation targetOrientation)
   {
      this.targetLocation.set(targetLocation);
      this.targetOrientation.set(targetOrientation);
      hasTargetBeenProvided.set(true);
      generateFootsteps();
   }

   public void setTarget(FramePose2d targetPose2dInWorld)
   {
      targetPose2dInWorld.checkReferenceFrameMatch(worldFrame);
      this.targetLocation.set(targetPose2dInWorld.getX(), targetPose2dInWorld.getY(), 0.0);
      this.targetOrientation.setYawPitchRoll(targetPose2dInWorld.getYaw(), 0.0, 0.0);
      hasTargetBeenProvided.set(true);
      generateFootsteps();
   }

   public void setwalkingYawOrientationAngle(double walkingYawOrientationAngle)
   {
      pathType.setAngle(walkingYawOrientationAngle);
   }

   @Override
   public void initialize()
   {
      hasTargetBeenProvided.set(false);
      haveFootstepsBeenGenerated.set(false);
      footstepListBehavior.initialize();

      robotPose.setToZero(fullRobotModel.getRootJoint().getFrameAfterJoint());
      robotPose.changeFrame(worldFrame);

      robotYoPose.set(robotPose);

      robotPose.getPosition(robotLocation);
      robotPose.getOrientation(robotOrientation);
      
    this.targetLocation.set(robotLocation);
    this.targetOrientation.set(robotOrientation);

      //for testing purpose
      //this.setTarget(new Point3d(2.0, 2.0,0.0),new YoFrameOrientation( "blabla", ReferenceFrame.getWorldFrame(), registry));
   }

   private void generateFootsteps()
   {
      footsteps.clear();
      FramePose2d endPose = new FramePose2d(worldFrame);
      endPose.setPosition(new FramePoint2d(worldFrame, targetLocation.getX(), targetLocation.getY()));
      endPose.setOrientation(new FrameOrientation2d(worldFrame, targetOrientation.getYaw().getDoubleValue()));
            
      footstepGenerator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, endPose, pathType);
      footstepGenerator.initialize();
      
      if(footstepGenerator.getDistance() > minDistanceThresholdForWalking || Math.abs(footstepGenerator.getSignedInitialTurnDirection()) > minYawThresholdForWalking) 
      {
         footsteps.addAll(footstepGenerator.generateDesiredFootstepList());

         FramePoint midFeetPosition = new FramePoint(referenceFrames.getMidFeetZUpFrame());
         midFeetPosition.changeFrame(worldFrame);

         for (int i = 0; i < footsteps.size(); i++)
         {
            Footstep footstep = footsteps.get(i);
            footstep.setZ(midFeetPosition.getZ());
         }         
      }

      footstepListBehavior.set(footsteps);
      haveFootstepsBeenGenerated.set(true);
   }

   @Override
   public void doControl()
   {
      if (!hasTargetBeenProvided.getBooleanValue())
         return;
      if (!haveFootstepsBeenGenerated.getBooleanValue())
         generateFootsteps();
      footstepListBehavior.doControl();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      if (footstepListBehavior != null)
         footstepListBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      if (footstepListBehavior != null)
         footstepListBehavior.consumeObjectFromController(object);
   }

   @Override
   public void stop()
   {
      footstepListBehavior.stop();
      isStopped.set(true);
   }

   @Override
   public void enableActions()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void pause()
   {
      footstepListBehavior.pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      footstepListBehavior.resume();
      isPaused.set(false);

   }

   @Override
   public boolean isDone()
   {
      if (!haveFootstepsBeenGenerated.getBooleanValue() || !hasTargetBeenProvided.getBooleanValue())
         return false;
      if (haveFootstepsBeenGenerated.getBooleanValue() && footsteps.size() == 0)
         return true;
      return footstepListBehavior.isDone();
   }

   @Override
   public void finalize()
   {
      isPaused.set(false);
      isStopped.set(false);
      hasTargetBeenProvided.set(false);
      haveFootstepsBeenGenerated.set(false);
      footstepListBehavior.finalize();
   }

   public boolean hasInputBeenSet()
   {
      if (haveFootstepsBeenGenerated.getBooleanValue())
         return true;
      else
         return false;
   }

   public void setFootstepLength(double footstepLength)
   {
      pathType.setStepLength(footstepLength);
   }
   
   public void setDistanceThreshold(double minDistanceThresholdForWalking) 
   {
      this.minDistanceThresholdForWalking = minDistanceThresholdForWalking;      
   }
   
   public void setYawAngleThreshold(double minYawThresholdForWalking)
   {
      this.minYawThresholdForWalking = minYawThresholdForWalking;
   }

}
