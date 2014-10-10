package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.FootSpoof;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.humanoidRobot.footstep.footsepGenerator.FootstepGenerator;
import us.ihmc.yoUtilities.humanoidRobot.footstep.footsepGenerator.SimplePathParameters;
import us.ihmc.yoUtilities.humanoidRobot.footstep.footsepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
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
   private final BooleanYoVariable hasFootstepsBeenGenerated = new BooleanYoVariable("hasFootstepsBeenGenerated", registry);
  
   private final Point3d targetLocation = new Point3d();
   private final YoFrameOrientation targetOrientation = new YoFrameOrientation("targetOrientation", ReferenceFrame.getWorldFrame(), registry);

   SimplePathParameters pathType = new SimplePathParameters(0.4, 0.2, 0.0, Math.PI * 0.8, Math.PI * 0.15, 0.35);

   private FootstepGenerator footstepGenerator;

   private ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
   private FootstepListBehavior footstepListBehavior;

   double xToAnkle = -0.15;
   double yToAnkle = 0.02;
   double zToAnkle = 0.21;
   double footForward = 0.1;
   double footBack = 0.05;
   double footSide = 0.05;
   double coefficientOfFriction = 0.0;

   ContactablePlaneBody leftFoot = new FootSpoof("leftFoot", xToAnkle, yToAnkle, zToAnkle, footForward, footBack, footSide, coefficientOfFriction);
   ContactablePlaneBody rightFoot = new FootSpoof("rightFoot", xToAnkle, yToAnkle, zToAnkle, footForward, footBack, footSide, coefficientOfFriction);
   SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();

   {
      bipedFeet.set(RobotSide.LEFT, leftFoot);
      bipedFeet.set(RobotSide.RIGHT, rightFoot);
   }

   public WalkToLocationBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         ReferenceFrames referenceFrames)
   {
      super(outgoingCommunicationBridge);

      this.targetLocation.set(robotLocation);
      this.targetOrientation.set(robotOrientation);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
   }

   public void setTarget(Point3d targetLocation, YoFrameOrientation targetOrientation)//(Point3d targetLocation, YoFrameOrientation targetOrientation)   //(YoFramePoint targetLocation, YoFrameOrientation targetOrientation)
   {
      this.targetLocation.set(targetLocation);
      this.targetOrientation.set(targetOrientation);
      hasTargetBeenProvided.set(true);
   }

   @Override
   public void initialize()
   {
      hasTargetBeenProvided.set(false);
      hasFootstepsBeenGenerated.set(false);
      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge);

      robotPose.setToZero(fullRobotModel.getRootJoint().getFrameAfterJoint());
      robotPose.changeFrame(worldFrame);

      robotYoPose.set(robotPose);

      robotPose.getPosition(robotLocation);
      robotPose.getOrientation(robotOrientation);
   }

   private void generateFootsteps()
   {
      FramePose2d endPose = new FramePose2d(worldFrame);
      endPose.setPosition(new FramePoint2d(worldFrame, targetLocation.getX(), targetLocation.getY()));
      endPose.setOrientation(new FrameOrientation2d(worldFrame, targetOrientation.getYaw().getDoubleValue()));

      footstepGenerator = new TurnStraightTurnFootstepGenerator(bipedFeet, endPose, pathType, RobotSide.LEFT);
      footsteps.addAll(footstepGenerator.generateDesiredFootstepList());

      FramePoint midFeetPosition = new FramePoint(referenceFrames.getMidFeetZUpFrame());
      midFeetPosition.changeFrame(worldFrame);

      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         footstep.setZ(midFeetPosition.getZ());
      }
      footstepListBehavior.set(footsteps, bipedFeet);
      hasFootstepsBeenGenerated.set(true);
   }

   @Override
   public void doControl()
   {
      if (!hasTargetBeenProvided.getBooleanValue())
         return;
      if (!hasFootstepsBeenGenerated.getBooleanValue())
         generateFootsteps();
      footstepListBehavior.doControl();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      footstepListBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      footstepListBehavior.consumeObjectFromController(object);

   }

   @Override
   public void stop()
   {
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
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);

   }

   @Override
   public boolean isDone()
   {
      if (!hasFootstepsBeenGenerated.getBooleanValue() || !hasTargetBeenProvided.getBooleanValue())
         return false;
      return footstepListBehavior.isDone();
   }

   @Override
   public void finalize()
   {
      isPaused.set(false);
      isStopped.set(false);
      hasTargetBeenProvided.set(false);
      hasFootstepsBeenGenerated.set(false);
   }

}
