package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simpleWholeBodyWalking.SimpleBipedCoMTrajectoryPlanner;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SimpleSphereVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SphereVisualizer");
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double footLengthForControl = 0.2;
   private static final double toeWidthForControl = 0.15;
   private static final double footWidthForControl = 0.15;
   
   private final YoDouble omega = new YoDouble("omega", registry);
   private final double gravity;
   
   private final YoFramePoint3D desiredCoMPosition;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D desiredCoMAcceleration;
   private final YoFrameVector3D desiredForce;
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;
   private final YoFramePoint3D desiredECMPPosition;
   private final YoFramePoint3D desiredCoPPosition;
   
   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;
   
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final List<YoFramePoseUsingYawPitchRoll> nextFootstepPoses = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> nextFootstepPolygons = new ArrayList<>();
   
   private final YoFramePoseUsingYawPitchRoll leftFootPose = new YoFramePoseUsingYawPitchRoll("leftFootPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll rightFootPose = new YoFramePoseUsingYawPitchRoll("rightFootPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2D leftFoot = new YoFrameConvexPolygon2D("leftFoot", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D rightFoot = new YoFrameConvexPolygon2D("rightFoot", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextNextFootstep", "", worldFrame, 4, registry);
   
   private final SimpleBipedCoMTrajectoryPlanner dcmPlan;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   
   public SimpleSphereVisualizer(SimpleBipedCoMTrajectoryPlanner comTrajectoryProvider, YoGraphicsListRegistry yoGraphicsListRegistry, 
                                 DoubleProvider omega0Provider, double gravityZ)
   {
      this.dcmPlan = comTrajectoryProvider;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      omega.set(omega0Provider.getValue());
      gravity = gravityZ;
      
      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      YoGraphicPosition comViz = new YoGraphicPosition("desiredCoM", desiredCoMPosition, 0.02, YoAppearance.Black(), 
                                                       YoGraphicPosition.GraphicType.SOLID_BALL);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      
      desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
      YoGraphicPosition dcmViz = new YoGraphicPosition("desiredDCM", desiredDCMPosition, 0.02, YoAppearance.Yellow(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      
      desiredVRPPosition = new YoFramePoint3D("desiredVRPPosition", worldFrame, registry);
      YoGraphicPosition vrpViz = new YoGraphicPosition("desiredVRP", desiredVRPPosition, 0.02, YoAppearance.Purple(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());
      
      desiredECMPPosition = new YoFramePoint3D("desiredECMPPosition", worldFrame, registry);
      desiredForce = new YoFrameVector3D("desiredForce", worldFrame, registry);
      YoGraphicVector forceVector = new YoGraphicVector("desiredForce", desiredECMPPosition, desiredForce, 0.1,YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("dcmPlanner", forceVector);
      
      desiredCoPPosition = new YoFramePoint3D("desiredCoPPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector3D("desiredCoMAcceleration", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
      
      //set bag of balls
      dcmTrajectory = new BagOfBalls(100, 0.01, "dcmTrajectory", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      comTrajectory = new BagOfBalls(100, 0.01, "comTrajectory", YoAppearance.Black(), registry, yoGraphicsListRegistry);
      vrpTrajectory = new BagOfBalls(100, 0.01, "vrpTrajectory", YoAppearance.Green(), registry, yoGraphicsListRegistry);
      
      //Add Foot Visualations 
      List<Point2D> contactPointsInSoleFrame = new ArrayList<Point2D>();
      contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
      contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
      contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
      contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));

      contactPointsInSoleFrame.forEach(footPolygon::addVertex);
      footPolygon.update();
      
      Graphics3DObject footstepGraphics = new Graphics3DObject();
      Graphics3DObject stanceFootGraphics = new Graphics3DObject();
      footstepGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, YoAppearance.Color(Color.blue));
      stanceFootGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, YoAppearance.Color(Color.green));
      
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("leftFootPose", stanceFootGraphics, leftFootPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("rightFootPose", stanceFootGraphics, rightFootPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0));
      
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("leftFoot", leftFoot, Color.green, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("rightFoot", rightFoot, Color.green, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false));
      
      nextFootstepPoses.add(yoNextFootstepPose);
      nextFootstepPoses.add(yoNextNextFootstepPose);
      nextFootstepPoses.add(yoNextNextNextFootstepPose);

      nextFootstepPolygons.add(yoNextFootstepPolygon);
      nextFootstepPolygons.add(yoNextNextFootstepPolygon);
      nextFootstepPolygons.add(yoNextNextNextFootstepPolygon);
      
   }
   
   


   private final PoseReferenceFrame stepPoseFrame = new PoseReferenceFrame("stepPoseFrame", worldFrame);
   
   public void updateVizPoints(double currentTime, YoFrameVector3D controlForce)
   {
      desiredCoMPosition.set(dcmPlan.getDesiredCoMPosition());
      desiredCoMVelocity.set(dcmPlan.getDesiredCoMVelocity());
      desiredCoMAcceleration.set(dcmPlan.getDesiredCoMAcceleration());
      desiredDCMPosition.set(dcmPlan.getDesiredDCMPosition());
      desiredDCMVelocity.set(dcmPlan.getDesiredDCMVelocity());
      desiredVRPPosition.set(dcmPlan.getDesiredVRPPosition());
      
      desiredECMPPosition.set(desiredVRPPosition);
      desiredECMPPosition.subZ(gravity / MathTools.square(omega.getDoubleValue()));

      desiredCoPPosition.set(desiredECMPPosition);
      desiredCoPPosition.setZ(0.0);

      desiredForce.set(controlForce);
      
      dcmTrajectory.setBallLoop(desiredDCMPosition);
      comTrajectory.setBallLoop(desiredCoMPosition);
      vrpTrajectory.setBallLoop(desiredVRPPosition);
   }
   
   public void updateFeetStates(double currentTime)
   {
      List<RobotSide> feetInContact = dcmPlan.getFeetInContact();
      
      feetInContact.clear();
      for (RobotSide robotSide : RobotSide.values)
         feetInContact.add(robotSide);

      int stepNumber = 0;
      while (stepNumber < steps.size())
      {
         BipedTimedStep step = steps.get(stepNumber);
         if (currentTime > step.getTimeInterval().getEndTime() && steps.size() > 1)
         {
            steps.remove(stepNumber);
            stepsInProgress.remove(step);
            planner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);
         }
         else
         {
            stepNumber++;
         }

         if (currentTime > step.getTimeInterval().getStartTime() && !stepsInProgress.contains(step))
         {
            stepsInProgress.add(step);
            planner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);
         }

      }

      for (int i = 0; i < steps.size(); i++)
      {
         BipedTimedStep step = steps.get(i);

         if (step.getTimeInterval().intervalContains(currentTime))
         {
            soleFramesForModifying.get(step.getRobotSide()).updateTranslation(step.getGoalPose().getPosition());
            feetInContact.remove(step.getRobotSide());
         }
      }

      int nextStepIndex = 0;

      int stepIndex = 0;
      while (stepIndex < steps.size() && nextStepIndex < nextFootstepPoses.size() && nextStepIndex < nextFootstepPolygons.size())
      {
         BipedTimedStep step = steps.get(stepIndex);
         nextFootstepPoses.get(nextStepIndex).set(step.getGoalPose());

         stepPoseFrame.setPoseAndUpdate(step.getGoalPose());
         FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
         tempPolygon.setReferenceFrame(stepPoseFrame);
         tempPolygon.set(footPolygon);
         tempPolygon.changeFrame(worldFrame);
         nextFootstepPolygons.get(nextStepIndex).set(tempPolygon);

         stepIndex++;
         nextStepIndex++;
      }
      while (nextStepIndex < nextFootstepPoses.size() && nextStepIndex < nextFootstepPolygons.size())
      {
         nextFootstepPoses.get(nextStepIndex).setToNaN();
         nextFootstepPolygons.get(nextStepIndex).setToNaN();
         nextStepIndex++;
      }

      leftFootPose.setToNaN();
      rightFootPose.setToNaN();
      leftFoot.clearAndUpdate();
      rightFoot.clearAndUpdate();
      combinedFeet.clear();
      if (feetInContact.contains(RobotSide.LEFT))
      {
         FramePose3D pose = new FramePose3D(soleFramesForModifying.get(RobotSide.LEFT));
         pose.changeFrame(worldFrame);
         stepPoseFrame.setPoseAndUpdate(pose);
         FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
         tempPolygon.setReferenceFrame(stepPoseFrame);
         tempPolygon.set(footPolygon);
         tempPolygon.changeFrame(worldFrame);

         leftFoot.set(tempPolygon);
         leftFootPose.setMatchingFrame(pose);
         combinedFeet.addVertices(leftFoot);
      }
      if (feetInContact.contains(RobotSide.RIGHT))
      {
         FramePose3D pose = new FramePose3D(soleFramesForModifying.get(RobotSide.RIGHT));
         pose.changeFrame(worldFrame);
         stepPoseFrame.setPoseAndUpdate(pose);
         FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
         tempPolygon.setReferenceFrame(stepPoseFrame);
         tempPolygon.set(footPolygon);
         tempPolygon.changeFrame(worldFrame);

         rightFoot.set(tempPolygon);
         rightFootPose.setMatchingFrame(pose);
         combinedFeet.addVertices(rightFoot);
      }
      combinedFeet.update();
   }

   public void plotVRPTrajectory()
   {
      //dcmPlan.getVRPTrajectories()
      
   }

}
