package us.ihmc.simpleWholeBodyWalking.SimpleSphere;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
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
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
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
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double footLengthForControl = 0.2;
   private static final double toeWidthForControl = 0.15;
   private static final double footWidthForControl = 0.15;
   
   private final YoDouble omega;
   private final double gravity;
   
   private final YoFramePoint3D desiredCoMPosition;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D desiredCoMAcceleration;
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;
   private final YoFrameVector3D Force;
   private final YoFramePoint3D ECMPPosition;
   private final YoFramePoint3D CoPPosition;
   
   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;
   
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final List<YoFramePoseUsingYawPitchRoll> nextFootstepPoses = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> nextFootstepPolygons = new ArrayList<>();
   
   private final YoFramePoseUsingYawPitchRoll leftFootPose;
   private final YoFramePoseUsingYawPitchRoll rightFootPose;
   private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose;
   private final YoFramePoseUsingYawPitchRoll yoNextNextFootstepPose;
   private final YoFramePoseUsingYawPitchRoll yoNextNextNextFootstepPose;
   private final YoFrameConvexPolygon2D leftFoot;
   private final YoFrameConvexPolygon2D rightFoot;
   private final YoFrameConvexPolygon2D yoNextFootstepPolygon;
   private final YoFrameConvexPolygon2D yoNextNextFootstepPolygon;
   private final YoFrameConvexPolygon2D yoNextNextNextFootstepPolygon;
   
   private final SimpleBipedCoMTrajectoryPlanner dcmPlan;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final SimpleSphereRobot sphereRobot;
   
   public SimpleSphereVisualizer(SimpleBipedCoMTrajectoryPlanner comTrajectoryProvider, YoGraphicsListRegistry yoGraphicsListRegistry, 
                                 SimpleSphereRobot sphereRobot, YoVariableRegistry registry)
   {
      this.dcmPlan = comTrajectoryProvider;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      omega = new YoDouble("omega", registry);
      omega.set(sphereRobot.getOmega0());
      gravity = sphereRobot.getGravityZ();
      this.sphereRobot = sphereRobot;
      
      //Plot Desired COM, DCM, VRP
      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      YoGraphicPosition comViz = new YoGraphicPosition("desiredCoM", desiredCoMPosition, 0.02, YoAppearance.Black(), 
                                                       YoGraphicPosition.GraphicType.SOLID_BALL);
      //yoGraphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      
      desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
      YoGraphicPosition dcmViz = new YoGraphicPosition("desiredDCM", desiredDCMPosition, 0.02, YoAppearance.Yellow(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      //yoGraphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      
      desiredVRPPosition = new YoFramePoint3D("desiredVRPPosition", worldFrame, registry);
      YoGraphicPosition vrpViz = new YoGraphicPosition("desiredVRP", desiredVRPPosition, 0.02, YoAppearance.Purple(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      //yoGraphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());
      
      //Plot Force Vector
      ECMPPosition = new YoFramePoint3D("desiredECMPPosition", worldFrame, registry);
      CoPPosition = new YoFramePoint3D("desiredCoPPosition", worldFrame, registry);
      Force = new YoFrameVector3D("desiredForce", worldFrame, registry);
      YoGraphicVector forceVector = new YoGraphicVector("desiredForce", CoPPosition, Force, 0.1,YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("dcmPlanner", forceVector);
      
      
      desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector3D("desiredCoMAcceleration", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
      
      //set bag of balls
      dcmTrajectory = new BagOfBalls(100, 0.005, "dcmTrajectory", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      comTrajectory = new BagOfBalls(100, 0.005, "comTrajectory", YoAppearance.Black(), registry, yoGraphicsListRegistry);
      vrpTrajectory = new BagOfBalls(100, 0.005, "vrpTrajectory", YoAppearance.Green(), registry, yoGraphicsListRegistry);
      
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
      
      leftFootPose = new YoFramePoseUsingYawPitchRoll("leftFootPose", worldFrame, registry);
      rightFootPose = new YoFramePoseUsingYawPitchRoll("rightFootPose", worldFrame, registry);
      yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
      yoNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextFootstepPose", worldFrame, registry);
      yoNextNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextNextFootstepPose", worldFrame, registry);
      leftFoot = new YoFrameConvexPolygon2D("leftFoot", "", worldFrame, 4, registry);
      rightFoot = new YoFrameConvexPolygon2D("rightFoot", "", worldFrame, 4, registry);
      yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);
      yoNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextFootstep", "", worldFrame, 4, registry);
      yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextNextFootstep", "", worldFrame, 4, registry);
      
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
   
   
   // update Ball Loop every updateFrame ticks
   private int updateFrame = 20;
   private int updateCounter = 0;
   private final PoseReferenceFrame stepPoseFrame = new PoseReferenceFrame("stepPoseFrame", worldFrame);
   
   public void updateVizPoints(double currentTime, YoFrameVector3D controlForce)
   {
      desiredCoMPosition.set(dcmPlan.getDesiredCoMPosition());
      desiredCoMVelocity.set(dcmPlan.getDesiredCoMVelocity());
      desiredCoMAcceleration.set(dcmPlan.getDesiredCoMAcceleration());
      desiredDCMPosition.set(dcmPlan.getDesiredDCMPosition());
      desiredDCMVelocity.set(dcmPlan.getDesiredDCMVelocity());
      desiredVRPPosition.set(dcmPlan.getDesiredVRPPosition());
      
      
      
      //Calc real ecmp
      ECMPPosition.set(desiredVRPPosition);
      ECMPPosition.subZ(gravity / MathTools.square(omega.getDoubleValue()));

      //desiredCoPPosition.set(desiredECMPPosition);
      //desiredCoPPosition.setZ(0.0);

      //calculate scale for force to reach ground
      double scale = sphereRobot.getCenterOfMass().getZ() / controlForce.getZ();
      FrameVector3D COPtoCOM = new FrameVector3D();
      COPtoCOM.set(controlForce);
      COPtoCOM.scale(scale);
      CoPPosition.set(sphereRobot.getCenterOfMass());
      CoPPosition.sub(COPtoCOM);
      
      Force.set(controlForce);
      
      
      updateCounter++;
      if (updateCounter == updateFrame)
      {
         updateCounter = 0;
         dcmTrajectory.setBallLoop(desiredDCMPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);
         vrpTrajectory.setBallLoop(desiredVRPPosition);
      }
   }

   public void updateVizFeet(double currentTime, List<RobotSide> currentFeetInContact)
   {
      List<Footstep> footstepList= dcmPlan.getFootstepList();
      List<FootstepTiming> footstepTimingList= dcmPlan.getFootstepTimingList();
      
      if (footstepList.size() == 0)
         return;
      
      int stepIndex = 0;
      double step0SwingStartTime = footstepTimingList.get(stepIndex).getExecutionStartTime()+footstepTimingList.get(stepIndex).getSwingStartTime();
      double step0SwingEndTIme = step0SwingStartTime + footstepTimingList.get(stepIndex).getSwingTime();
      
      //check if update is needed
      if (currentTime > step0SwingStartTime )
      {
         Footstep step = footstepList.get(0);
         FootstepTiming timing = footstepTimingList.get(0);
         //if swing has finished, step(0) is a current foot, if not it is the next foot
         if (currentTime > step0SwingEndTIme)
         {
            stepIndex++;
            if (step.getRobotSide() == RobotSide.LEFT)
            {
               leftFootPose.set(step.getFootstepPose());
               stepPoseFrame.setPoseAndUpdate(step.getFootstepPose());
               FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
               tempPolygon.setReferenceFrame(stepPoseFrame);
               tempPolygon.set(footPolygon);
               tempPolygon.changeFrame(worldFrame);
               leftFoot.set(tempPolygon);
            }
            else
            {
               rightFootPose.set(step.getFootstepPose());
               stepPoseFrame.setPoseAndUpdate(step.getFootstepPose());
               FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
               tempPolygon.setReferenceFrame(stepPoseFrame);
               tempPolygon.set(footPolygon);
               tempPolygon.changeFrame(worldFrame);
               rightFoot.set(tempPolygon);
            }
            
         }
         
         for (int nextStepIndex = 0; nextStepIndex < nextFootstepPoses.size(); nextStepIndex++)
         {
            if (stepIndex < footstepList.size())
            {
               Footstep nextStep = footstepList.get(stepIndex);
               FootstepTiming nextTiming = footstepTimingList.get(stepIndex);
               
               nextFootstepPoses.get(nextStepIndex).set(nextStep.getFootstepPose());
               
               stepPoseFrame.setPoseAndUpdate(nextStep.getFootstepPose());
               FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
               tempPolygon.setReferenceFrame(stepPoseFrame);
               tempPolygon.set(footPolygon);
               tempPolygon.changeFrame(worldFrame);
               nextFootstepPolygons.get(nextStepIndex).set(tempPolygon);
               stepIndex++;          
            }
            else
            {
               nextFootstepPoses.get(nextStepIndex).setToNaN();
               nextFootstepPolygons.get(nextStepIndex).setToNaN();
            }
         }
         
         //leftFootPose.setToNaN();
         //rightFootPose.setToNaN();
         //leftFoot.clearAndUpdate();
         //rightFoot.clearAndUpdate();
 
      }
      
   }

}
