package us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.SE3ModelPredictiveController;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SE3MPCVisualizer
{
   private final SE3ModelPredictiveController mpc;
   private final SimulationConstructionSet scs;

   private final YoFramePoint3D desiredVRP;
   private final YoFramePoint3D desiredECMP;
   private final YoFramePoint3D desiredDCM;
   private final YoFramePoint3D desiredCoM;
   private final YoFrameYawPitchRoll desiredBodyOrientation;
   private final YoFrameYawPitchRoll desiredBodyOrientationFeedForward;
   private final YoFrameVector3D desiredBodyAngularVelocity;
   private final YoFrameVector3D desiredForce;
   private final YoFrameVector3D desiredBodyAngularVelocityFeedForward;
   private final YoFramePoseUsingYawPitchRoll desiredBodyPose;

   private final YoFrameVector3D desiredCoMVelocity;

   private final SideDependentList<ContactPlaneForceViewer> forceViewers = new SideDependentList<>();
   private final double mass;
   private final double gravity;

   public SE3MPCVisualizer(SE3ModelPredictiveController mpc, SimulationConstructionSet scs,
                           double mass, double gravity,
                           YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.mpc = mpc;
      this.scs = scs;
      this.mass = mass;
      this.gravity = gravity;

      mpc.setCornerPointViewer(new MPCCornerPointViewer(registry, graphicsListRegistry));
      mpc.setupCoMTrajectoryViewer(graphicsListRegistry);
      mpc.setContactPlaneViewers(() -> getNextContactPlaneForceViewer(registry, graphicsListRegistry));

      desiredVRP = new YoFramePoint3D("DesiredVRPVis", ReferenceFrame.getWorldFrame(), registry);
      desiredECMP = new YoFramePoint3D("DesiredECMPVis", ReferenceFrame.getWorldFrame(), registry);
      desiredDCM = new YoFramePoint3D("DesiredDCMVis", ReferenceFrame.getWorldFrame(), registry);
      desiredCoM = new YoFramePoint3D("DesiredCoMVis", ReferenceFrame.getWorldFrame(), registry);
      desiredCoMVelocity = new YoFrameVector3D("DesiredCoMVelocityVis", ReferenceFrame.getWorldFrame(), registry);

      desiredBodyOrientation = new YoFrameYawPitchRoll("DesiredBodyOrientation", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyOrientationFeedForward = new YoFrameYawPitchRoll("DesiredBodyOrientationFeedForward", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyAngularVelocity = new YoFrameVector3D("DesiredBodyAngularVelocity", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyAngularVelocityFeedForward = new YoFrameVector3D("DesiredBodyAngularVelocityFeedForward", ReferenceFrame.getWorldFrame(), registry);

      desiredBodyPose = new YoFramePoseUsingYawPitchRoll(desiredCoM, desiredBodyOrientation);
      desiredForce = new YoFrameVector3D("DesiredForce", ReferenceFrame.getWorldFrame(), registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         forceViewers.put(robotSide, new ContactPlaneForceViewer(robotSide.getCamelCaseName(), 4, registry, graphicsListRegistry));
      }

      YoGraphicPosition vrpVis = new YoGraphicPosition("DesiredVRPVis", desiredVRP, 0.025, YoAppearance.Green());
      YoGraphicPosition dcmVis = new YoGraphicPosition("DesiredDCMVis", desiredDCM, 0.025, YoAppearance.Yellow());
      YoGraphicPosition comVis = new YoGraphicPosition("DesiredCoMVis", desiredCoM, 0.025, YoAppearance.Black());
      YoGraphicVector forceViz = new YoGraphicVector("DesiredNetForceVis", desiredECMP, desiredForce, 0.01, YoAppearance.Red());
      AppearanceDefinition bodyVisAppearance = YoAppearance.Green();
      bodyVisAppearance.setTransparency(0.5);
      YoGraphicEllipsoid bodyVis = new YoGraphicEllipsoid("DesiredBodyVis", desiredBodyPose, bodyVisAppearance, new Vector3D(0.05, 0.05, 0.1));

      YoGraphicVector comVelocityVis = new YoGraphicVector("DesiredCoMVelocityVis", desiredCoM, desiredCoMVelocity, 0.05, YoAppearance.White());

      String name = "MPC Points";
      graphicsListRegistry.registerYoGraphic(name, vrpVis);
      graphicsListRegistry.registerYoGraphic(name, dcmVis);
      graphicsListRegistry.registerYoGraphic(name, comVis);
      graphicsListRegistry.registerYoGraphic(name, comVelocityVis);
      graphicsListRegistry.registerYoGraphic(name, bodyVis);
      graphicsListRegistry.registerYoGraphic(name, forceViz);
   }

   int contactPlaneViewer = 0;
   private ContactPlaneForceViewer getNextContactPlaneForceViewer(YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      return new ContactPlaneForceViewer("plane" + contactPlaneViewer++, 4, registry, graphicsListRegistry);
   }

   public void visualize(double startTime, double durationToVisualize)
   {
      for (double time = startTime; time <= startTime + durationToVisualize; time += 0.005)
      {
         mpc.compute(time);
         int currentSegment = mpc.getCurrentSegmentIndex();

         for (int i = 0; i < currentSegment; i++)
         {
            for (int j = 0; j < mpc.contactPlaneHelperPool.get(i).size(); j++)
               mpc.contactPlaneHelperPool.get(i).get(j).clearViz();
         }
         if (currentSegment < mpc.contactPlaneHelperPool.size())
         {
            for (int i = 0; i < mpc.contactPlaneHelperPool.get(currentSegment).size(); i++)
               mpc.contactPlaneHelperPool.get(currentSegment).get(i).computeContactForce(3.0, time);
         }


         desiredCoM.setMatchingFrame(mpc.getDesiredCoMPosition());
         desiredDCM.setMatchingFrame(mpc.getDesiredDCMPosition());
         desiredVRP.setMatchingFrame(mpc.getDesiredVRPPosition());
         desiredECMP.setMatchingFrame(mpc.getDesiredECMPPosition());

         desiredForce.setMatchingFrame(mpc.getDesiredCoMAcceleration());
         desiredForce.addZ(Math.abs(gravity));
         desiredForce.scale(mass);

         desiredCoMVelocity.setMatchingFrame(mpc.getDesiredCoMVelocity());

         desiredBodyOrientation.setMatchingFrame(mpc.getDesiredBodyOrientationSolution());
         desiredBodyOrientationFeedForward.setMatchingFrame(mpc.getDesiredFeedForwardBodyOrientation());
         desiredBodyAngularVelocity.setMatchingFrame(mpc.getDesiredBodyAngularVelocitySolution());
         desiredBodyAngularVelocityFeedForward.setMatchingFrame(mpc.getDesiredFeedForwardBodyAngularVelocity());

         scs.setTime(time);
         scs.tickAndUpdate();
      }
   }

}
