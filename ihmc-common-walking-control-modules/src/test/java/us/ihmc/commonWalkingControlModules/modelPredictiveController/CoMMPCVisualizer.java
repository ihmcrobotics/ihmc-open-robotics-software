package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.CenterOfMassFeedbackController;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoMMPCVisualizer
{
   private final CoMTrajectoryModelPredictiveController mpc;
   private final SimulationConstructionSet scs;

   private final YoFramePoint3D desiredVRP;
   private final YoFramePoint3D desiredDCM;
   private final YoFramePoint3D desiredCoM;

   private final YoFrameVector3D desiredCoMVelocity;

   public CoMMPCVisualizer(CoMTrajectoryModelPredictiveController mpc, SimulationConstructionSet scs,
                           YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.mpc = mpc;
      this.scs = scs;

      mpc.setCornerPointViewer(new CornerPointViewer(registry, graphicsListRegistry));
      mpc.setupCoMTrajectoryViewer(graphicsListRegistry);
      mpc.setContactPlaneViewers(() -> getNextContactPlaneForceViewer(registry, graphicsListRegistry));

      desiredVRP = new YoFramePoint3D("DesiredVRPVis", ReferenceFrame.getWorldFrame(), registry);
      desiredDCM = new YoFramePoint3D("DesiredDCMVis", ReferenceFrame.getWorldFrame(), registry);
      desiredCoM = new YoFramePoint3D("DesiredCoMVis", ReferenceFrame.getWorldFrame(), registry);
      desiredCoMVelocity = new YoFrameVector3D("DesiredCoMVelocityVis", ReferenceFrame.getWorldFrame(), registry);

      YoGraphicPosition vrpVis = new YoGraphicPosition("DesiredVRPVis", desiredVRP, 0.05, YoAppearance.Green());
      YoGraphicPosition dcmVis = new YoGraphicPosition("DesiredDCMVis", desiredDCM, 0.05, YoAppearance.Blue());
      YoGraphicPosition comVis = new YoGraphicPosition("DesiredCoMVis", desiredCoM, 0.05, YoAppearance.White());

      YoGraphicVector comVelocityVis = new YoGraphicVector("DesiredCoMVelocityVis", desiredCoM, desiredCoMVelocity, 0.05, YoAppearance.White());

      String name = "MPC Points";
      graphicsListRegistry.registerYoGraphic(name, vrpVis);
      graphicsListRegistry.registerYoGraphic(name, dcmVis);
      graphicsListRegistry.registerYoGraphic(name, comVis);
      graphicsListRegistry.registerYoGraphic(name, comVelocityVis);
   }

   int contactPlaneViewer = 0;
   private ContactPlaneForceViewer getNextContactPlaneForceViewer(YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      return new ContactPlaneForceViewer("plane" + contactPlaneViewer++, 6, registry, graphicsListRegistry);
   }

   public void visualize(double duration)
   {
      for (double time = 0.0; time <= duration; time += 0.005)
      {
         mpc.compute(time);

         desiredCoM.setMatchingFrame(mpc.getDesiredCoMPosition());
         desiredDCM.setMatchingFrame(mpc.getDesiredDCMPosition());
         desiredVRP.setMatchingFrame(mpc.getDesiredVRPPosition());

         desiredCoMVelocity.setMatchingFrame(mpc.getDesiredCoMVelocity());

         scs.tickAndUpdate();
      }
   }

}
