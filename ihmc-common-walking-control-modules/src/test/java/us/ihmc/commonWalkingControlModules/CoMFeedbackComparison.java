package us.ihmc.commonWalkingControlModules;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.YoICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPController;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.ICPProportionalController;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRMomentumController;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoMTrajectoryModelPredictiveController;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class CoMFeedbackComparison
{
   private static final double controlDt = 1e-5;
   private static final double gravity = -9.81;
   private static final double nominalHeight = 0.9;

   private static final double yOffsetBetweenThings = 0.5;

   private class TestController implements RobotController
   {
      private static final double graphicSize = 0.03;

      private final YoRegistry registry = new YoRegistry("testRegistry");

      private final YoICPControlGains icpGains;
      private final ICPProportionalController icpController;
      private final CoMTrajectoryPlanner basicTrajectoryPlanner;
      private final LQRMomentumController lqrController;
      private final CoMTrajectoryModelPredictiveController mpcController;

      private final YoBoolean restartTime = new YoBoolean("restartTimeTrigger", registry);
      private final YoDouble timeAtStart = new YoDouble("timeAtStart", registry);
      private final YoDouble timeInState = new YoDouble("timeInState", registry);
      private final YoDouble clock;

      private final YoFramePoint3D comPositionForICPController = new YoFramePoint3D("comPositionForICPController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint3D comPositionForLQRController = new YoFramePoint3D("comPositionForLQRController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint3D comPositionForMPCController = new YoFramePoint3D("comPositionForMPCController", ReferenceFrame.getWorldFrame(), registry);

      private final YoFrameVector3D comVelocityForICPController = new YoFrameVector3D("comVelocityForICPController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameVector3D comVelocityForLQRController = new YoFrameVector3D("comVelocityForLQRController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameVector3D comVelocityForMPCController = new YoFrameVector3D("comVelocityForMPCController", ReferenceFrame.getWorldFrame(), registry);

      private final YoFramePoint3D dcmPositionForICPController = new YoFramePoint3D("dcmPositionForICPController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint3D dcmPositionForLQRController = new YoFramePoint3D("dcmPositionForLQRController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint3D dcmPositionForMPCController = new YoFramePoint3D("dcmPositionForMPCController", ReferenceFrame.getWorldFrame(), registry);

      private final YoFrameVector3D comAccelerationFromICPController = new YoFrameVector3D("comAccelerationFromICPController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameVector3D comAccelerationFromLQRController = new YoFrameVector3D("comAccelerationFromLQRController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameVector3D comAccelerationFromMPCController = new YoFrameVector3D("comAccelerationFromMPCController", ReferenceFrame.getWorldFrame(), registry);

      private final YoFramePoint3D vrpPositionFromICPController = new YoFramePoint3D("vrpPositionFromICPController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint3D vrpPositionFromLQRController = new YoFramePoint3D("vrpPositionFromLQRController", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint3D vrpPositionFromMPCController = new YoFramePoint3D("vrpPositionFromMPCController", ReferenceFrame.getWorldFrame(), registry);


      private final List<ContactStateProvider> icpContacts;
      private final List<ContactStateProvider> lqrContacts;
      private final List<ContactPlaneProvider> mpcContacts;

      public TestController(YoDouble clock,
                            List<ContactStateProvider> icpContacts,
                            List<ContactStateProvider> lqrContacts,
                            List<ContactPlaneProvider> mpcContacts,
                            YoGraphicsListRegistry graphicsListRegistry)
      {
         this.clock = clock;
         this.icpContacts = icpContacts;
         this.lqrContacts = lqrContacts;
         this.mpcContacts = mpcContacts;

         icpGains = new YoICPControlGains("ICP", registry);
         icpGains.setKpParallelToMotion(2.0);
         icpGains.setKpOrthogonalToMotion(2.0);

         basicTrajectoryPlanner = new CoMTrajectoryPlanner(gravity, nominalHeight, registry);

         icpController = new ICPProportionalController(icpGains, controlDt, registry);
         lqrController = new LQRMomentumController(basicTrajectoryPlanner.getOmega(), registry);
         mpcController = new CoMTrajectoryModelPredictiveController(gravity, nominalHeight, controlDt, registry);

         restartTime.addListener(v ->
                                 {
                                    if (restartTime.getBooleanValue())
                                    {
                                       timeAtStart.set(clock.getValue());
                                       restartTime.set(false, false);
                                    }
                                 });

         YoGraphicPosition icpCoMViz = new YoGraphicPosition("icpCoMViz", comPositionForICPController, graphicSize, YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition lqrCoMViz = new YoGraphicPosition("lqrCoMViz", comPositionForLQRController, graphicSize, YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition mpcCoMViz = new YoGraphicPosition("mpcCoMViz", comPositionForMPCController, graphicSize, YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

         YoGraphicPosition icpDCMViz = new YoGraphicPosition("icpCoMViz", dcmPositionForICPController, graphicSize, YoAppearance.Yellow(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition lqrDCMViz = new YoGraphicPosition("lqrCoMViz", dcmPositionForLQRController, graphicSize, YoAppearance.Yellow(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition mpcDCMViz = new YoGraphicPosition("mpcCoMViz", dcmPositionForMPCController, graphicSize, YoAppearance.Yellow(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

         YoGraphicPosition icpVRPViz = new YoGraphicPosition("icpCoMViz", vrpPositionFromICPController, graphicSize, YoAppearance.Purple(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition lqrVRPViz = new YoGraphicPosition("lqrCoMViz", vrpPositionFromLQRController, graphicSize, YoAppearance.Purple(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition mpcVRPViz = new YoGraphicPosition("mpcCoMViz", vrpPositionFromMPCController, graphicSize, YoAppearance.Purple(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

         graphicsListRegistry.registerArtifact("viz", icpCoMViz.createArtifact());
         graphicsListRegistry.registerArtifact("viz", lqrCoMViz.createArtifact());
         graphicsListRegistry.registerArtifact("viz", mpcCoMViz.createArtifact());

         graphicsListRegistry.registerArtifact("viz", icpDCMViz.createArtifact());
         graphicsListRegistry.registerArtifact("viz", lqrDCMViz.createArtifact());
         graphicsListRegistry.registerArtifact("viz", mpcDCMViz.createArtifact());

         graphicsListRegistry.registerArtifact("viz", icpVRPViz.createArtifact());
         graphicsListRegistry.registerArtifact("viz", lqrVRPViz.createArtifact());
         graphicsListRegistry.registerArtifact("viz", mpcVRPViz.createArtifact());
      }

      @Override
      public void doControl()
      {
         comVelocityForICPController.scaleAdd(controlDt, comAccelerationFromICPController, comVelocityForICPController);
         comPositionForICPController.scaleAdd(controlDt, comVelocityForICPController, comPositionForICPController);

         comVelocityForLQRController.scaleAdd(controlDt, comAccelerationFromLQRController, comVelocityForLQRController);
         comPositionForLQRController.scaleAdd(controlDt, comVelocityForLQRController, comPositionForLQRController);

         comVelocityForMPCController.scaleAdd(controlDt, comAccelerationFromMPCController, comVelocityForMPCController);
         comPositionForMPCController.scaleAdd(controlDt, comVelocityForMPCController, comPositionForMPCController);

         double omega = Math.sqrt(gravity / nominalHeight);
         CapturePointTools.computeCapturePointPosition(comPositionForICPController, comVelocityForICPController, omega, dcmPositionForICPController);
         CapturePointTools.computeCapturePointPosition(comPositionForLQRController, comVelocityForLQRController, omega, dcmPositionForLQRController);
         CapturePointTools.computeCapturePointPosition(comPositionForMPCController, comVelocityForMPCController, omega, dcmPositionForMPCController);

         timeInState.set(clock.getValue() - timeAtStart.getDoubleValue());

         basicTrajectoryPlanner.solveForTrajectory(icpContacts);
         basicTrajectoryPlanner.compute(timeInState.getDoubleValue());

         vrpPositionFromICPController.set(icpController.doProportionalControl(dcmPositionForICPController,
                                                                                basicTrajectoryPlanner.getDesiredDCMPosition(),
                                                                                basicTrajectoryPlanner.getDesiredDCMVelocity(),
                                                                                omega));

         comAccelerationFromICPController.sub(comPositionForICPController, vrpPositionFromICPController);
         comAccelerationFromICPController.scale(omega * omega);

         basicTrajectoryPlanner.solveForTrajectory(lqrContacts);
         basicTrajectoryPlanner.compute(timeInState.getDoubleValue());

         lqrController.setVRPTrajectory(basicTrajectoryPlanner.getVRPTrajectories());
         lqrController.computeControlInput(comPositionForLQRController, comVelocityForLQRController, timeInState.getDoubleValue());

         vrpPositionFromLQRController.set(lqrController.getFeedbackVRPPosition());
         comAccelerationFromLQRController.sub(comPositionForLQRController, vrpPositionFromLQRController);
         comAccelerationFromLQRController.scale(omega * omega);

         mpcController.setCurrentCenterOfMassState(comPositionForMPCController, comVelocityForMPCController, timeInState.getDoubleValue());
         mpcController.solveForTrajectory(mpcContacts);
         comAccelerationFromMPCController.set(mpcController.getDesiredCoMAcceleration());
         vrpPositionFromMPCController.set(mpcController.getDesiredVRPPosition());
      }


      @Override
      public void initialize()
      {

      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return null;
      }
   }
}
