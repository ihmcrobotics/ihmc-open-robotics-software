package us.ihmc.darpaRoboticsChallenge.logProcessor;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class OverallDesiredCoPProcessor implements LogDataProcessorFunction
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final LogDataProcessorHelper logDataProcessorHelper;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final FramePoint2d tempCoP = new FramePoint2d();
   private final YoFramePoint2d admissibleDesiredCenterOfPressure = new YoFramePoint2d("admissibleDesiredCenterOfPressure", worldFrame, registry);
   private final YoFrameVector admissibleDesiredGroundReactionTorque;
   private final YoFrameVector admissibleDesiredGroundReactionForce;
   private final Wrench admissibleGroundReactionWrench = new Wrench();
   
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

   private final ReferenceFrame centerOfMassFrame;

   private final FrameVector tempVector = new FrameVector();
   
   public OverallDesiredCoPProcessor(LogDataProcessorHelper logDataProcessorHelper)
   {
      this.logDataProcessorHelper = logDataProcessorHelper;
      yoGraphicsListRegistry.registerArtifact("Desired Center of Pressure", new YoGraphicPosition("Desired Overall Center of Pressure", admissibleDesiredCenterOfPressure, 0.008, YoAppearance.Navy(), GraphicType.BALL).createArtifact());
      centerOfMassFrame = logDataProcessorHelper.getReferenceFrames().getCenterOfMassFrame();
      admissibleDesiredGroundReactionTorque = logDataProcessorHelper.findYoFrameVector("admissibleDesiredGroundReactionTorque", centerOfMassFrame);
      admissibleDesiredGroundReactionForce = logDataProcessorHelper.findYoFrameVector("admissibleDesiredGroundReactionForce", centerOfMassFrame);
   }

   @Override
   public void processDataAtControllerRate()
   {
      logDataProcessorHelper.update();

      admissibleGroundReactionWrench.setToZero(centerOfMassFrame, centerOfMassFrame);
      admissibleDesiredGroundReactionTorque.getFrameTupleIncludingFrame(tempVector);
      admissibleGroundReactionWrench.setAngularPart(tempVector);
      admissibleDesiredGroundReactionForce.getFrameTupleIncludingFrame(tempVector);
      admissibleGroundReactionWrench.setLinearPart(tempVector);
      
      centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(tempCoP, admissibleGroundReactionWrench, worldFrame);
      admissibleDesiredCenterOfPressure.set(tempCoP);
   }

   @Override
   public void processDataAtStateEstimatorRate()
   {
      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

}
