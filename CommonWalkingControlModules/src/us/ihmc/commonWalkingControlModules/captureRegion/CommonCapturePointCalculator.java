package us.ihmc.commonWalkingControlModules.captureRegion;

import java.util.ArrayList;

import javax.vecmath.Vector2d;

import us.ihmc.CapturePointCalculator.LinearInvertedPendulumCapturePointCalculator;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.plotting.Artifact;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePointInMultipleFrames;



public class CommonCapturePointCalculator implements CapturePointCalculatorInterface
{
   public static final String CAPTURE_POINT_DYNAMIC_GRAPHIC_OBJECT_NAME = "Capture Point";
   
   private final ArrayList<Artifact> artifactsToRecordHistory = new ArrayList<Artifact>();

   private final boolean CHECK_COLLINEAR = false;

   private final YoFramePointInMultipleFrames capturePointInMultipleFrames;

   private final YoVariableRegistry registry = new YoVariableRegistry("CapturePoint");
   private final ProcessedSensorsInterface processedSensors;

   private final ReferenceFrame pelvisZUpFrame, midFeetZUpFrame;

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final DoubleYoVariable pointsCollinearAngle = new DoubleYoVariable("pointsCollinearAngle", registry);

   private final boolean useWorldFrame;

   public CommonCapturePointCalculator(ProcessedSensorsInterface processedSensors, CommonHumanoidReferenceFrames referenceFrames, boolean useWorldFrame,
           YoVariableRegistry yoVariableRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.processedSensors = processedSensors;

      this.pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      this.midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      this.useWorldFrame = useWorldFrame;

      ReferenceFrame[] referenceFramesForCapturePoint = {ReferenceFrame.getWorldFrame(), referenceFrames.getPelvisZUpFrame(),
            referenceFrames.getMidFeetZUpFrame(), referenceFrames.getAnkleZUpFrame(RobotSide.LEFT), referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT)};
      capturePointInMultipleFrames = new YoFramePointInMultipleFrames("capture", registry, referenceFramesForCapturePoint);

      if (yoVariableRegistry != null)
      {
         yoVariableRegistry.addChild(registry);
      }

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition capturePointWorldGraphicPosition = new YoGraphicPosition(CAPTURE_POINT_DYNAMIC_GRAPHIC_OBJECT_NAME, capturePointInMultipleFrames, 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
         YoGraphicsList yoGraphicsList = new YoGraphicsList("CapturePoint");
         yoGraphicsList.add(capturePointWorldGraphicPosition);

         ArtifactList artifactList = new ArtifactList("CapturePointCalculator");

         Artifact capturePointArtifact = capturePointWorldGraphicPosition.createArtifact();
         capturePointArtifact.setRecordHistory(true);
         capturePointArtifact.setDrawHistory(true);
         artifactsToRecordHistory.add(capturePointArtifact);
         artifactList.add(capturePointArtifact);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
   }

   public void computeCapturePoint(RobotSide supportLeg)
   {
      ReferenceFrame capturePointFrame = getFrameToComputeCapturePointIn(supportLeg);

      FrameVector comVelocity = processedSensors.getCenterOfMassVelocityInFrame(capturePointFrame);
      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(capturePointFrame);
      double gravity = -processedSensors.getGravityInWorldFrame().getZ();
      double comHeight = centerOfMassPosition.getZ();

      double captureX = LinearInvertedPendulumCapturePointCalculator.calculateCapturePoint(comVelocity.getX(), gravity, comHeight);
      double captureY = LinearInvertedPendulumCapturePointCalculator.calculateCapturePoint(comVelocity.getY(), gravity, comHeight);

      captureX += centerOfMassPosition.getX();
      captureY += centerOfMassPosition.getY();

      FramePoint capturePoint = new FramePoint(capturePointFrame, captureX, captureY, 0.0);

      capturePointInMultipleFrames.setIncludingFrame(capturePoint);
      
      takeArtifactHistorySnapshots();
   }
  

   public FramePoint computePredictedCapturePoint(RobotSide supportLeg, double captureTime, FramePoint centerOfPressure)
   {      
      ReferenceFrame capturePointFrame = getFrameToComputeCapturePointIn(supportLeg);
      
      centerOfPressure.changeFrame(capturePointFrame);

      FrameVector comVelocity = processedSensors.getCenterOfMassVelocityInFrame(capturePointFrame);
      FramePoint centerOfMassPosition = processedSensors.getCenterOfMassPositionInFrame(capturePointFrame);
      double gravity = -processedSensors.getGravityInWorldFrame().getZ();
      double comHeight = centerOfMassPosition.getZ();

      double xCoM = centerOfMassPosition.getX();
      double yCoM = centerOfMassPosition.getY();

      double predictedCaptureBodyZUpX = LinearInvertedPendulumCapturePointCalculator.calculatePredictedCapturePoint(xCoM, centerOfPressure.getX(),
                                           comVelocity.getX(), gravity, comHeight, 0.0, captureTime);
      double predictedCaptureBodyZUpY = LinearInvertedPendulumCapturePointCalculator.calculatePredictedCapturePoint(yCoM, centerOfPressure.getY(),
                                           comVelocity.getY(), gravity, comHeight, 0.0, captureTime);

      FramePoint2d predictedCapturePoint2d = new FramePoint2d(capturePointFrame, predictedCaptureBodyZUpX, predictedCaptureBodyZUpY);

      FramePoint predictedCapturePoint = new FramePoint(predictedCapturePoint2d.getReferenceFrame(),
            predictedCapturePoint2d.getX(), predictedCapturePoint2d.getY(), 0.0);    // laInBody.getZ());

      if (CHECK_COLLINEAR)
      {
         capturePointInMultipleFrames.changeFrame(pelvisZUpFrame);
         FramePoint capturePointBodyZUp = new FramePoint();
         capturePointInMultipleFrames.getFrameTupleIncludingFrame(capturePointBodyZUp);
         checkCoPCapturePredictedColinear(centerOfPressure, capturePointBodyZUp, predictedCapturePoint);
      }
      
      return predictedCapturePoint;
   }

   public FramePoint getCapturePointInFrame(ReferenceFrame referenceFrame)
   {
      FramePoint ret = new FramePoint();
      capturePointInMultipleFrames.changeFrame(referenceFrame);
      capturePointInMultipleFrames.getFrameTupleIncludingFrame(ret);
      return ret;
   }

   public FramePoint2d getCapturePoint2dInFrame(ReferenceFrame referenceFrame)
   {
      if (!referenceFrame.isZupFrame())
      {
         throw new RuntimeException("!referenceFrame.isZupFrame()");
      }

      FramePoint capturePoint = getCapturePointInFrame(referenceFrame);

      return new FramePoint2d(capturePoint.getReferenceFrame(), capturePoint.getX(), capturePoint.getY());
   }
   
   public FrameVector computeCapturePointVelocityInFrame(ReferenceFrame referenceFrame)
   {
      throw new RuntimeException("Not yet implemented");
   }
   
   private ReferenceFrame getFrameToComputeCapturePointIn(RobotSide supportLeg)
   {
      if (useWorldFrame)
         return ReferenceFrame.getWorldFrame();
      else
      {
         boolean doubleSupport = (supportLeg == null);
         
         ReferenceFrame capturePointFrame;
         if (doubleSupport)
         {
            capturePointFrame = midFeetZUpFrame;
         }
         else
         {
            capturePointFrame = ankleZUpFrames.get(supportLeg);
         }
         
         return capturePointFrame;         
      }
   }

   private void checkCoPCapturePredictedColinear(FramePoint CoPInBodyZUp, FramePoint captureInBodyZUp, FramePoint predictedCaptureInBodyZUp)
   {
      CoPInBodyZUp.checkReferenceFrameMatch(pelvisZUpFrame);
      captureInBodyZUp.checkReferenceFrameMatch(pelvisZUpFrame);
      predictedCaptureInBodyZUp.checkReferenceFrameMatch(pelvisZUpFrame);

      FrameVector CoPToCapture = new FrameVector(captureInBodyZUp);
      CoPToCapture.sub(CoPInBodyZUp);

      FrameVector captureToPredicted = new FrameVector(predictedCaptureInBodyZUp);
      captureToPredicted.sub(captureInBodyZUp);

      Vector2d CoPToCapture2d = new Vector2d(CoPToCapture.getX(), CoPToCapture.getY());
      Vector2d captureToPredicted2d = new Vector2d(captureToPredicted.getX(), captureToPredicted.getY());

      pointsCollinearAngle.set(CoPToCapture2d.angle(captureToPredicted2d));
   }

   
   private double timeOfLastSnapshot = Double.MIN_VALUE;
   private static final double snapshotTimeSpacing = 0.1;
   
   private void takeArtifactHistorySnapshots()
   {
      if (processedSensors.getTime() > timeOfLastSnapshot + snapshotTimeSpacing)
      {
         timeOfLastSnapshot = processedSensors.getTime();
         
         for (Artifact artifact : artifactsToRecordHistory)
         {
            artifact.takeHistorySnapshot();
         }
      }
   }
}
