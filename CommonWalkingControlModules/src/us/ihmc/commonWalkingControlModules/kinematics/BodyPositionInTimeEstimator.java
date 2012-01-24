package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.CapturePointCalculator.LinearInvertedPendulumCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class BodyPositionInTimeEstimator
{
   private final ProcessedSensorsInterface processedSensors;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredHeadingFrame;
   private final CouplingRegistry couplingRegistry;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("BodyPositionInTimeEstimator");
   private final DoubleYoVariable currentCoMHeight = new DoubleYoVariable("currentCoMHeight", registry);
   
   public BodyPositionInTimeEstimator(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames,
         DesiredHeadingControlModule desiredHeadingControlModule,
         CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.pelvisFrame = referenceFrames.getPelvisFrame();
      this.desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      this.couplingRegistry = couplingRegistry;
      referenceFrames.getABodyAttachedZUpFrame();
      
      referenceFrames.getFootReferenceFrames();
      referenceFrames.getAnkleZUpReferenceFrames();
      parentRegistry.addChild(registry);
      
   }
   
   
   public Pair<FramePose, FrameVector> getPelvisPoseAndVelocityInTime(double t, RobotSide swingFoot)
   {
      FramePoint2d currentCoPPosition = couplingRegistry.getDesiredCoP();
      currentCoPPosition.changeFrame(desiredHeadingFrame);
      FramePoint currentCoMPosition = processedSensors.getCenterOfMassPositionInFrame(desiredHeadingFrame);
      FrameVector currentCoMVelocity = processedSensors.getCenterOfMassVelocityInFrame(desiredHeadingFrame);
      currentCoMVelocity.changeFrame(desiredHeadingFrame);
      
      FramePoint bodyPosition = new FramePoint(pelvisFrame);
      bodyPosition.changeFrame(desiredHeadingFrame);
      
      
      // LIPM Model in time
      FramePoint currentCoMPositionInStanceFoot = currentCoMPosition.changeFrameCopy(ReferenceFrame.getWorldFrame());
      double comHeight = currentCoMPositionInStanceFoot.getZ();
      currentCoMHeight.set(comHeight);
      
      double gravity = Math.abs(processedSensors.getGravityInWorldFrame().getZ());
      
      double[] xCoMInTime = LinearInvertedPendulumCapturePointCalculator.calculatePredictedCoMState(
                           currentCoMPosition.getX(), currentCoPPosition.getX(), currentCoMVelocity.getX(), 
                           gravity, comHeight, 0.0, t);
      double[] yCoMInTime = LinearInvertedPendulumCapturePointCalculator.calculatePredictedCoMState(
                           currentCoMPosition.getY(), currentCoPPosition.getY(), currentCoMVelocity.getY(), 
                           gravity, comHeight, 0.0, t);
      
      double xDeltaInTime = xCoMInTime[0] - currentCoMPosition.getX();
//      double yDeltaInTime = yCoMInTime[0] - currentCoMPosition.getY();
      bodyPosition.setX(bodyPosition.getX() + xDeltaInTime);
//      bodyPosition.setY(bodyPosition.getY() + yDeltaInTime);
      bodyPosition.changeFrame(pelvisFrame);
      
      FramePose pelvisPoseInTime = new FramePose(pelvisFrame);
      pelvisPoseInTime.setPosition(bodyPosition);
      
      
      FrameVector pelvisVelocityInTime = new FrameVector(desiredHeadingFrame, xCoMInTime[1], yCoMInTime[1], 0.0);
      pelvisVelocityInTime.changeFrame(pelvisFrame);
      
      
      return new Pair<FramePose, FrameVector>(pelvisPoseInTime, pelvisVelocityInTime);
      
   }
}
