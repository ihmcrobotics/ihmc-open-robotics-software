package us.ihmc.commonWalkingControlModules.kinematics;

import javax.vecmath.Vector3d;

import us.ihmc.CapturePointCalculator.LinearInvertedPendulumCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
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
   private final ReferenceFrame worldFrame;
   private final ReferenceFrame bodyZUpFrame;
   private final CouplingRegistry couplingRegistry;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("BodyPositionInTimeEstimator");
   private final DoubleYoVariable wxToYDotCoupling = new DoubleYoVariable("wxToYDotCoupling", registry);
   private final DoubleYoVariable wyToXDotCoupling = new DoubleYoVariable("wyToXDotCoupling", registry);
   public BodyPositionInTimeEstimator(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames, CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.pelvisFrame = referenceFrames.getPelvisFrame();
      this.worldFrame = ReferenceFrame.getWorldFrame();
      this.couplingRegistry = couplingRegistry;
      this.bodyZUpFrame = referenceFrames.getABodyAttachedZUpFrame();
      parentRegistry.addChild(registry);
      
      
      // TODO: Get rid of this as soon as we have a good body velocity estimation
      wxToYDotCoupling.set(0.3); //+++JEP 111108: Changed from 0.36 to 0.3, which really reduced the shakies on the real robot.
      wyToXDotCoupling.set(0.3); 
   }
   
   
   public Pair<FramePose, FrameVector> getPelvisPoseAndPositionInTime(double t, RobotSide swingFoot)
   {
//      FramePoint2d currentCoPPosition = couplingRegistry.getDesiredCoP();
      FramePoint2d currentCoPPosition = couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(swingFoot.getOppositeSide());
      //FramePoint currentCoPPosition = new FramePoint(referenceFrames.getAnkleZUpFrame(swingFoot));
      currentCoPPosition.changeFrame(worldFrame);
      FramePoint currentCoMPosition = processedSensors.getCenterOfMassPositionInFrame(worldFrame);
      FrameVector currentCoMVelocity = getBodyVelocity();
//      currentCoMVelocity.setX(0.4);
//      currentCoMVelocity.setY(0.0);
//      currentCoMVelocity.setZ(0.0);
      currentCoMVelocity.changeFrame(worldFrame);
      
      FramePoint bodyPosition = new FramePoint(pelvisFrame);
      bodyPosition.changeFrame(worldFrame);
      
      
      // LIPM Model in time
      
      double comHeight = 0.94;
      double gravity = Math.abs(processedSensors.getGravityInWorldFrame().getZ());
      
      double[] xCoMInTime = LinearInvertedPendulumCapturePointCalculator.calculatePredictedCoMState(
                           currentCoMPosition.getX(), currentCoPPosition.getX(), currentCoMVelocity.getX(), 
                           gravity, comHeight, 0.0, t);
      double[] yCoMInTime = LinearInvertedPendulumCapturePointCalculator.calculatePredictedCoMState(
                           currentCoMPosition.getY(), currentCoPPosition.getY(), currentCoMVelocity.getY(), 
                           gravity, comHeight, 0.0, t);
      
      double xDeltaInTime = xCoMInTime[0] - currentCoMPosition.getX();
      double yDeltaInTime = yCoMInTime[0] - currentCoMPosition.getY();
      
      bodyPosition.setX(bodyPosition.getX() + xDeltaInTime);
      bodyPosition.setY(bodyPosition.getY() + yDeltaInTime);
      bodyPosition.changeFrame(pelvisFrame);
      
      
      FramePose pelvisPoseInTime = new FramePose(pelvisFrame);
      pelvisPoseInTime.setPosition(bodyPosition);
      
      
      FrameVector pelvisVelocityInTime = new FrameVector(worldFrame, xCoMInTime[1], yCoMInTime[1], 0.0);
      pelvisVelocityInTime.changeFrame(pelvisFrame);
      
      
      return new Pair<FramePose, FrameVector>(pelvisPoseInTime, pelvisVelocityInTime);
      
   }
   

   
   private FrameVector getBodyVelocity()
   {

         FrameVector pdBodyZUp = processedSensors.getBodyVelocity();
         pdBodyZUp.changeFrame(bodyZUpFrame);
         

         Vector3d angularPartCopy = processedSensors.getTwistOfPelvisWithRespectToWorld().getAngularPartCopy();
         double pd_wy = angularPartCopy.getY();
         double pd_wx = angularPartCopy.getX();
         
         pdBodyZUp.setX(pdBodyZUp.getX() - wyToXDotCoupling.getDoubleValue() * pd_wy);
         pdBodyZUp.setY(pdBodyZUp.getY() + wxToYDotCoupling.getDoubleValue() * pd_wx);
         
         return pdBodyZUp;
         
    
   }
}
