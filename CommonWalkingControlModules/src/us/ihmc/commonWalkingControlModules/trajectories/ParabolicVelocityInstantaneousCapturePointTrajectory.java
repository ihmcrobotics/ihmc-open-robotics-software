package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;


public class ParabolicVelocityInstantaneousCapturePointTrajectory implements InstantaneousCapturePointTrajectory
{
   //TODO: This class is completely incomprehensible. Should make it more readable and clean it up...
   
   private static final int MAX_LOOPS = (int) 100;//1e7;
   private final YoVariableRegistry registry;
   private final ZeroToOneParabolicVelocityTrajectoryGenerator parameterGenerator;
   private final YoFramePoint2d initialDesiredICP;
   private final YoFramePoint2d finalDesiredICP;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable currentTime;
   private final IntegerYoVariable loopCount;
   
   private final BipedSupportPolygons bipedSupportPolygons;
   
   private FramePoint2d tempPointInitialDesired = new FramePoint2d(ReferenceFrame.getWorldFrame());
   private final FramePoint2d tempPointFinalDesired = new FramePoint2d(ReferenceFrame.getWorldFrame());

   public ParabolicVelocityInstantaneousCapturePointTrajectory(String namePrefix, BipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.bipedSupportPolygons = bipedSupportPolygons;

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

      this.initialDesiredICP = new YoFramePoint2d(namePrefix, "InitialDesiredICP", referenceFrame, registry);
      this.finalDesiredICP = new YoFramePoint2d(namePrefix, "FinalDesiredICP", referenceFrame, registry);
      
      trajectoryTime = new DoubleYoVariable(namePrefix + "icpTrajectoryTime", registry);
      startTime = new DoubleYoVariable(namePrefix + "icpStartTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "icpCurrentTime", registry);
            
      loopCount = new IntegerYoVariable(namePrefix + "loopCount", registry);
      
      parameterGenerator = new ZeroToOneParabolicVelocityTrajectoryGenerator(namePrefix, Double.NaN, registry); // Set a sensible value for the trajectoryTime during initialization.
      
      parentRegistry.addChild(registry);   
   }

   public boolean isDone()
   {
      return parameterGenerator != null && parameterGenerator.isDone();
   }

   public void initialize(FramePoint2d initialDesiredICP, FramePoint2d finalDesiredICP, double moveTime, double omega0, double amountToBeInside, RobotSide supportSide, double time)
   {
      startTime.set(time);
      
      initialDesiredICP.changeFrame(this.initialDesiredICP.getReferenceFrame());
      finalDesiredICP.changeFrame(this.finalDesiredICP.getReferenceFrame());
      
      this.initialDesiredICP.set(initialDesiredICP);
      this.finalDesiredICP.set(finalDesiredICP);
      
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();

      
      ReferenceFrame supportReferenceFrame = supportPolygon.getReferenceFrame();
      FramePoint2d lowerExtremeCoP = new FramePoint2d(supportReferenceFrame);
      FramePoint2d higherExtremeCoP = new FramePoint2d(supportReferenceFrame);
      
      packLowerExtremeCoPPoint(lowerExtremeCoP, moveTime, omega0);
      packHigherExtremeCoPPoint(higherExtremeCoP, moveTime, omega0);  // TODO: take amountToBeInside into account.
      lowerExtremeCoP.changeFrame(supportReferenceFrame);
      higherExtremeCoP.changeFrame(supportReferenceFrame);
      
      loopCount.set(0);
      while(!supportPolygon.isPointInside(lowerExtremeCoP) && supportPolygon.isPointInside(higherExtremeCoP))
      {
         if (loopCount.getIntegerValue() >= MAX_LOOPS)
         {
            System.err.println("Maximum amount of loops reached to determine trajectoryTime. Possible infinite loop.");
            System.err.println("supportPolygon = " + supportPolygon);
            System.err.println("initialDesiredICP = " + initialDesiredICP);
            System.err.println("finalDesiredICP = " + finalDesiredICP);
            System.err.println("moveTime = " + moveTime);
            System.err.println("omega0 = " + omega0);
            System.err.println("amountToBeInside = " + amountToBeInside);
            System.err.println("supportSide = " + supportSide);
            System.err.println("time = " + time);
            
            parameterGenerator.setTrajectoryTime(moveTime);
            parameterGenerator.initialize();
            
            this.trajectoryTime.set(moveTime);
            this.currentTime.set(0.0);
            return;
//            throw new RuntimeException("Maximum amount of loops reached to determine trajectoryTime. Possible infinite loop.");
         }
         
         moveTime += 0.001; //TODO: What the heck is this?
         
         packLowerExtremeCoPPoint(lowerExtremeCoP, moveTime, omega0);
         packHigherExtremeCoPPoint(higherExtremeCoP, moveTime, omega0); // TODO: take amountToBeInside into account.
         lowerExtremeCoP.changeFrame(supportReferenceFrame);
         higherExtremeCoP.changeFrame(supportReferenceFrame);
         
         loopCount.increment();
      }
      
      parameterGenerator.setTrajectoryTime(moveTime);
      parameterGenerator.initialize();
      
      this.trajectoryTime.set(moveTime);
      this.currentTime.set(0.0);
   }
   
   private double symmetricPart(double finalTime, double omega0)
   {
      return 1.0/omega0 + finalTime/2.0;
   }
   
   private double aSymmetricPart(double finalTime, double omega0)
   {
      return Math.sqrt(4.0 + MathTools.square(finalTime*omega0));
   }
   
   private void packCoP(FramePoint2d point2Pack, double time, double finalTime, double omega0)
   {
      point2Pack.setIncludingFrame(finalDesiredICP.getFramePoint2dCopy());
      tempPointInitialDesired.setIncludingFrame(initialDesiredICP.getFramePoint2dCopy());

      // See ParabolicVelocityICPTrajectoryGenerator.nb in /mathematica
      point2Pack.scale(time * (-6.0 * finalTime + time *(6.0 - 2.0 * time * omega0 + 3.0 * finalTime * omega0)));
      tempPointInitialDesired.scale((time - finalTime) * (-6.0 * time + (time - finalTime) * (2.0 * time + finalTime) * omega0));
      
      point2Pack.add(tempPointInitialDesired);
      point2Pack.scale(1.0 / (MathTools.cube(finalTime) * omega0 ));
   }
   
   private void packLowerExtremeCoPPoint(FramePoint2d point2Pack, double finalTime, double omega0)
   {
      double time = symmetricPart(finalTime, omega0) - aSymmetricPart(finalTime, omega0);
      packCoP(point2Pack, time, finalTime, omega0);
   }
   
   private void packHigherExtremeCoPPoint(FramePoint2d point2Pack, double finalTime, double omega0)
   {
      double time = symmetricPart(finalTime, omega0) + aSymmetricPart(finalTime, omega0);
      packCoP(point2Pack, time, finalTime, omega0);
   }

   public void getCurrentDesiredICPPositionAndVelocity(FramePoint2d desiredPosition, FrameVector2d desiredVelocity, double omega0, double time)
   {
      currentTime.set(time - startTime.getDoubleValue());
      parameterGenerator.compute(currentTime.getDoubleValue());
      
      double parameter = parameterGenerator.getValue();
      double parameterDot = parameterGenerator.getVelocity();
      tempPointInitialDesired.setIncludingFrame(initialDesiredICP.getFramePoint2dCopy());
      tempPointFinalDesired.setIncludingFrame(finalDesiredICP.getFramePoint2dCopy());
      
      desiredPosition.setIncludingFrame(tempPointInitialDesired);
      desiredPosition.scale(1.0 - parameter);
      tempPointFinalDesired.scale(parameter);
      desiredPosition.add(tempPointFinalDesired);
      
      tempPointFinalDesired.scale(parameterDot/parameter);
      desiredVelocity.setIncludingFrame(tempPointInitialDesired);
      desiredVelocity.scale(-parameterDot);
      desiredVelocity.add(tempPointFinalDesired);      
   }

   public double getMoveTime()
   {
      return trajectoryTime.getDoubleValue();
   }

   public void reset()
   {
      this.currentTime.set(0.0);
   }

   public double getStartTime()
   {
      return startTime.getDoubleValue();
   }
   
   public double getEstimatedTimeRemainingForState(double time)
   {
      return currentTime.getDoubleValue() - trajectoryTime.getDoubleValue();
   }

}
