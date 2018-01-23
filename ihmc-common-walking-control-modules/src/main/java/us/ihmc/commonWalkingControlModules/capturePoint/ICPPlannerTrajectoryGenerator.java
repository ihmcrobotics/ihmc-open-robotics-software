package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

import static us.ihmc.commonWalkingControlModules.dynamicReachability.CoMIntegrationTools.integrateCoMPositionUsingCubicICP;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class ICPPlannerTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final VelocityConstrainedPositionTrajectoryGenerator doubleSupportCapturePointTrajectory;

   private final FramePoint3D initialPositionInSpecificFrame = new FramePoint3D();
   private final FrameVector3D initialVelocityInSpecificFrame = new FrameVector3D();
   private final FramePoint3D finalPositionInSpecificFrame = new FramePoint3D();
   private final FrameVector3D finalVelocityInSpecificFrame = new FrameVector3D();

   private final FramePoint3D initialCoMPositionInSpecificFrame = new FramePoint3D();
   private final FramePoint3D desiredCoMPosition = new FramePoint3D();

   private final YoDouble omega0;

   public ICPPlannerTrajectoryGenerator(String namePrefix, ReferenceFrame trajectoryFrame, YoDouble omega0, YoVariableRegistry registry)
   {
      this.omega0 = omega0;
      doubleSupportCapturePointTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix, trajectoryFrame, registry);
   }

   public void setTrajectoryTime(double duration)
   {
      doubleSupportCapturePointTrajectory.setTrajectoryTime(duration);
   }

   public void setInitialConditions(YoFramePoint initialPosition, YoFrameVector initialVelocity, ReferenceFrame attachedFrame)
   {
      initialPositionInSpecificFrame.setIncludingFrame(initialPosition);
      initialVelocityInSpecificFrame.setIncludingFrame(initialVelocity);
      initialPositionInSpecificFrame.changeFrame(attachedFrame);
      initialVelocityInSpecificFrame.changeFrame(attachedFrame);
   }

   public void setInitialCoMPosition(YoFramePoint initialCoMPosition, ReferenceFrame attachedFrame)
   {
      initialCoMPositionInSpecificFrame.setIncludingFrame(initialCoMPosition);
      initialCoMPositionInSpecificFrame.changeFrame(attachedFrame);
   }
   
   public void setFinalConditions(YoFramePoint finalPosition, YoFrameVector finalVelocity, ReferenceFrame attachedFrame)
   {
      finalPositionInSpecificFrame.setIncludingFrame(finalPosition);
      finalVelocityInSpecificFrame.setIncludingFrame(finalVelocity);
      finalPositionInSpecificFrame.changeFrame(attachedFrame);
      finalVelocityInSpecificFrame.changeFrame(attachedFrame);
   }

   public void computeFinalCoMPosition(FramePoint3D finalCoMToPack)
   {
      computeCoMPosition(doubleSupportCapturePointTrajectory.getTrajectoryTime(), finalCoMToPack);
   }

   @Override
   public void initialize()
   {
      doubleSupportCapturePointTrajectory.setInitialConditions(initialPositionInSpecificFrame, initialVelocityInSpecificFrame);
      doubleSupportCapturePointTrajectory.setFinalConditions(finalPositionInSpecificFrame, finalVelocityInSpecificFrame);
      doubleSupportCapturePointTrajectory.initialize();
   }

   @Override
   public void compute(double time)
   {
      doubleSupportCapturePointTrajectory.setInitialConditions(initialPositionInSpecificFrame, initialVelocityInSpecificFrame);
      doubleSupportCapturePointTrajectory.setFinalConditions(finalPositionInSpecificFrame, finalVelocityInSpecificFrame);
      doubleSupportCapturePointTrajectory.initialize();
      doubleSupportCapturePointTrajectory.compute(time);

      computeCoMPosition(time, desiredCoMPosition);
   }

   public void computeCoMPosition(double time, FramePoint3D comPositionToPack)
   {
      YoPolynomial xPolynomial = doubleSupportCapturePointTrajectory.getXPolynomial();
      YoPolynomial yPolynomial = doubleSupportCapturePointTrajectory.getYPolynomial();

      if (Double.isNaN(doubleSupportCapturePointTrajectory.getTrajectoryTime()))
      {
         comPositionToPack.set(initialCoMPositionInSpecificFrame);
      }
      else
      {
         integrateCoMPositionUsingCubicICP(0.0, time, doubleSupportCapturePointTrajectory.getTrajectoryTime(), omega0.getDoubleValue(),
               doubleSupportCapturePointTrajectory.getCurrentTrajectoryFrame(), xPolynomial, yPolynomial, initialCoMPositionInSpecificFrame, comPositionToPack);
      }
   }

   @Override
   public boolean isDone()
   {
      return doubleSupportCapturePointTrajectory.isDone();
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      doubleSupportCapturePointTrajectory.getPosition(positionToPack);
   }

   public void get(YoFramePoint positionToPack)
   {
      doubleSupportCapturePointTrajectory.get(positionToPack);
   }


   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      doubleSupportCapturePointTrajectory.getVelocity(velocityToPack);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      doubleSupportCapturePointTrajectory.getVelocity(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.getAcceleration(accelerationToPack);
   }

   public void getAcceleration(YoFrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.getAcceleration(accelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.getLinearData(positionToPack, velocityToPack, accelerationToPack);
   }

   public void getLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
   {
      doubleSupportCapturePointTrajectory.getLinearData(positionToPack, velocityToPack, accelerationToPack);
   }

   public void getCoMPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(desiredCoMPosition);
   }

   @Override
   public void showVisualization()
   {
      doubleSupportCapturePointTrajectory.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      doubleSupportCapturePointTrajectory.hideVisualization();
   }
}
