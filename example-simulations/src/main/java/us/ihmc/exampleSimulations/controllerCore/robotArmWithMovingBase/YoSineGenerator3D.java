package us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSineGenerator3D implements PositionTrajectoryGenerator
{
   private final YoFrameVector3D offset;
   private final YoFrameVector3D amplitude;
   private final YoFrameVector3D frequency;
   private final YoFrameVector3D phase;

   private final YoFramePoint3D position;
   private final YoFrameVector3D velocity;
   private final YoFrameVector3D acceleration;

   public YoSineGenerator3D(String namePrefix, ReferenceFrame signalFrame, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      offset = new YoFrameVector3D(namePrefix + "Offset", signalFrame, registry);
      amplitude = new YoFrameVector3D(namePrefix + "Amplitude", signalFrame, registry);
      frequency = new YoFrameVector3D(namePrefix + "Frequency", signalFrame, registry);
      phase = new YoFrameVector3D(namePrefix + "Phase", signalFrame, registry);

      position = new YoFramePoint3D(namePrefix + "Position", signalFrame, registry);
      velocity = new YoFrameVector3D(namePrefix + "Velocity", signalFrame, registry);
      acceleration = new YoFrameVector3D(namePrefix + "Acceleration", signalFrame, registry);
   }

   public void setOffset(double x, double y, double z)
   {
      offset.set(x, y, z);
   }

   public void setOffset(Vector3DReadOnly offset)
   {
      this.offset.set(offset);
   }

   public void setOffset(FrameTuple3DReadOnly offset)
   {
      this.offset.set(offset);
   }

   public void setAmplitude(double x, double y, double z)
   {
      amplitude.set(x, y, z);
   }

   public void setFrequency(double x, double y, double z)
   {
      frequency.set(x, y, z);
   }

   public void setPhase(double x, double y, double z)
   {
      phase.set(x, y, z);
   }

   @Override
   public void initialize()
   {
   }

   public void compute(double time)
   {
      for (int i = 0; i < 3; i++)
      {
         double amplitude = this.amplitude.getElement(i);
         double offset = this.offset.getElement(i);
         double pulse = 2.0 * Math.PI * frequency.getElement(i);
         double phase = this.phase.getElement(i);

         double x = offset + amplitude * Math.sin(pulse * time + phase);
         double xd = amplitude * pulse * Math.cos(pulse * time + phase);
         double xdd = -amplitude * pulse * pulse * Math.sin(pulse * time + phase);

         position.setElement(i, x);
         velocity.setElement(i, xd);
         acceleration.setElement(i, xdd);
      }
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(velocity);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(acceleration);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}
