package us.ihmc.commonWalkingControlModules.pushRecovery;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PushRobotTestConductor
{
   private final DoubleYoVariable pushDuration;
   private final DoubleYoVariable pushMagnitude;
   private final YoFrameVector pushDirection;
   private final YoFrameVector pushForce;
   private final DoubleYoVariable pushTimeSwitch;
   private final IntegerYoVariable pushNumber;
   private final DoubleYoVariable pushDelay;
   private final DoubleYoVariable yoTime;
   
   public PushRobotTestConductor(SimulationConstructionSet scs)
   {
      pushDuration = (DoubleYoVariable) scs.getVariable("pushDuration");
      pushMagnitude = (DoubleYoVariable) scs.getVariable("pushMagnitude");
      pushTimeSwitch = (DoubleYoVariable) scs.getVariable("pushTimeSwitch");
      pushNumber = (IntegerYoVariable) scs.getVariable("pushNumber");
      pushDelay = (DoubleYoVariable) scs.getVariable("pushDelay");
      yoTime = (DoubleYoVariable) scs.getVariable("t");
      
      pushDirection = new YoFrameVector((DoubleYoVariable) scs.getVariable("pushDirectionX"), (DoubleYoVariable) scs.getVariable("pushDirectionY"), (DoubleYoVariable) scs.getVariable("pushDirectionZ"), ReferenceFrame.getWorldFrame());
      pushForce = new YoFrameVector((DoubleYoVariable) scs.getVariable("pushForceX"), (DoubleYoVariable) scs.getVariable("pushForceY"), (DoubleYoVariable) scs.getVariable("pushForceZ"), ReferenceFrame.getWorldFrame());
   }
   
   public void applyForce(Vector3d direction, double magnitude, double duration)
   {
      pushDuration.set(duration);
      pushDelay.set(0.0);
      pushDirection.set(direction);
      pushMagnitude.set(magnitude);
      
      if (pushDirection.length() > 1e-5)
      {
         pushForce.set(pushDirection);
         pushForce.normalize();
         pushForce.scale(pushMagnitude.getDoubleValue());
         pushTimeSwitch.set(yoTime.getDoubleValue());
      }
      else
      {
         pushForce.setToZero();
         pushTimeSwitch.set(Double.NEGATIVE_INFINITY);
      }

      pushNumber.increment();
   }
}
