package us.ihmc.commonWalkingControlModules.pushRecovery;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PushRobotTestConductor
{
   private final String jointName;
   private final DoubleYoVariable pushDuration;
   private final DoubleYoVariable pushMagnitude;
   private final YoFrameVector pushDirection;
   private final YoFrameVector pushForce;
   private final DoubleYoVariable pushTimeSwitch;
   private final IntegerYoVariable pushNumber;
   private final DoubleYoVariable pushDelay;
   private final DoubleYoVariable yoTime;
   
   public PushRobotTestConductor(SimulationConstructionSet scs, String jointName)
   {
      this.jointName = jointName;
      
      pushDuration = (DoubleYoVariable) scs.getVariable(jointName + "_pushDuration");
      pushMagnitude = (DoubleYoVariable) scs.getVariable(jointName + "_pushMagnitude");
      pushTimeSwitch = (DoubleYoVariable) scs.getVariable(jointName + "_pushTimeSwitch");
      pushNumber = (IntegerYoVariable) scs.getVariable(jointName + "_pushNumber");
      pushDelay = (DoubleYoVariable) scs.getVariable(jointName + "_pushDelay");
      yoTime = (DoubleYoVariable) scs.getVariable("t");
      
      pushDirection = new YoFrameVector((DoubleYoVariable) scs.getVariable(jointName + "_pushDirectionX"),
                                        (DoubleYoVariable) scs.getVariable(jointName + "_pushDirectionY"),
                                        (DoubleYoVariable) scs.getVariable(jointName + "_pushDirectionZ"), ReferenceFrame.getWorldFrame());
      pushForce = new YoFrameVector((DoubleYoVariable) scs.getVariable(jointName + "_pushForceX"),
                                    (DoubleYoVariable) scs.getVariable(jointName + "_pushForceY"),
                                    (DoubleYoVariable) scs.getVariable(jointName + "_pushForceZ"), ReferenceFrame.getWorldFrame());
   }
   
   public void applyForce(Vector3D direction, double magnitude, double duration)
   {
      PrintTools.info("\nPushing " + jointName + " direction: " + direction + " magnitude: " + magnitude + "(N) for " + duration + "(s)");
      
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
