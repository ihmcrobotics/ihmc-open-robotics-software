package us.ihmc.commonWalkingControlModules.pushRecovery;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PushRobotTestConductor
{
   private final String jointName;
   private final YoDouble pushDuration;
   private final YoDouble pushMagnitude;
   private final YoFrameVector pushDirection;
   private final YoFrameVector pushForce;
   private final YoDouble pushTimeSwitch;
   private final YoInteger pushNumber;
   private final YoDouble pushDelay;
   private final YoDouble yoTime;
   
   public PushRobotTestConductor(SimulationConstructionSet scs, String jointName)
   {
      this.jointName = jointName;
      
      pushDuration = (YoDouble) scs.getVariable(jointName + "_pushDuration");
      pushMagnitude = (YoDouble) scs.getVariable(jointName + "_pushMagnitude");
      pushTimeSwitch = (YoDouble) scs.getVariable(jointName + "_pushTimeSwitch");
      pushNumber = (YoInteger) scs.getVariable(jointName + "_pushNumber");
      pushDelay = (YoDouble) scs.getVariable(jointName + "_pushDelay");
      yoTime = (YoDouble) scs.getVariable("t");
      
      pushDirection = new YoFrameVector((YoDouble) scs.getVariable(jointName + "_pushDirectionX"),
                                        (YoDouble) scs.getVariable(jointName + "_pushDirectionY"),
                                        (YoDouble) scs.getVariable(jointName + "_pushDirectionZ"), ReferenceFrame.getWorldFrame());
      pushForce = new YoFrameVector((YoDouble) scs.getVariable(jointName + "_pushForceX"),
                                    (YoDouble) scs.getVariable(jointName + "_pushForceY"),
                                    (YoDouble) scs.getVariable(jointName + "_pushForceZ"), ReferenceFrame.getWorldFrame());
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
