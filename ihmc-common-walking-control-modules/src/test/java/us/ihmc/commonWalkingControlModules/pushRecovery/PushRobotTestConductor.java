package us.ihmc.commonWalkingControlModules.pushRecovery;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class PushRobotTestConductor
{
   private final String jointName;
   private final YoDouble pushDuration;
   private final YoDouble pushMagnitude;
   private final YoFrameVector3D pushDirection;
   private final YoFrameVector3D pushForce;
   private final YoDouble pushTimeSwitch;
   private final YoInteger pushNumber;
   private final YoDouble pushDelay;
   private final YoDouble yoTime;
   
   public PushRobotTestConductor(SimulationConstructionSet scs, String jointName)
   {
      this.jointName = jointName;
      
      pushDuration = (YoDouble) scs.findVariable(jointName + "_pushDuration");
      pushMagnitude = (YoDouble) scs.findVariable(jointName + "_pushMagnitude");
      pushTimeSwitch = (YoDouble) scs.findVariable(jointName + "_pushTimeSwitch");
      pushNumber = (YoInteger) scs.findVariable(jointName + "_pushNumber");
      pushDelay = (YoDouble) scs.findVariable(jointName + "_pushDelay");
      yoTime = (YoDouble) scs.findVariable("t");
      
      pushDirection = new YoFrameVector3D((YoDouble) scs.findVariable(jointName + "_pushDirectionX"),
                                        (YoDouble) scs.findVariable(jointName + "_pushDirectionY"),
                                        (YoDouble) scs.findVariable(jointName + "_pushDirectionZ"), ReferenceFrame.getWorldFrame());
      pushForce = new YoFrameVector3D((YoDouble) scs.findVariable(jointName + "_pushForceX"),
                                    (YoDouble) scs.findVariable(jointName + "_pushForceY"),
                                    (YoDouble) scs.findVariable(jointName + "_pushForceZ"), ReferenceFrame.getWorldFrame());
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
