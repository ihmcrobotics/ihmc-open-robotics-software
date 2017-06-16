package us.ihmc.humanoidBehaviors.behaviors.coactiveElements;

import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFramePointArray;

public abstract class KickBallBehaviorCoactiveElement extends BehaviorCoactiveElement
{
   private static final int MAX_DETECTED_BALLS = 50;

   //UI SIDE YOVARS
   public final IntegerYoVariable userInterfaceSideCount = new IntegerYoVariable("userInterfaceSideCount", userInterfaceWritableRegistry);
   public final YoBoolean abortClicked = new YoBoolean("abortClicked", userInterfaceWritableRegistry);
   public final YoBoolean validClicked = new YoBoolean("validClicked", userInterfaceWritableRegistry);

   //BEHAVIOR SIDE YOVARS
   public final IntegerYoVariable machineSideCount = new IntegerYoVariable("machineSideCount", machineWritableRegistry);
   public final IntegerYoVariable abortCount = new IntegerYoVariable("abortCount", machineWritableRegistry);
   public final YoBoolean abortAcknowledged = new YoBoolean("abortAcknowledged", machineWritableRegistry);
   public final YoBoolean searchingForBall = new YoBoolean("searchingForBall", machineWritableRegistry);
   public final YoBoolean foundBall = new YoBoolean("foundBall", machineWritableRegistry);

   public final IntegerYoVariable numBlobsDetected = new IntegerYoVariable("numBlobsDetected", machineWritableRegistry);
   public final YoDouble blobX = new YoDouble("blobX", machineWritableRegistry);
   public final YoDouble blobY = new YoDouble("blobY", machineWritableRegistry);

   public final YoFramePointArray ballPositions = new YoFramePointArray(MAX_DETECTED_BALLS, "detectedBall", machineWritableRegistry);
   public final YoDouble[] ballRadii = new YoDouble[MAX_DETECTED_BALLS];
   {
      for (int index = 0; index < MAX_DETECTED_BALLS; index++)
      {
         ballRadii[index] = new YoDouble("ballRadius" + index, machineWritableRegistry);
      }
   }
   public final YoBoolean validAcknowledged = new YoBoolean("validAcknowledged", machineWritableRegistry);
   public final YoBoolean waitingForValidation = new YoBoolean("waitingForValidation", machineWritableRegistry);
}
