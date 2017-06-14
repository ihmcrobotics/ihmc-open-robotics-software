package us.ihmc.humanoidBehaviors.behaviors.coactiveElements;

import us.ihmc.yoVariables.variable.BooleanYoVariable;
import us.ihmc.yoVariables.variable.DoubleYoVariable;
import us.ihmc.yoVariables.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFramePointArray;

public abstract class KickBallBehaviorCoactiveElement extends BehaviorCoactiveElement
{
   private static final int MAX_DETECTED_BALLS = 50;

   //UI SIDE YOVARS
   public final IntegerYoVariable userInterfaceSideCount = new IntegerYoVariable("userInterfaceSideCount", userInterfaceWritableRegistry);
   public final BooleanYoVariable abortClicked = new BooleanYoVariable("abortClicked", userInterfaceWritableRegistry);
   public final BooleanYoVariable validClicked = new BooleanYoVariable("validClicked", userInterfaceWritableRegistry);

   //BEHAVIOR SIDE YOVARS
   public final IntegerYoVariable machineSideCount = new IntegerYoVariable("machineSideCount", machineWritableRegistry);
   public final IntegerYoVariable abortCount = new IntegerYoVariable("abortCount", machineWritableRegistry);
   public final BooleanYoVariable abortAcknowledged = new BooleanYoVariable("abortAcknowledged", machineWritableRegistry);
   public final BooleanYoVariable searchingForBall = new BooleanYoVariable("searchingForBall", machineWritableRegistry);
   public final BooleanYoVariable foundBall = new BooleanYoVariable("foundBall", machineWritableRegistry);

   public final IntegerYoVariable numBlobsDetected = new IntegerYoVariable("numBlobsDetected", machineWritableRegistry);
   public final DoubleYoVariable blobX = new DoubleYoVariable("blobX", machineWritableRegistry);
   public final DoubleYoVariable blobY = new DoubleYoVariable("blobY", machineWritableRegistry);

   public final YoFramePointArray ballPositions = new YoFramePointArray(MAX_DETECTED_BALLS, "detectedBall", machineWritableRegistry);
   public final DoubleYoVariable[] ballRadii = new DoubleYoVariable[MAX_DETECTED_BALLS];
   {
      for (int index = 0; index < MAX_DETECTED_BALLS; index++)
      {
         ballRadii[index] = new DoubleYoVariable("ballRadius" + index, machineWritableRegistry);
      }
   }
   public final BooleanYoVariable validAcknowledged = new BooleanYoVariable("validAcknowledged", machineWritableRegistry);
   public final BooleanYoVariable waitingForValidation = new BooleanYoVariable("waitingForValidation", machineWritableRegistry);
}
