package us.ihmc.humanoidBehaviors.behaviors;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFramePointArray;

public abstract class KickBallBehaviorCoactiveElement extends BehaviorCoactiveElement
{
   private static final int MAX_DETECTED_BALLS = 50;
   
   //UI SIDE YOVARS
   protected final IntegerYoVariable userInterfaceSideCount = new IntegerYoVariable("userInterfaceSideCount", userInterfaceWritableRegistry);
   protected final BooleanYoVariable abortClicked = new BooleanYoVariable("abortClicked", userInterfaceWritableRegistry);
   protected final BooleanYoVariable validClicked = new BooleanYoVariable("validClicked", userInterfaceWritableRegistry);

   //BEHAVIOR SIDE YOVARS
   protected final IntegerYoVariable machineSideCount = new IntegerYoVariable("machineSideCount", machineWritableRegistry);
   protected final IntegerYoVariable abortCount = new IntegerYoVariable("abortCount", machineWritableRegistry);
   protected final BooleanYoVariable abortAcknowledged = new BooleanYoVariable("abortAcknowledged", machineWritableRegistry);
   protected final BooleanYoVariable searchingForBall = new BooleanYoVariable("searchingForBall", machineWritableRegistry);
   protected final BooleanYoVariable foundBall = new BooleanYoVariable("foundBall", machineWritableRegistry);
   protected final YoFramePointArray ballPositions = new YoFramePointArray(MAX_DETECTED_BALLS, "detectedBall", machineWritableRegistry);
   protected final DoubleYoVariable[] ballRadii = new DoubleYoVariable[MAX_DETECTED_BALLS];
   {
      for (int index = 0; index < MAX_DETECTED_BALLS; index++)
      {
         ballRadii[index] = new DoubleYoVariable("ballRadius" + index, machineWritableRegistry);
      }
   }
   protected final BooleanYoVariable validAcknowledged = new BooleanYoVariable("validAcknowledged", machineWritableRegistry);
   protected final BooleanYoVariable waitingForValidation = new BooleanYoVariable("waitingForValidation", machineWritableRegistry);
}
