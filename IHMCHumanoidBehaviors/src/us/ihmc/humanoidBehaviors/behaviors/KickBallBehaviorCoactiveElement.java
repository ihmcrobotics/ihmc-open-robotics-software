package us.ihmc.humanoidBehaviors.behaviors;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFramePointArray;

public abstract class KickBallBehaviorCoactiveElement extends BehaviorCoactiveElement
{
   private static final int MAX_DETECTED_BALLS = 50;
   
   //UI SIDE YOVARS
<<<<<<< HEAD
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

   public final DoubleYoVariable ballX = new DoubleYoVariable("ballX", machineWritableRegistry);
   public final DoubleYoVariable ballY = new DoubleYoVariable("ballY", machineWritableRegistry);
   public final DoubleYoVariable ballZ = new DoubleYoVariable("ballZ", machineWritableRegistry);

   public final DoubleYoVariable ballRadius = new DoubleYoVariable("ballRadius", machineWritableRegistry);
   public final BooleanYoVariable validAcknowledged = new BooleanYoVariable("validAcknowledged", machineWritableRegistry);
   public final BooleanYoVariable waitingForValidation = new BooleanYoVariable("waitingForValidation", machineWritableRegistry);
=======
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
>>>>>>> 193b146e57b234e6256cfc0a71f8c100364c969a
}
