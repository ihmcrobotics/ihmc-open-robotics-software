package us.ihmc.humanoidBehaviors.behaviors;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public abstract class PickUpBallsBehaviorCoactiveElement extends BehaviorCoactiveElement
{
   // UI SIDE YOVARS
   public final IntegerYoVariable minHue = new IntegerYoVariable("minHue", userInterfaceWritableRegistry);
   public final IntegerYoVariable minSat = new IntegerYoVariable("minSat", userInterfaceWritableRegistry);
   public final IntegerYoVariable minVal = new IntegerYoVariable("minVal", userInterfaceWritableRegistry);
   public final IntegerYoVariable maxHue = new IntegerYoVariable("maxHue", userInterfaceWritableRegistry);
   public final IntegerYoVariable maxSat = new IntegerYoVariable("maxSat", userInterfaceWritableRegistry);
   public final IntegerYoVariable maxVal = new IntegerYoVariable("maxVal", userInterfaceWritableRegistry);

   public final BooleanYoVariable abortClicked = new BooleanYoVariable("abortClicked", userInterfaceWritableRegistry);
   public final BooleanYoVariable validClicked = new BooleanYoVariable("validClicked", userInterfaceWritableRegistry);

   // BEHAVIOR SIDE YOVARS
   public final IntegerYoVariable numBlobsDetected = new IntegerYoVariable("numBlobsDetected", machineWritableRegistry);
   public final DoubleYoVariable blobX = new DoubleYoVariable("blobX", machineWritableRegistry);
   public final DoubleYoVariable blobY = new DoubleYoVariable("blobY", machineWritableRegistry);
   public final DoubleYoVariable blobZ = new DoubleYoVariable("blobZ", machineWritableRegistry);

   public final BooleanYoVariable validAcknowledged = new BooleanYoVariable("validAcknowledged", machineWritableRegistry);
   public final BooleanYoVariable waitingForValidation = new BooleanYoVariable("waitingForValidation", machineWritableRegistry);
}
