package us.ihmc.humanoidBehaviors.behaviors.coactiveElements;

import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class PickUpBallBehaviorCoactiveElementOLD extends BehaviorCoactiveElement
{
   public enum PickUpBallBehaviorState
   {
      STOPPED,
      ENABLING_LIDAR,
      SETTING_LIDAR_PARAMS,
      CLEARING_LIDAR,
      SEARCHING_FOR_BALL,
      WAITING_FOR_USER_CONFIRMATION,
      WALKING_TO_BALL,
      BENDING_OVER,
      REACHING_FOR_BALL,
      CLOSING_HAND,
      PICKING_UP_BALL,
      PUTTING_BALL_IN_BASKET
   }

   //UI SIDE YOVARS
   public final YoInteger minHue = new YoInteger("minHue", userInterfaceWritableRegistry);
   public final YoInteger minSat = new YoInteger("minSat", userInterfaceWritableRegistry);
   public final YoInteger minVal = new YoInteger("minVal", userInterfaceWritableRegistry);
   public final YoInteger maxHue = new YoInteger("maxHue", userInterfaceWritableRegistry);
   public final YoInteger maxSat = new YoInteger("maxSat", userInterfaceWritableRegistry);
   public final YoInteger maxVal = new YoInteger("maxVal", userInterfaceWritableRegistry);
   
   public final YoInteger userInterfaceSideCount = new YoInteger("userInterfaceSideCount", userInterfaceWritableRegistry);
   public final YoBoolean abortClicked = new YoBoolean("abortClicked", userInterfaceWritableRegistry);
   public final YoBoolean validClicked = new YoBoolean("validClicked", userInterfaceWritableRegistry);

   //BEHAVIOR SIDE YOVARS
   public final YoEnum<PickUpBallBehaviorState> currentState = new YoEnum<PickUpBallBehaviorState>("currentPickUpState", machineWritableRegistry,
         PickUpBallBehaviorState.class);
   public final YoInteger machineSideCount = new YoInteger("machineSideCount", machineWritableRegistry);
   public final YoInteger abortCount = new YoInteger("abortCount", machineWritableRegistry);
   public final YoBoolean abortAcknowledged = new YoBoolean("abortAcknowledged", machineWritableRegistry);
   public final YoBoolean searchingForBall = new YoBoolean("searchingForBall", machineWritableRegistry);
   public final YoBoolean foundBall = new YoBoolean("foundBall", machineWritableRegistry);
   public final YoDouble ballX = new YoDouble("ballX", machineWritableRegistry);
   public final YoDouble ballY = new YoDouble("ballY", machineWritableRegistry);
   public final YoDouble ballZ = new YoDouble("ballZ", machineWritableRegistry);
   public final YoDouble ballRadius = new YoDouble("ballRadius", machineWritableRegistry);
   public final YoBoolean validAcknowledged = new YoBoolean("validAcknowledged", machineWritableRegistry);
   public final YoBoolean waitingForValidation = new YoBoolean("waitingForValidation", machineWritableRegistry);

}
