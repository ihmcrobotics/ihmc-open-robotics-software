package us.ihmc.lerobot;

import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class VLAYoRegistry
{
   protected final YoRegistry registry = new YoRegistry(VLAUpdateThread.class.getSimpleName());
   protected final SideDependentList<YoPose3D> stateHandPoses = new SideDependentList<>(side -> new YoPose3D(side.getLowerCaseName() + "StateHandPose", registry));
   protected final SideDependentList<YoPose3D> stateForearmPoses = new SideDependentList<>(side -> new YoPose3D(side.getLowerCaseName() + "StateForearmPose", registry));
   protected final SideDependentList<YoPose3D> actionHandPoses = new SideDependentList<>(side -> new YoPose3D(side.getLowerCaseName() + "ActionHandPose", registry));
   protected final SideDependentList<YoPose3D> actionForearmPoses = new SideDependentList<>(side -> new YoPose3D(side.getLowerCaseName() + "ActionForearmPose", registry));
   protected final YoLong actionTimestampNanos = new YoLong("actionTimestampNanos", registry);
   protected final YoLong numberOfActionsReceived = new YoLong("numberOfActionsReceived", registry);
   protected final YoLong numberOfActionsTaken = new YoLong("numberOfActionsTaken", registry);

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
