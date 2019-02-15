package us.ihmc.humanoidBehaviors.ui;

import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class BehaviorUIMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme Any = apiFactory.createCategoryTheme("Any");
   private static final CategoryTheme EditMode = apiFactory.createCategoryTheme("EditMode");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final CategoryTheme PositionTheme = apiFactory.createCategoryTheme("PositionTheme");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");
   private static final CategoryTheme GoHome = apiFactory.createCategoryTheme("GoHome");
   private static final TypedTopicTheme<GoHomeMessage> GoHomeMessage = apiFactory.createTypedTopicTheme("GoHomeMessage");
   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("BehaviorUI"));

   public static final Topic<Boolean> EditModeEnabledTopic = Root.child(Any).child(EditMode).topic(Enable);
   public static final Topic<Boolean> WaypointAPositionEditModeEnabledTopic = Root.child(Start).child(EditMode).child(PositionTheme).topic(Enable);
   public static final Topic<Boolean> WaypointBPositionEditModeEnabledTopic = Root.child(Goal).child(EditMode).child(PositionTheme).topic(Enable);
   public static final Topic<GoHomeMessage> GoHomeTopic = Root.child(GoHome).topic(GoHomeMessage);
   public static final Topic<PlanarRegionsList> PlanarRegionDataTopic = Root.child(PlanarRegion).topic(Data);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
