package us.ihmc.humanoidBehaviors.ui;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;

public class BehaviorUIMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();


   private static final CategoryTheme Any = apiFactory.createCategoryTheme("Any");
   private static final CategoryTheme EditMode = apiFactory.createCategoryTheme("EditMode");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final CategoryTheme PositionTheme = apiFactory.createCategoryTheme("PositionTheme");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("BehaviorUI"));

   public static final Topic<Boolean> EditModeEnabledTopic = Root.child(Any).child(EditMode).topic(Enable);
   public static final Topic<Boolean> WaypointAPositionEditModeEnabledTopic = Root.child(Start).child(EditMode).child(PositionTheme).topic(Enable);
   public static final Topic<Boolean> WaypointBPositionEditModeEnabledTopic = Root.child(Goal).child(EditMode).child(PositionTheme).topic(Enable);
}
