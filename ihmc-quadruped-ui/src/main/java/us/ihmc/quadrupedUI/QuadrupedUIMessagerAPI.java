package us.ihmc.quadrupedUI;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotModels.FullQuadrupedRobotModel;

public class QuadrupedUIMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme Robot = apiFactory.createCategoryTheme("Robot");

   private static final TypedTopicTheme<FullQuadrupedRobotModel> RobotModel = apiFactory.createTypedTopicTheme("RobotModel");
   private static final TypedTopicTheme<QuadrupedReferenceFrames> ReferenceFrames = apiFactory.createTypedTopicTheme("ReferenceFrames");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("QuadrupedUI"));

   public static final Topic<FullQuadrupedRobotModel> RobotModelTopic = Root.child(Robot).topic(RobotModel);
   public static final Topic<QuadrupedReferenceFrames> ReferenceFramesTopic = Root.child(Robot).topic(ReferenceFrames);
}
