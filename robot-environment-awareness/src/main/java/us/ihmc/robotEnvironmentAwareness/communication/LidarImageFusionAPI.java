package us.ihmc.robotEnvironmentAwareness.communication;

import java.awt.image.BufferedImage;
import java.util.List;

import controller_msgs.msg.dds.ImageMessage;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.ObjectType;

public class LidarImageFusionAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("LidarImageFusion"));

   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");
   private static final CategoryTheme UI = apiFactory.createCategoryTheme("UserInterface");
   private static final CategoryTheme ObjectDetection = apiFactory.createCategoryTheme("ObjectDetection");

   private static final Category ModuleCategory = Root.child(Module);
   private static final Category UICategory = Root.child(UI);
   private static final Category ObjectDetectionCategory = Root.child(ObjectDetection);

   private static final CategoryTheme Image = apiFactory.createCategoryTheme("Image");
   private static final CategoryTheme Socket = apiFactory.createCategoryTheme("Socket");
   
   private static final CategoryTheme Result = apiFactory.createCategoryTheme("Result");

   private static final TypedTopicTheme<Boolean> SnapShot = apiFactory.createTypedTopicTheme("SnapShot");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   private static final TypedTopicTheme<Boolean> Request = apiFactory.createTypedTopicTheme("Request");
   private static final TypedTopicTheme<String> Address = apiFactory.createTypedTopicTheme("Address");
   private static final TypedTopicTheme<List<RegionOfInterest>> ROIs = apiFactory.createTypedTopicTheme("ROIs");
   private static final TypedTopicTheme<List<ObjectType>> ObjecTypes = apiFactory.createTypedTopicTheme("ObjecTypes");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   public static final Topic<Boolean> EnableStreaming = UICategory.child(Image).topic(Enable);
   public static final Topic<Boolean> ImageSnapShot = UICategory.child(Image).topic(SnapShot);
   public static final Topic<Boolean> ImageViewClear = UICategory.child(Image).topic(Clear);

   public static final Topic<ImageMessage> ImageState = ModuleCategory.child(Image).topic(Data);
   public static final Topic<BufferedImage> ImageResultState = ModuleCategory.child(Image).child(Result).topic(Data);

   public static final Topic<Boolean> RequestSocketConnection = ObjectDetectionCategory.child(Socket).topic(Request);
   public static final Topic<Boolean> RequestObjectDetection = ObjectDetectionCategory.topic(Request);
   public static final Topic<String> ObjectDetectionModuleAddress = ObjectDetectionCategory.child(Socket).topic(Address);
   public static final Topic<List<ObjectType>> SelectedObjecTypes = ObjectDetectionCategory.topic(ObjecTypes);
   public static final Topic<List<RegionOfInterest>> ReceivedROIs = ObjectDetectionCategory.topic(ROIs);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}