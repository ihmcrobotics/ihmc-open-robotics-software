package us.ihmc.robotEnvironmentAwareness.communication;

import java.awt.image.BufferedImage;
import java.util.List;

import boofcv.struct.calib.CameraPinholeBrown;
import perception_msgs.msg.dds.Image32;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionData;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.ObjectType;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.PlanarRegionPropagationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;

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
   private static final CategoryTheme Camera = apiFactory.createCategoryTheme("Camera");
   private static final CategoryTheme Object = apiFactory.createCategoryTheme("Object");
   private static final CategoryTheme Socket = apiFactory.createCategoryTheme("Socket");
   private static final CategoryTheme ImageSegmentation = apiFactory.createCategoryTheme("ImageSegmentation");
   private static final CategoryTheme DataFiltering = apiFactory.createCategoryTheme("DataFiltering");
   private static final CategoryTheme StereoREA = apiFactory.createCategoryTheme("StereoREA");
   private static final CategoryTheme FusionData = apiFactory.createCategoryTheme("RawData");
   private static final CategoryTheme Result = apiFactory.createCategoryTheme("Result");
   private static final CategoryTheme Buffer = apiFactory.createCategoryTheme("Buffer");

   private static final TypedTopicTheme<Boolean> SnapShot = apiFactory.createTypedTopicTheme("SnapShot");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   private static final TypedTopicTheme<Boolean> Request = apiFactory.createTypedTopicTheme("Request");
   private static final TypedTopicTheme<String> Address = apiFactory.createTypedTopicTheme("Address");
   private static final TypedTopicTheme<List<RegionOfInterest>> ROIs = apiFactory.createTypedTopicTheme("ROIs");
   private static final TypedTopicTheme<List<ObjectType>> ObjecTypes = apiFactory.createTypedTopicTheme("ObjecTypes");
   private static final TypedTopicTheme<String> Time = apiFactory.createTypedTopicTheme("Time");
   private static final TypedTopicTheme<Integer> Size = apiFactory.createTypedTopicTheme("Size");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");
   private static final TypedTopicTheme<Quaternion> Orientation = apiFactory.createTypedTopicTheme("Orientation");
   private static final TypedTopicTheme<CameraPinholeBrown> CameraPinholeBrown = apiFactory.createTypedTopicTheme("CameraPinholeBrown");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   public static final Topic<Boolean> EnableStreaming = UICategory.child(Image).topic(Enable);
   public static final Topic<Boolean> TakeSnapShot = UICategory.child(Image).topic(SnapShot);
   public static final Topic<Boolean> ClearSnapShot = UICategory.child(Image).topic(Clear);
   public static final Topic<Boolean> ClearObjects = UICategory.child(Object).topic(Clear);

   public static final Topic<Image32> ImageState = ModuleCategory.child(Image).topic(Data);
   public static final Topic<BufferedImage> ImageResultState = ModuleCategory.child(Image).child(Result).topic(Data);
   public static final Topic<LidarImageFusionData> FusionDataState = ModuleCategory.child(StereoREA).child(FusionData).topic(Data);
   
   public static final Topic<Point3D> CameraPositionState = ModuleCategory.child(Camera).topic(Position);
   public static final Topic<Quaternion> CameraOrientationState = ModuleCategory.child(Camera).topic(Orientation);
   public static final Topic<CameraPinholeBrown> CameraIntrinsicParametersState = ModuleCategory.child(Camera).topic(CameraPinholeBrown);

   public static final Topic<Boolean> RequestSocketConnection = ObjectDetectionCategory.child(Socket).topic(Request);
   public static final Topic<Boolean> RequestObjectDetection = ObjectDetectionCategory.topic(Request);
   public static final Topic<String> ObjectDetectionModuleAddress = ObjectDetectionCategory.child(Socket).topic(Address);
   public static final Topic<List<ObjectType>> SelectedObjecTypes = ObjectDetectionCategory.topic(ObjecTypes);
   public static final Topic<List<RegionOfInterest>> ReceivedROIs = ObjectDetectionCategory.topic(ROIs);

   public static final Topic<Boolean> EnableREA = UICategory.child(StereoREA).topic(Enable);
   public static final Topic<Boolean> ClearREA = UICategory.child(StereoREA).topic(Clear);
   public static final Topic<String> ComputationTime = UICategory.child(StereoREA).topic(Time);

   public static final Topic<Integer> StereoBufferSize = UICategory.child(StereoREA).child(Buffer).topic(Size);
   public static final Topic<Boolean> ShowFusionData = UICategory.child(StereoREA).child(FusionData).topic(Request);
   public static final Topic<Boolean> ShowStereoBufferProjection = UICategory.child(StereoREA).child(Buffer).topic(SnapShot);
   public static final Topic<Boolean> RunStereoREA = UICategory.child(StereoREA).topic(Request);

   public static final Topic<SegmentationRawDataFilteringParameters> SegmentationRawDataFilteringParameters = UICategory.child(DataFiltering).topic(Parameters);
   public static final Topic<ImageSegmentationParameters> ImageSegmentationParameters = UICategory.child(ImageSegmentation).topic(Parameters);
   public static final Topic<PlanarRegionPropagationParameters> PlanarRegionPropagationParameters = UICategory.child(StereoREA).topic(Parameters);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}