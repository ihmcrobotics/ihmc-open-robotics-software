package us.ihmc.perception.lerobot;

import gnu.trove.list.array.TDoubleArrayList;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Float32MultiArray;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.function.Consumer;

/**
 * Manages communication with the Python side, which is running the LeRobot code
 * with pytorch inference of the visuomotor policy. We use a ROS 2 API to interface with it.
 */
public class LeRobotInferenceManager
{
   private static final ROS2Topic<?> LEROBOT = new ROS2Topic<>().withPrefix("lerobot");
   private static final ROS2Topic<std_msgs.msg.dds.String> CONNECT = LEROBOT.withSuffix("connect").withType(std_msgs.msg.dds.String.class);
   private static final ROS2Topic<std_msgs.msg.dds.String> COMMAND = LEROBOT.withSuffix("command").withType(std_msgs.msg.dds.String.class);
   private static final ROS2Topic<std_msgs.msg.dds.String> STATUS = LEROBOT.withSuffix("status").withType(std_msgs.msg.dds.String.class);
   private static final ROS2Topic<Float32MultiArray> STATE = LEROBOT.withSuffix("/lerobot/state").withType(Float32MultiArray.class);
   public static final ROS2Topic<Float32MultiArray> ACTION = LEROBOT.withSuffix("/lerobot/action").withType(Float32MultiArray.class);

   private final ROS2Helper ros2Helper;

   private final String modelName;
   private final boolean useHandPoses;
   private final RepeatingTaskThread thread = new RepeatingTaskThread("LeRobotROS2Thread", this::update);
   private final std_msgs.msg.dds.String command = new std_msgs.msg.dds.String();
   private final std_msgs.msg.dds.String status = new std_msgs.msg.dds.String();
   private final Float32MultiArray stateMessage = new Float32MultiArray();
   private final FrequencyCalculator statusFrequency = new FrequencyCalculator();
   private final SideDependentList<Pose3D> actionHandPoses = new SideDependentList<>(new Pose3D(), new Pose3D()); // TODO
   private final TDoubleArrayList actionJointAngles = new TDoubleArrayList();
   private final SideDependentList<Image> zedImages = new SideDependentList<>(new Image(), new Image());
   private boolean running = false;
   private final LeRobotIKStreaming ikStreaming;
   private long actionTimestampNanos = 0L;
   private long numberOfActionsReceived = 0L;

   public LeRobotInferenceManager(String modelName, String robotName, FullHumanoidRobotModel fullRobotModel, ROS2Node ros2Node, boolean useHandPoses)
   {
      this.modelName = modelName;
      this.useHandPoses = useHandPoses;

      actionHandPoses.forEach(Pose3D::setToNaN);
      ikStreaming = new LeRobotIKStreaming(actionHandPoses, actionJointAngles, useHandPoses, robotName, ros2Node, fullRobotModel);
      ros2Helper = new ROS2Helper(ros2Node);
      ros2Helper.subscribeViaVolatileCallback(STATUS, message ->
      {
         status.set(message);
         statusFrequency.ping();
      });
      ros2Helper.subscribeViaVolatileCallback(ACTION, message ->
      {
         // TODO: Add timestamp to /lerobot/action/hand_poses topic or something
         //   This should be calculated according to the policy output
         actionTimestampNanos = System.nanoTime();
         ++numberOfActionsReceived;

         IDLSequence.Float data = message.getData();
         if (useHandPoses)
         {
            int i = 0;
            for (RobotSide side : RobotSide.values)
            {
               Pose3D pose = actionHandPoses.get(side);
               pose.getPosition().set(data.get(i++), data.get(i++), data.get(i++));
               pose.getOrientation().set(data.get(i++), data.get(i++), data.get(i++), data.get(i++));
            }
         }
         else
         {
            actionJointAngles.clear();
            for (int i = 0; i < data.size(); i++)
               actionJointAngles.add(data.get(i));
         }
      });
   }

   /** Optionally start a thread for calling update. */
   public void startUpdateThread()
   {
      thread.setFrequencyLimit(20.0);
      thread.startRepeating();
   }

   public void publishState(Consumer<IDLSequence.Float> stateSetter)
   {
      IDLSequence.Float messageData = stateMessage.getData();
      messageData.resetQuick();
      stateSetter.accept(messageData);
      ros2Helper.publish(STATE, stateMessage);
   }

   public void publishImage(RobotSide side, Mat bgra8Mat)
   {
      Mat bgr8Mat = new Mat(bgra8Mat.rows(), bgra8Mat.cols(), opencv_core.CV_8UC3);
      opencv_imgproc.cvtColor(bgra8Mat, bgr8Mat, opencv_imgproc.COLOR_BGRA2BGR);

      Image message = zedImages.get(side);
      message.setWidth(bgra8Mat.cols());
      message.setHeight(bgra8Mat.rows());
      message.setStep(bgr8Mat.step());
      message.setEncoding("bgr8");

      int memorySize = (int) OpenCVTools.memorySize(bgr8Mat);
      ByteBuffer dataBuffer = message.getData().getBuffer();
      message.setIsBigendian((byte) (dataBuffer.order().equals(ByteOrder.BIG_ENDIAN) ? 1 : 0));
      dataBuffer.position(0).put(bgr8Mat.data().limit(memorySize).asByteBuffer());

      bgr8Mat.close();

      ros2Helper.publish(PerceptionAPI.ROS2_ZED_COLOR_IMAGES.get(side), message);
   }

   public void update()
   {
      command.setData(running ? "diffusion" : ""); // TODO: In future, possibly request model name
      ros2Helper.publish(COMMAND, command);

      if (running)
      {
         ikStreaming.update(actionTimestampNanos);
      }
   }

   public void setRunning(boolean running)
   {
      this.running = running;
   }

   public LeRobotIKStreaming getIKStreaming()
   {
      return ikStreaming;
   }

   public void startPythonServer()
   {
      std_msgs.msg.dds.String startServer = new std_msgs.msg.dds.String();
      startServer.setData("connect");
      ros2Helper.publish(CONNECT, startServer);
   }

   public double getStatusFrequency()
   {
      return statusFrequency.getFrequencyDecaying();
   }

   public String getStatusMessage()
   {
      return status.getDataAsString();
   }

   public long getNumberOfActionsReceived()
   {
      return numberOfActionsReceived;
   }

   public SideDependentList<Pose3D> getActionHandPoses()
   {
      return actionHandPoses;
   }

   public void destroy()
   {
      thread.kill();
   }

   public String getModelName()
   {
      return modelName;
   }

   public boolean isRunning()
   {
      return running;
   }
}
