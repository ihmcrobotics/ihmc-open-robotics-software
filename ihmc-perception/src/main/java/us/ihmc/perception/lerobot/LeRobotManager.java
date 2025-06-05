package us.ihmc.perception.lerobot;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import sensor_msgs.msg.dds.Image;
import std_msgs.msg.dds.Float32MultiArray;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class LeRobotManager
{
   private static final ROS2Topic<?> LEROBOT = new ROS2Topic<>().withPrefix("lerobot");
   private static final ROS2Topic<std_msgs.msg.dds.String> CONNECT = LEROBOT.withSuffix("connect").withType(std_msgs.msg.dds.String.class);
   private static final ROS2Topic<std_msgs.msg.dds.String> COMMAND = LEROBOT.withSuffix("command").withType(std_msgs.msg.dds.String.class);
   private static final ROS2Topic<std_msgs.msg.dds.String> STATUS = LEROBOT.withSuffix("status").withType(std_msgs.msg.dds.String.class);
   private static final SideDependentList<ROS2Topic<Image>> ZED_IMAGES = new SideDependentList<>(LEROBOT.withSuffix("/zed/left/color").withType(Image.class),
                                                                                                 LEROBOT.withSuffix("/zed/right/color").withType(Image.class));
   private static final ROS2Topic<Float32MultiArray> STATE_HAND_POSES = LEROBOT.withSuffix("/lerobot/state/hand_poses").withType(Float32MultiArray.class);
   private static final ROS2Topic<Float32MultiArray> ACTION_HAND_POSES = LEROBOT.withSuffix("/lerobot/action/hand_poses").withType(Float32MultiArray.class);

   private final String modelName;
   private final RepeatingTaskThread thread = new RepeatingTaskThread("LeRobotROS2Thread", this::update);
   private final ROS2Node ros2Node = new ROS2NodeBuilder().domainId(185).build("lerobot_java_side");
   private final ROS2Helper ros2 = new ROS2Helper(ros2Node);
   private final std_msgs.msg.dds.String command = new std_msgs.msg.dds.String();
   private final std_msgs.msg.dds.String status = new std_msgs.msg.dds.String();
   private final Float32MultiArray stateHandPosesMessage = new Float32MultiArray();
   private final FrequencyCalculator statusFrequency = new FrequencyCalculator();
   private final SideDependentList<Pose3D> actionHandPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Image> zedImages = new SideDependentList<>(new Image(), new Image());
   private boolean running = false;

   public LeRobotManager(String modelName)
   {
      this.modelName = modelName;

      ros2.subscribeViaVolatileCallback(STATUS, message ->
      {
         status.set(message);
         statusFrequency.ping();
      });
      ros2.subscribeViaVolatileCallback(ACTION_HAND_POSES, message ->
      {
         IDLSequence.Float data = message.getData();
         int i = 0;
         for (RobotSide side : RobotSide.values)
         {
            Pose3D pose = actionHandPoses.get(side);
            pose.getPosition().set(data.get(i++), data.get(i++), data.get(i++));
            pose.getOrientation().set(data.get(i++), data.get(i++), data.get(i++), data.get(i++));
         }
      });

      thread.setFrequencyLimit(20.0);
      thread.startRepeating();
   }

   public void publishHandPoses(Pose3DReadOnly leftPose, Pose3DReadOnly rightPose)
   {
      int i = 0;
      IDLSequence.Float data = stateHandPosesMessage.getData();
      data.resetQuick();
      for (RobotSide side : RobotSide.values)
      {
         data.add((side == RobotSide.LEFT ? leftPose : rightPose).getPosition().getX32());
         data.add((side == RobotSide.LEFT ? leftPose : rightPose).getPosition().getY32());
         data.add((side == RobotSide.LEFT ? leftPose : rightPose).getPosition().getZ32());
         data.add((side == RobotSide.LEFT ? leftPose : rightPose).getOrientation().getX32());
         data.add((side == RobotSide.LEFT ? leftPose : rightPose).getOrientation().getY32());
         data.add((side == RobotSide.LEFT ? leftPose : rightPose).getOrientation().getZ32());
         data.add((side == RobotSide.LEFT ? leftPose : rightPose).getOrientation().getS32());
      }
      ros2.publish(STATE_HAND_POSES, stateHandPosesMessage);
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

      ros2.publish(ZED_IMAGES.get(side), message);
   }

   private void update()
   {
      command.setData(running ? "diffusion" : ""); // TODO: In future, possibly request model name
      ros2.publish(COMMAND, command);
   }

   public void setRunning(boolean running)
   {
      this.running = running;
   }

   public void startPythonServer()
   {
      std_msgs.msg.dds.String startServer = new std_msgs.msg.dds.String();
      startServer.setData("connect");
      ros2.publish(CONNECT, startServer);
   }

   public double getStatusFrequency()
   {
      return statusFrequency.getFrequencyDecaying();
   }

   public SideDependentList<Pose3D> getActionHandPoses()
   {
      return actionHandPoses;
   }

   public void destroy()
   {
      ros2Node.destroy();
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
