package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.BehaviorCommandMessage;
import behavior_msgs.BehaviorResultMessage;
import behavior_msgs.BehaviorTreeYoDataMessage;
import org.apache.commons.lang3.function.TriFunction;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8Detection;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeExecutor;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

import static us.ihmc.communication.AutonomyAPI.BEHAVIOR_TREE_MODULE;

public class ROS2BehaviorCommandExecutor{
    private final ROS2ControllerHelper ros2ControllerHelper;
    private final ROS2BehaviorTreeExecutor ros2BehaviorTreeExecutor;
    private final ROS2Publisher<BehaviorResultMessage> resultPublisher;
    private final ROS2Subscription<BehaviorCommandMessage> commandSubscription;
    private final Queue<BehaviorCommandMessage> commands = new ConcurrentLinkedQueue<>();
    private volatile Starter mimicStarter;
    private volatile Starter followStarter;
    private volatile Starter treeStarter;
    private volatile Consumer<Byte> stopKind;
    private byte activeKind;
    private boolean hasActive;
    private boolean running;

    public ROS2BehaviorCommandExecutor(
            ROS2ControllerHelper helper,
            ROS2BehaviorTreeExecutor treeExecutor)
    {
        ros2ControllerHelper = helper;
        ros2BehaviorTreeExecutor = treeExecutor;
        resultPublisher = ros2ControllerHelper.getROS2Node().createPublisher(AutonomyAPI.BEHAVIOR_RESULT);
        commandSubscription = ros2ControllerHelper.getROS2Node().createSubscription(AutonomyAPI.BEHAVIOR_COMMAND, reader -> commands.add(new BehaviorCommandMessage(reader.read())));
    }

    public void setMimicStarter(Starter starter){
        mimicStarter = starter;
    }

    public void setFollowStarter(Starter starter){
        followStarter = starter;
    }

    public void setTreeStarter(Starter starter){
        treeStarter = starter;
    }

    public void setStopKind(Consumer<Byte> stopKind){
        this.stopKind = stopKind;
    }

    public void update(){
        if(!running){
            running = true;
            try{
                for(BehaviorCommandMessage command : commands){
                    if(command != null) {
                        apply(command);
                    }
                }
            }
            finally{
                running = false;
            }
        }
    }

    public void destroy(){
        if(commandSubscription != null){
            ros2ControllerHelper.getROS2Node().destroySubscription(commandSubscription);
        }
        ros2BehaviorTreeExecutor.destroy();
    }

    private void apply(BehaviorCommandMessage command){
        if(command.getNameAsString().isBlank()){
            publish(command.getRequestId(), BehaviorResultMessage.REJECTED, "Missing name", true);
            return;
        }

        if(command.getKind() == BehaviorCommandMessage.TREE){
            ros2BehaviorTreeExecutor.startTree(command);
            return;
        }

        Starter starter = command.getKind() == BehaviorCommandMessage.FOLLOW ? followStarter
                : command.getKind() == BehaviorCommandMessage.MIMIC ? mimicStarter
                : null;

        if(starter == null){
            publish(command.getRequestId(), BehaviorResultMessage.REJECTED, "Unknown Command", true);
            return;
        }

        StartResult result = start(starter, command);
        if(result.rejected()){
            publish(command.getRequestId(), result.phase(), result.summary(), true);
            return;
        }
        cancelActive(command.getKind());
        if(result.phase() != BehaviorResultMessage.SUCCEEDED){
            remember(command);
        }
        publish(command.getRequestId(), result.phase(), result.summary(), false);
    }

    private StartResult start(Starter starter, BehaviorCommandMessage command){
        if(starter == null){
            return new StartResult(BehaviorResultMessage.REJECTED, "Starter not selected", true);
        }
        return starter.start(command.getRequestId(), command.getNameAsString(), command.getClosestPerson());
    }

    private void cancelActive(byte newKind){
        if(!hasActive){
            return;
        }
        if(activeKind == BehaviorCommandMessage.TREE){
            ros2BehaviorTreeExecutor.cancelActive(newKind);
        }
        else if (activeKind != newKind && stopKind != null){
            stopKind.accept(activeKind);
        }
        hasActive = false;
    }

    private void remember(BehaviorCommandMessage command){
        hasActive = true;
        activeKind = command.getKind();
    }

    private void publish(long requestId, byte phase, String summary, boolean rejected){
        BehaviorResultMessage message = new BehaviorResultMessage();
        message.setRequestId(requestId);
        message.setPhase(phase);
        message.setSummary(summary);
        message.setRejected(rejected);
        resultPublisher.publish(message);
    }

    @FunctionalInterface
    public interface Starter{
        StartResult start(long requestId, String name, boolean closestPerson);
    }

    public record StartResult(byte phase, String summary, boolean rejected) {}
}
