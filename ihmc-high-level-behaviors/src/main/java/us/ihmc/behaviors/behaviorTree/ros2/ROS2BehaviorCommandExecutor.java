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
    private final Queue<BehaviorCommandMessage> pending = new ConcurrentLinkedQueue<>();
    private final Queue<Byte> deferredStops = new ConcurrentLinkedQueue<>();
    private final Queue<Byte> deferredTreeCancels = new ConcurrentLinkedQueue<>();
    private final Object scheduleLock = new Object();
    private volatile Starter mimicStarter;
    private volatile Starter followStarter;
    private volatile Consumer<Byte> stopKind;
    private Consumer<BehaviorResultMessage> resultSink;
    private byte activeKind;
    private boolean hasActive;
    private boolean draining;

    public ROS2BehaviorCommandExecutor(
            ROS2ControllerHelper helper,
            ROS2BehaviorTreeExecutor treeExecutor)
    {
        ros2ControllerHelper = helper;
        ros2BehaviorTreeExecutor = treeExecutor;
        resultPublisher = ros2ControllerHelper.getROS2Node().createPublisher(AutonomyAPI.BEHAVIOR_RESULT);
        commandSubscription = ros2ControllerHelper.getROS2Node().createSubscription(AutonomyAPI.BEHAVIOR_COMMAND, reader -> commands.add(new BehaviorCommandMessage(reader.read())));
    }

    /** No ROS node. Tests enqueue commands and read results through {@code resultSink}. */
    ROS2BehaviorCommandExecutor(Consumer<BehaviorResultMessage> resultSink)
    {
        ros2ControllerHelper = null;
        ros2BehaviorTreeExecutor = null;
        resultPublisher = null;
        commandSubscription = null;
        this.resultSink = resultSink;
    }

    void enqueue(BehaviorCommandMessage command)
    {
        commands.add(new BehaviorCommandMessage(command));
    }

    public void setMimicStarter(Starter starter){
        mimicStarter = starter;
    }

    public void setFollowStarter(Starter starter){
        followStarter = starter;
    }

    public void setStopKind(Consumer<Byte> stopKind){
        this.stopKind = stopKind;
    }

    public void update(){
        synchronized(scheduleLock){
            Byte treeCancel;
            while((treeCancel = deferredTreeCancels.poll()) != null){
                if(ros2BehaviorTreeExecutor != null)
                    ros2BehaviorTreeExecutor.cancelActive(treeCancel);
            }
            stageIncoming();
            applyMatching(true);
        }
    }

    /**
     * Starts queued mimic and follow commands. Called from the command tick, not from {@link #update()}.
     * A tree waiting at the head of the queue does not block them, and a mimic or follow does not block a tree.
     */
    public void drain(){
        synchronized(scheduleLock){
            if(draining)
                return;
            draining = true;
            try{
                stageIncoming();
                flushDeferredStops();
                applyMatching(false);
            }
            finally{
                draining = false;
            }
        }
    }

    /**
     * {@code trees} applies {@link BehaviorCommandMessage#TREE} and leaves every other kind queued, in order.
     * Otherwise applies every other kind and leaves trees queued.
     */
    private void applyMatching(boolean trees){
        int queued = pending.size();
        for(int i = 0; i < queued; i++){
            BehaviorCommandMessage command = pending.poll();
            if(command == null)
                return;
            if((command.getKind() == BehaviorCommandMessage.TREE) == trees)
                apply(command);
            else
                pending.add(command);
        }
    }

    private void stageIncoming(){
        BehaviorCommandMessage command;
        while((command = commands.poll()) != null)
            pending.add(command);
    }

    private void flushDeferredStops(){
        Byte kind;
        while((kind = deferredStops.poll()) != null){
            if(stopKind != null)
                stopKind.accept(kind);
        }
    }

    public void destroy(){
        if(commandSubscription != null && ros2ControllerHelper != null){
            ros2ControllerHelper.getROS2Node().destroySubscription(commandSubscription);
        }
        if(ros2BehaviorTreeExecutor != null)
            ros2BehaviorTreeExecutor.destroy();
    }

    private void apply(BehaviorCommandMessage command){
        if(command.getNameAsString().isBlank()){
            publish(command.getRequestId(), BehaviorResultMessage.REJECTED, "Missing name", true);
            return;
        }

        if(command.getKind() == BehaviorCommandMessage.TREE){
            if(!(hasActive && activeKind == BehaviorCommandMessage.TREE))
                cancelActive(command.getKind());
            int phase = ros2BehaviorTreeExecutor.startTree(command);
            String treeName = command.getNameAsString();
            if(phase != BehaviorResultMessage.ACCEPTED){
                publish(command.getRequestId(), (byte) phase, treeName, true);
                return;
            }
            remember(command);
            publish(command.getRequestId(), BehaviorResultMessage.ACCEPTED, treeName, false);
            return;
        }

        Starter starter = command.getKind() == BehaviorCommandMessage.FOLLOW ? followStarter
                : command.getKind() == BehaviorCommandMessage.MIMIC ? mimicStarter
                : null;

        if(starter == null){
            publish(command.getRequestId(), BehaviorResultMessage.REJECTED, "Unknown Command", true);
            return;
        }

        cancelActive(command.getKind());
        StartResult result = start(starter, command);
        if(result.rejected()){
            publish(command.getRequestId(), result.phase(), result.summary(), true);
            return;
        }

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
        byte previous = activeKind;
        hasActive = false;
        if(previous == BehaviorCommandMessage.TREE){
            if(draining)
                deferredTreeCancels.add(newKind);
            else if(ros2BehaviorTreeExecutor != null)
                ros2BehaviorTreeExecutor.cancelActive(newKind);
            return;
        }
        if(stopKind == null)
            return;
        if(draining)
            stopKind.accept(previous);
        else
            deferredStops.add(previous);
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
        if(resultSink != null)
            resultSink.accept(message);
        else if(resultPublisher != null)
            resultPublisher.publish(message);
    }

    @FunctionalInterface
    public interface Starter{
        StartResult start(long requestId, String name, boolean closestPerson);
    }

    public record StartResult(byte phase, String summary, boolean rejected) {}
}
