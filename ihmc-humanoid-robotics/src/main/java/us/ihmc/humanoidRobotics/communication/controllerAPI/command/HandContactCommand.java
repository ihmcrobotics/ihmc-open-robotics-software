package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.HandContactMessage;
import ihmc_common_msgs.msg.dds.Point2DMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandContactCommand implements Command<HandContactCommand, HandContactMessage>
{
   private long sequenceId;
   private boolean load;
   private double trajectoryDuration;
   private RobotSide robotSide;
   private final FramePoint3D bracingPoint = new FramePoint3D();
   private final FrameVector3D bracingNormal = new FrameVector3D();
   private final RecyclingArrayList<Point2D> supportRegionsPointsInMidFeetZUp = new RecyclingArrayList<>(Point2D.class);

   private final RigidBodyTransform regionTransformFromWorld = new RigidBodyTransform();
   private final ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

   @Override
   public void clear()
   {
      robotSide = null;
      load = false;
      trajectoryDuration = Double.NaN;
      bracingPoint.setToNaN();
      bracingNormal.setToNaN();
      supportRegionsPointsInMidFeetZUp.clear();

      regionTransformFromWorld.setToNaN();
      convexPolygon.clear();
   }

   @Override
   public void setFromMessage(HandContactMessage message)
   {
      sequenceId = message.getSequenceId();
      load = message.getLoad();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      bracingPoint.set(ReferenceFrame.getWorldFrame(), message.getBracingPoint());
      bracingNormal.set(ReferenceFrame.getWorldFrame(), message.getBracingNormal());

      for (int i = 0; i < message.getSupportRegionInMidFeetFrame().size(); i++)
      {
         Point2DMessage vertexMessage = message.getSupportRegionInMidFeetFrame().get(i);
         supportRegionsPointsInMidFeetZUp.add().set(vertexMessage.getX(), vertexMessage.getY());
      }

      MessageTools.toEuclid(message.getRegionTransform(), regionTransformFromWorld);

      for (int i = 0; i < message.getScaledConvexHull().size(); i++)
      {
         Point2DMessage vertex = message.getScaledConvexHull().get(i);
         convexPolygon.addVertex(vertex.getX(), vertex.getY());
      }
      convexPolygon.update();
   }

   @Override
   public Class<HandContactMessage> getMessageClass()
   {
      return HandContactMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && !bracingPoint.containsNaN() && !bracingNormal.containsNaN() && trajectoryDuration > 0.0;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(HandContactCommand other)
   {
      robotSide = other.robotSide;
      load = other.load;
      trajectoryDuration = other.trajectoryDuration;
      bracingPoint.set(other.bracingPoint);
      bracingNormal.set(other.bracingNormal);

      supportRegionsPointsInMidFeetZUp.clear();
      for (int i = 0; i < other.supportRegionsPointsInMidFeetZUp.size(); i++)
      {
         supportRegionsPointsInMidFeetZUp.add().set(other.supportRegionsPointsInMidFeetZUp.get(i));
      }

      regionTransformFromWorld.set(other.regionTransformFromWorld);
      convexPolygon.set(other.convexPolygon);
   }

   public void setLoad(boolean load)
   {
      this.load = load;
   }

   public boolean load()
   {
      return load;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public FramePoint3DBasics getBracingPoint()
   {
      return bracingPoint;
   }

   public FrameVector3DBasics getBracingNormal()
   {
      return bracingNormal;
   }

   public boolean hasSupportRegion()
   {
      return !supportRegionsPointsInMidFeetZUp.isEmpty();
   }

   public RecyclingArrayList<Point2D> getSupportRegionsPointsInMidFeetZUp()
   {
      return supportRegionsPointsInMidFeetZUp;
   }

   public RigidBodyTransform getRegionTransformFromWorld()
   {
      return regionTransformFromWorld;
   }

   public ConvexPolygon2D getConvexPolygon()
   {
      return convexPolygon;
   }
}
