package us.ihmc.humanoidBehaviors.behaviors;

//~--- non-JDK imports --------------------------------------------------------

//~--- JDK imports ------------------------------------------------------------
import java.io.InputStream;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class TurnValveBehavior extends BehaviorInterface
{
   private final double MAX_WRIST_FORCE_THRESHOLD_N = 140.0; // Sensor reads ~ 100 N after stand-prep with Robotiq hands
   private final double MAX_CAPTURE_POINT_ERROR_M = 0.5 * 0.075; // Reasonable value < 0.01   Max < 0.02
   private final double MIN_DISTANCE_CAPTURE_POINT_TO_SUPPORT_POLYGON_M = 0.005;

   private final Vector3d valveInteractionOffsetInValveFrame = new Vector3d(-0.13, 0.0, 0.64);
   private Vector3d valveLocation = new Vector3d();
   private Vector3d valveOffsetInWorldFrame = new Vector3d();
   private final ArrayList<BehaviorInterface> behaviorQueue = new ArrayList<>();
   private final ScriptBehavior scriptBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;

   private final HandPoseBehavior handPoseBehavior;

   private Point3d targetWalkLocation;
   private final YoFrameOrientation targetWalkOrientation;
   private InputStream scriptResourceStream;
   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
   private RigidBodyTransform worldToValveTransform;
   private YoFrameOrientation valveOrientation;
   private BehaviorInterface currentBehavior;
   private boolean isDone;

   private DoubleYoVariable rightWristForceFiltered;
   private double maxObservedWristForce = 0.0;
   private final DoubleYoVariable yoTime;
   private final double DT;

   private final YoFramePoint2d yoCapturePoint;
   private final YoFramePoint2d yoDesiredCapturePoint;
   private final DoubleYoVariable capturePointErrorMag;
   private final YoFrameConvexPolygon2d yoSupportPolygon;
   private final DoubleYoVariable minIcpDistanceToSupportPolygon;
   private final DoubleYoVariable desiredMinIcpDistanceToSupportPolygon;
   
   private Double maxObservedCapturePointError = 0.0;

   // private final ModifiableValveModel valveModel;

   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, YoFramePoint2d yoCapturePoint, YoFramePoint2d yoDesiredCapturePoint, YoFrameConvexPolygon2d yoSupportPolygon, DoubleYoVariable rightWristForceFiltered, double DT)
   {
      super(outgoingCommunicationBridge);
      targetWalkLocation = new Point3d();
      valveOrientation = new YoFrameOrientation(behaviorName + "ValveOrientation", ReferenceFrame.getWorldFrame(), registry);
      targetWalkOrientation = new YoFrameOrientation(behaviorName + "WalkToOrientation", ReferenceFrame.getWorldFrame(), registry);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames);

      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<>();

      this.yoTime = yoTime;
      this.DT = DT;

      this.rightWristForceFiltered = rightWristForceFiltered;

      this.yoCapturePoint = yoCapturePoint;
      this.yoDesiredCapturePoint = yoDesiredCapturePoint;
      this.capturePointErrorMag = new DoubleYoVariable("capturePointError", registry);
      this.yoSupportPolygon = yoSupportPolygon;
      this.minIcpDistanceToSupportPolygon = new DoubleYoVariable("minIcpDistanceToSupportPolygon", registry);
      this.desiredMinIcpDistanceToSupportPolygon = new DoubleYoVariable("desiredMinIcpDistanceToSupportPolygon", registry);
      
      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
   }

   
   @Override
   public void doControl()
   {
      updateCapturePointError();
//      updateCapturePointDistanceToSupportPolygon();

      if (!currentBehavior.equals(walkToLocationBehavior))
      {
//         pauseBehaviorIfCapturePointIsTooCloseToSupportPolygonEdge();
         pauseBehaviorIfCapturePointErrorIsTooLarge();
//         pauseBehaviorIfWristForceSensorSuddenlySpikes();
      }

      if (scriptBehaviorInputPacketListener.isNewPacketAvailable())
      {
         ScriptBehaviorInputPacket scriptBehaviorInputPacket = scriptBehaviorInputPacketListener.getNewestPacket();
         System.out.println("TurnValveBehavior: New Script Behavior Input Packet Received: " + scriptBehaviorInputPacket);

         worldToValveTransform = scriptBehaviorInputPacket.getReferenceTransform();

         setValveLocationAndOrientation(worldToValveTransform);
         setTargetWalkLocationAndOrientation(valveLocation, valveOrientation);

         HandPosePacket bumpValveHandPosePacket = createHandPosePacketToBumpTheValveToVerifyItsPosition(worldToValveTransform);
         handPoseBehavior.setInput(bumpValveHandPosePacket);
         //       handPoseBehavior.setInput(Frame.WORLD, worldToValveTransformWithYawCorrection, RobotSide.RIGHT, 1.0);

         String scriptName = scriptBehaviorInputPacket.getScriptName();

         scriptResourceStream = this.getClass().getClassLoader().getResourceAsStream(scriptName);

         if (scriptResourceStream == null)
         {
            throw new RuntimeException("TurnValveBehavior: Script Resource Stream is null. Can't load script!  scriptResourceStream : " + scriptResourceStream);
         }
         else
         {
            System.out.println("TurnValveBehavior: Script " + scriptBehaviorInputPacket.getScriptName() + " loaded.");
         }

         scriptBehavior.setScriptInputs(scriptResourceStream, worldToValveTransform);
      }

      if (currentBehavior.isDone())
      {
         System.out.println("TurnValveBehavior: " + currentBehavior.getName() + " is done.");
         currentBehavior.finalize();
         if (!behaviorQueue.isEmpty())
         {
            currentBehavior = behaviorQueue.remove(0);
            currentBehavior.initialize();
            System.out.println("TurnValveBehavior: " + currentBehavior.getName() + " is starting.");
         }
         else
         {
            isDone = true;
         }
      }

      currentBehavior.doControl();
   }

   FramePoint2d tempFramePoint2d = new FramePoint2d();

   private void updateCapturePointError()
   {
      tempFramePoint2d.set(yoDesiredCapturePoint.getX(), yoDesiredCapturePoint.getY());

      double error = Math.abs(yoCapturePoint.distance(tempFramePoint2d));

      if (error > maxObservedCapturePointError)
      {
         maxObservedCapturePointError = error;
//         System.out.println("TurnValveBehavior: Max Capture Point Error : " + maxObservedCapturePointError);
      }
      capturePointErrorMag.set(error);
   }

   private Point2d icp = new Point2d();
   private Point2d centroid = new Point2d();
   
   private void updateCapturePointDistanceToSupportPolygon()
   {
      yoCapturePoint.get(icp); 
      
      ConvexPolygon2d supportPolygon = yoSupportPolygon.getConvexPolygon2d();

      double distanceToClosestEdgeOfSupportPolygon = computeDistanceToClosestEdge(icp, supportPolygon);

      minIcpDistanceToSupportPolygon.set( distanceToClosestEdgeOfSupportPolygon );
      
      desiredMinIcpDistanceToSupportPolygon.set( MIN_DISTANCE_CAPTURE_POINT_TO_SUPPORT_POLYGON_M );
      
//      System.out.println("TurnValveBehavior: icp distance to polygon edge : " + icpDistanceToSupportPolygon.getDoubleValue());
   }


   private double computeDistanceToClosestEdge(Point2d pointInsideConvexPolygon, ConvexPolygon2d convexPolygon)
   {
      double minDistanceToEdge = Double.POSITIVE_INFINITY;

      double distanceToEdge = 0.0;
      int numberOfVertices = convexPolygon.getNumberOfVertices();
      
      for (int i=0; i<numberOfVertices-1; i++)
      {
         Point2d vertex = convexPolygon.getVertex(i);
         Point2d vertex2 = convexPolygon.getVertex(i+1);

         Point2d projectedPoint = projectPointOntoEdge(vertex, vertex2, pointInsideConvexPolygon);
         
         distanceToEdge = pointInsideConvexPolygon.distance(projectedPoint);
         
         if (distanceToEdge < minDistanceToEdge)
         {
            minDistanceToEdge = distanceToEdge;
//            System.out.println("TurnValveBehavior: minDistanceToEdge = " + minDistanceToEdge);
         }
      }
      return minDistanceToEdge;
   }
   
   private Vector2d edgeVector = new Vector2d();
   private Vector2d constuctEdgeFromTwoVertices(Point2d firstVertex, Point2d secondVertex)
   {
      edgeVector.set(secondVertex);
      edgeVector.sub(firstVertex);
      
      return edgeVector;
   }
   
   private Vector2d firstVertexToPoint = new Vector2d();
   private Point2d projectedPoint = new Point2d();

   private Point2d projectPointOntoEdge(Point2d firstVertex, Point2d secondVertex, Point2d point)
   {
      Vector2d edgeVector = constuctEdgeFromTwoVertices(firstVertex, secondVertex);

      firstVertexToPoint.set(point);
      firstVertexToPoint.sub(firstVertex);

      projectedPoint.set(firstVertex);

      if (edgeVector.lengthSquared() > 1e-10)
      {
         double dotProduct = edgeVector.dot(firstVertexToPoint);
         double lengthSquared = edgeVector.lengthSquared();
         double alpha = dotProduct / lengthSquared;

         // Need to keep alpha between 0.0 and 1.0 since if only one edge is seen, the projection can be outside the edge.
         if (alpha < 0.0)
            alpha = 0.0;
         if (alpha > 1.0)
            alpha = 1.0;

         edgeVector.scale(alpha);

         projectedPoint.add(edgeVector);
      }
      
      return projectedPoint;
   }
   
   private HandPosePacket createHandPosePacketToBumpTheValveToVerifyItsPosition(RigidBodyTransform worldToValveTransform)
   {
      RigidBodyTransform worldToValveTransformWithYawCorrection = new RigidBodyTransform();
      worldToValveTransformWithYawCorrection.set(worldToValveTransform);

      Vector3d eulerAngles = new Vector3d();
      worldToValveTransformWithYawCorrection.getEulerXYZ(eulerAngles);
      eulerAngles.setZ(eulerAngles.z + 0.5 * Math.PI);

      Vector3d handBumpPosition = new Vector3d();
      Point3d handBumpPositionPt = new Point3d();
      worldToValveTransform.get(handBumpPosition);
      handBumpPositionPt.set(handBumpPosition);

      Quat4d orientation = new Quat4d();
      worldToValveTransform.get(orientation);
      orientation.set(1, 0, 0, 0);

      HandPosePacket ret = new HandPosePacket(RobotSide.RIGHT, Frame.WORLD, handBumpPositionPt, orientation, 1.0);

      return ret;
   }

   private void pauseBehaviorIfCapturePointIsTooCloseToSupportPolygonEdge()
   {
      boolean icpIsTooCloseToSupportPolygonEdge = minIcpDistanceToSupportPolygon.getDoubleValue() < desiredMinIcpDistanceToSupportPolygon.getDoubleValue();
      
      if (icpIsTooCloseToSupportPolygonEdge)
      {
         this.pause();
         System.out.println("TurnValveBehavior: ICP is too close to Support Polygon Edge!  Icp distance to edge : " + minIcpDistanceToSupportPolygon.getDoubleValue() + ".  Desired minimum distance to edge : " + desiredMinIcpDistanceToSupportPolygon.getDoubleValue());
      }
   }
   
   private void pauseBehaviorIfCapturePointErrorIsTooLarge()
   {
      if (capturePointErrorMag.getDoubleValue() > MAX_CAPTURE_POINT_ERROR_M)
      {
         this.pause();
         System.out.println("TurnValveBehavior: MAX CAPTURE POINT ERROR EXCEEDED!  Capture Point Error =  " + capturePointErrorMag.getDoubleValue());
      }
   }

   private void pauseBehaviorIfWristForceSensorSuddenlySpikes()
   {
      if (rightWristForceFiltered.getDoubleValue() > MAX_WRIST_FORCE_THRESHOLD_N)
      {
         this.pause();
         System.out.println("TurnValveBehavior: MAX WRIST FORCE EXCEEDED!  Force Magnitude =  " + rightWristForceFiltered.getDoubleValue());
      }
   }

   private void setValveLocationAndOrientation(RigidBodyTransform worldToValveTransform)
   {
      worldToValveTransform.get(valveLocation);
      valveOrientation.set(worldToValveTransform);
   }

   private void setTargetWalkLocationAndOrientation(Vector3d valveLocation, YoFrameOrientation valveOrientation)
   {
      targetWalkLocation.set(valveLocation);
      worldToValveTransform.transform(valveInteractionOffsetInValveFrame, valveOffsetInWorldFrame);
      targetWalkLocation.add(valveOffsetInWorldFrame);
      targetWalkOrientation.setYawPitchRoll(valveOrientation.getYaw().getDoubleValue() + 0.5 * Math.PI, valveOrientation.getPitch().getDoubleValue(),
            valveOrientation.getRoll().getDoubleValue());
      walkToLocationBehavior.setTarget(targetWalkLocation, targetWalkOrientation);

      System.out.println("TurnValveBehavior:  ValveOffset in World Frame:" + valveOffsetInWorldFrame);
      System.out.println("TurnValveBehavior: Turn Valve Location Updated:" + valveLocation);
      System.out.println("TurnValveBehavior: Target Walk to Location Updated:" + targetWalkLocation);
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      currentBehavior.passReceivedNetworkProcessorObjectToChildBehaviors(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      currentBehavior.passReceivedControllerObjectToChildBehaviors(object);
   }

   @Override
   public void stop()
   {
      currentBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      System.out.println("TurnValveBehavior: Current Child Behavior: " + currentBehavior.getName());

      System.out.println("TurnValveBehavior: Script Resource Stream" + scriptResourceStream);

      System.out.println("TurnValveBehavior: max wrist force : " + maxObservedWristForce);
      System.out.println("TurnValveBehavior: max capture point error : " + maxObservedCapturePointError);

      System.out.println("TurnValveBehavior: Valve Location: " + valveLocation);
      System.out.println("TurnValveBehavior: ValveOffset in World Frame: " + valveOffsetInWorldFrame);
      System.out.println("TurnValveBehavior: Target Walk to Location: " + targetWalkLocation);
   }

   @Override
   public void pause()
   {
      currentBehavior.pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      currentBehavior.resume();
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return isDone;
   }

   @Override
   public void finalize()
   {
      currentBehavior.finalize();
   }

   @Override
   public void initialize()
   {
      if (scriptBehavior != null)
      {
         scriptBehavior.initialize();
      }
      if (walkToLocationBehavior != null)
      {
         walkToLocationBehavior.initialize();
      }
      if (handPoseBehavior != null)
      {
         handPoseBehavior.initialize();
      }

      behaviorQueue.clear();
      behaviorQueue.add(walkToLocationBehavior);
//      behaviorQueue.add(handPoseBehavior);
      behaviorQueue.add(scriptBehavior);
      currentBehavior = behaviorQueue.remove(0);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return currentBehavior.hasInputBeenSet();
   }
}
