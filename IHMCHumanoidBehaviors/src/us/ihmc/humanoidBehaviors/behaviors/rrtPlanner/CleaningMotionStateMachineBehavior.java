package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.CleaningMotionStateMachineBehavior.CleaningMotionState;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.SolarPanelCleaningInfo.CleaningPathType;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.SolarPanelCleaningInfo.DegreesOfRedundancy;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelWholeBodyTrajectoryMessageFactory;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.manipulation.planning.solarpanelmotion.SquareFittingFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class CleaningMotionStateMachineBehavior extends StateMachineBehavior<CleaningMotionState>
{   
   private int numberOfPlanar = 0;
   private PlanarRegion planarRegion;
   private GetSolarPanelBehavior getSolarPanelBehavior;
   //private ManuallyPutSolarPanelBehavior getSolarPanelBehavior;
   
   private ControlPointOptimizationStateMachineBehavior controlPointOptimizationBehavior;
   
   private WholeBodyTrajectoryBehavior wholebodyTrajectoryBehavior;
   
   private TestDoneBehavior doneBehavior;
   
   private SolarPanelWholeBodyTrajectoryMessageFactory motionFactory;
   
   private DoubleYoVariable yoTime;
   private FullHumanoidRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;
   
   private boolean manuallyPutControlPoint = true;
      
   
   
   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(10);
   
   public enum CleaningMotionState
   {
      GET_SOLARPANEL, CONTROLPOINT_OPTIMIZATION, GOTO_READYPOSE, CLEANING_MOTION, DONE
   }
   
   public CleaningMotionStateMachineBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime,  
                                         WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super("CleaningMotionStateMachineBehavior", CleaningMotionState.class, yoTime, communicationBridge);
      
      PrintTools.info("CleaningMotionStateMachineBehavior ");

      getSolarPanelBehavior = new GetSolarPanelBehavior(communicationBridge);
      //getSolarPanelBehavior = new ManuallyPutSolarPanelBehavior(communicationBridge);
      
      wholebodyTrajectoryBehavior = new WholeBodyTrajectoryBehavior(communicationBridge, yoTime);
      doneBehavior = new TestDoneBehavior(communicationBridge);      
      
      motionFactory = new SolarPanelWholeBodyTrajectoryMessageFactory(fullRobotModel);
      
      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;
      this.wholeBodyControllerParameters = wholeBodyControllerParameters;
      
      controlPointOptimizationBehavior
      = new ControlPointOptimizationStateMachineBehavior(communicationBridge, yoTime, wholeBodyControllerParameters, fullRobotModel, referenceFrames);
      
      
      attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);
      
      setUpStateMachine();
   }
   
   public void setUpStateMachine()
   {    
      BehaviorAction<CleaningMotionState> getSolarPanelAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.GET_SOLARPANEL, getSolarPanelBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("getSolarPanelAction");
            sendPacket(p1);
         }
      };
      
      BehaviorAction<CleaningMotionState> controlPointOptimizationAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.CONTROLPOINT_OPTIMIZATION, controlPointOptimizationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("controlPointOptimizationAction");
            sendPacket(p1);
         }
      };
      
      StateTransitionCondition yesSolutionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b;
            if(manuallyPutControlPoint)
               b = controlPointOptimizationAction.isDone();               
            else
               b = controlPointOptimizationAction.isDone() && controlPointOptimizationBehavior.isSolved() == true;
            return b;
         }
      };
      
      StateTransitionCondition noSolutionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b;
            if(manuallyPutControlPoint)
               b = controlPointOptimizationAction.isDone();               
            else
               b = controlPointOptimizationAction.isDone() && controlPointOptimizationBehavior.isSolved() != true;            
            return b;
         }
      };
      
      BehaviorAction<CleaningMotionState> gotoReadyPoseAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.GOTO_READYPOSE, wholebodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("gotoReadyPoseAction");
            sendPacket(p1);
            
            PrintTools.info("gotoReadyPoseAction");
            WholeBodyTrajectoryMessage wholebodyMessage = new WholeBodyTrajectoryMessage();
            
            double motionTime = 5.0;
            
            SolarPanelCleaningPose pose = SolarPanelCleaningInfo.getReadyPose();
            motionFactory.setMessage(pose, Math.PI*0.0, 0.0, motionTime);
            wholebodyMessage = motionFactory.getWholeBodyTrajectoryMessage();
            PelvisTrajectoryMessage pelvisReadyMessage = new PelvisTrajectoryMessage(motionTime, new Point3D(0.0, 0.0, 0.91), new Quaternion());
            SelectionMatrix6D selectionMatrixPelvis =  new SelectionMatrix6D();
            selectionMatrixPelvis.clearSelection();
            selectionMatrixPelvis.selectLinearZ(true);
            pelvisReadyMessage.setSelectionMatrix(selectionMatrixPelvis);
            wholebodyMessage.setPelvisTrajectoryMessage(pelvisReadyMessage);
            wholebodyTrajectoryBehavior.setInput(wholebodyMessage);
         }
      };
      
      BehaviorAction<CleaningMotionState> cleaningAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.CLEANING_MOTION, wholebodyTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("cleaningAction");
            sendPacket(p1);
            
            PrintTools.info("cleaningAction");
            WholeBodyTrajectoryMessage wholebodyMessage = new WholeBodyTrajectoryMessage();
            SolarPanelPath cleaningPath = SolarPanelCleaningInfo.getCleaningPath();
            motionFactory.setCleaningPath(cleaningPath);         
            
            // ************************* Manually put 
            ArrayList<RRTNode> manuallyPutPath = new ArrayList<RRTNode>();
            
            // HORIZONAL_FIXED
            if(SolarPanelCleaningInfo.getCleaningType() == CleaningPathType.HORIZONAL_FIXED)
            {
               PrintTools.info("manually put HORIZONAL_FIXED");
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(0), 0.9302, 0.3501, 0.302));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(1), 0.9214, 0.5121, 0.253));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(2), 0.8835, 0.5121, 0.135));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(3), 0.8742, 0.3512, 0.102));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(4), 0.8687, 0.3501, 0.079));
               
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(5), 0.8721, 0.5103, 0.050));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(6), 0.8501, 0.5070, 0.031));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(7), 0.8420, 0.3054, 0.003));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(8), 0.8630, 0.3032, 0.002));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(9), 0.9101, 0.3811, 0.013));
            }
            
            else if(SolarPanelCleaningInfo.getCleaningType() == CleaningPathType.DIAGONAL)
            {
               PrintTools.info("manually put DIAGONAL");
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(0), 0.93, 0.35, 0.30));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(1), 0.92, 0.36, 0.25));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(2), 0.92, 0.36, 0.13));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(3), 0.91, 0.38, 0.10));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(4), 0.91, 0.35, 0.08));
               
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(5), 0.91, 0.40, 0.05));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(6), 0.91, 0.35, 0.03));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(7), 0.86, 0.51, 0.00));
               
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(8), 0.86, 0.3, 0.00));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(9), 0.88, 0.51, 0.01));
               
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(10), 0.86, 0.2, 0.01));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(11), 0.87, 0.51, 0.01));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(12), 0.87, 0.38, 0.01));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(13), 0.86, 0.5, 0.01));
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(14), 0.88, 0.38, 0.01));
               
               manuallyPutPath.add(new TimeDomain3DNode(cleaningPath.getArrivalTime().get(15), 0.90, 0.38, 0.01));
            }
            // DIAGONAL

            
            // ************************* Manually put
            if(manuallyPutControlPoint)
               motionFactory.setMessage(manuallyPutPath);
            else
               motionFactory.setMessage(controlPointOptimizationBehavior.getOptimalControlPointNodePath());            
            wholebodyMessage = motionFactory.getWholeBodyTrajectoryMessage();
            wholebodyTrajectoryBehavior.setInput(wholebodyMessage);
            
            
         }
      };
            
      BehaviorAction<CleaningMotionState> doneAction = new BehaviorAction<CleaningMotionState>(CleaningMotionState.DONE, doneBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("doneAction");
            sendPacket(p1);
         }
      };
      
      
      StateTransitionCondition yesPlanarRegion = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b = getSolarPanelAction.isDone() && numberOfPlanar == 1;
            return b;
         }
      };
      
      StateTransitionCondition noPlanarRegion = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {            
            boolean b = getSolarPanelAction.isDone() && numberOfPlanar != 1;
            //boolean b = getSolarPanelAction.isDone();
            return b;
         }
      };
      
      
      statemachine.addState(getSolarPanelAction);
      getSolarPanelAction.addStateTransition(CleaningMotionState.CONTROLPOINT_OPTIMIZATION, yesPlanarRegion);
      getSolarPanelAction.addStateTransition(CleaningMotionState.DONE, noPlanarRegion);
                  
      statemachine.addState(controlPointOptimizationAction);            
      controlPointOptimizationAction.addStateTransition(CleaningMotionState.GOTO_READYPOSE, yesSolutionCondition);
      controlPointOptimizationAction.addStateTransition(CleaningMotionState.DONE, noSolutionCondition);
      
      statemachine.addStateWithDoneTransition(gotoReadyPoseAction, CleaningMotionState.CLEANING_MOTION);
      statemachine.addStateWithDoneTransition(cleaningAction, CleaningMotionState.DONE);
      
      statemachine.addState(doneAction);
      
      statemachine.setStartState(CleaningMotionState.GET_SOLARPANEL);
            
      PrintTools.info("setUpStateMachine done ");
   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub
   }
     
   private boolean isPlanarRegionWithinVolume(PlanarRegion planarRegion)
   {
      boolean isAllNullPolygon = true;
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);

      PrintTools.info("getNumberOfConvexPolygons "+planarRegion.getNumberOfConvexPolygons());
      
      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         MeshDataHolder polygon = MeshDataGenerator.Polygon(transformToWorld, convexPolygon);
         
         PrintTools.info("polygonIndex "+polygonIndex);
         if(polygon != null)
         {
            PrintTools.info("Vectices "+polygon.getVertices().length);
            for(int i=0;i<polygon.getVertices().length;i++)
            {
               PrintTools.info("raw data "+i+" "+polygon.getVertices()[i].getX()+" "+polygon.getVertices()[i].getY()+" "+polygon.getVertices()[i].getZ());
               
               Point3D pointOfVertex = new Point3D(polygon.getVertices()[i].getX(), polygon.getVertices()[i].getY(), polygon.getVertices()[i].getZ());
               
               HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
               referenceFrames.updateFrames();
               ReferenceFrame midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
               
               FramePoint framePointOfVertex = new FramePoint(ReferenceFrame.getWorldFrame(), pointOfVertex);
               
               framePointOfVertex.changeFrame(midFeetFrame);
               
               if(!isOutsideOftheVolume(framePointOfVertex.getPoint()))
               {
                  return false;
               }                  
            }
            isAllNullPolygon = false;
         }         
      }
      if(isAllNullPolygon == true)
      {
         PrintTools.info("All polygons are null ");
         return false;
      }
         
      
      return true;
   }
   
   private boolean isOutsideOftheVolume(Point3D pointOfVertex)
   {
      if(pointOfVertex.getX() > 2.0 || pointOfVertex.getX() < 0.3 || pointOfVertex.getY() > 1.0 || pointOfVertex.getY() < -1.0 || pointOfVertex.getZ() > 2.0 || pointOfVertex.getZ() < 0.4)
      {
         PrintTools.info("@@ This polygon is on outside of the volume ");
         return false;
      } 
      
      return true;
   }
   
   private void sortOutSolarPanel(PlanarRegionsList planarRegionsList)
   {
      PrintTools.info("getNumberOfPlanarRegions");
      PrintTools.info(""+planarRegionsList.getNumberOfPlanarRegions());
      
      ArrayList<PlanarRegion> planarRegionsWithinVolume = new ArrayList<PlanarRegion>();
      
      for(int i=0;i<planarRegionsList.getNumberOfPlanarRegions();i++)
      {
         PrintTools.info("");
         PrintTools.info("Planar Region "+i);         
         
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         if(isPlanarRegionWithinVolume(planarRegion))
         {
            planarRegionsWithinVolume.add(planarRegion);            
         }  
      }
      
      numberOfPlanar = planarRegionsWithinVolume.size();
      
      for(int i=0;i<numberOfPlanar;i++)
      {
         PrintTools.info("put factory "+i);
         SquareFittingFactory squareFittingFactory = putPlanarRegionOnFactory(planarRegionsWithinVolume.get(i));
      }
      
      PrintTools.info("");
      PrintTools.info("The number Of planar regions with in volume is " + numberOfPlanar);
      
      if(numberOfPlanar == 1)
      {
         planarRegion = planarRegionsWithinVolume.get(0);                  
      }
   }
   
   private SquareFittingFactory putPlanarRegionOnFactory(PlanarRegion planarRegion)
   {
      Vector3D normalToPack = new Vector3D();
      ArrayList<Point3D32> vertices = new ArrayList<Point3D32>();
      
      planarRegion.getNormal(normalToPack);
      
      vertices.clear();
      
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);
      
      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         MeshDataHolder polygon = MeshDataGenerator.Polygon(transformToWorld, convexPolygon);
         
         if(polygon != null)
         {
            for(int i=0;i<polygon.getVertices().length;i++)
            {
               Point3D pointOfVertex = new Point3D(polygon.getVertices()[i].getX(), polygon.getVertices()[i].getY(), polygon.getVertices()[i].getZ());
               
               HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
               referenceFrames.updateFrames();
               ReferenceFrame midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();
               
               FramePoint framePointOfVertex = new FramePoint(ReferenceFrame.getWorldFrame(), pointOfVertex);
               
               framePointOfVertex.changeFrame(midFeetFrame);
               
               Point3D32 convertedVertex = new Point3D32(framePointOfVertex.getPoint());
               vertices.add(convertedVertex);
            }
         }         
      }
      
      return new SquareFittingFactory(normalToPack, vertices);
   }
   
   private void requestPlanarRegions()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(
            RequestPlanarRegionsListMessage.RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      sendPacket(requestPlanarRegionsListMessage);
      sendPacket(requestPlanarRegionsListMessage);
   }
   
   private class GetSolarPanelBehavior extends AbstractBehavior
   {
      private final BooleanYoVariable receivedPlanarRegionsList = new BooleanYoVariable("ReceivedPlanarRegionsList", registry);
      private PlanarRegionsList planarRegionsList;

      public GetSolarPanelBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         if (planarRegionsListQueue.isNewPacketAvailable())
         {
            PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
            planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
            receivedPlanarRegionsList.set(true);
            
            sortOutSolarPanel(planarRegionsList);
            
            PrintTools.info("getNumberOfPlanarRegions "+ planarRegionsList.getNumberOfPlanarRegions());
         }
         else
         {
            requestPlanarRegions();
         }
      }

      @Override
      public void onBehaviorEntered()
      {   
         receivedPlanarRegionsList.set(false);
         requestPlanarRegions();

         PrintTools.info("Entered GetSolarPanelBehavior ");
      }

      @Override
      public void onBehaviorAborted()
      {
      }

      @Override
      public void onBehaviorPaused()
      {
      }

      @Override
      public void onBehaviorResumed()
      {
      }

      @Override
      public void onBehaviorExited()
      {           
         // ********************************** get SolarPanel Info ********************************** //  
         
         Pose poseSolarPanel = new Pose();
         Quaternion quaternionSolarPanel = new Quaternion();
         poseSolarPanel.setPosition(0.75, -0.1, 0.9);
         quaternionSolarPanel.appendYawRotation(Math.PI*0.05);
         quaternionSolarPanel.appendRollRotation(0.0);
         quaternionSolarPanel.appendPitchRotation(-Math.PI*0.25);
         poseSolarPanel.setOrientation(quaternionSolarPanel);
         
         SolarPanel solarPanel = new SolarPanel(poseSolarPanel, 0.6, 0.6);
         
         System.out.println(solarPanel.getCenterPose());
         
         if(planarRegion != null)
         {
            SquareFittingFactory squareFittingFactory = putPlanarRegionOnFactory(planarRegion);
            //SquareFittingFactory squareFittingFactory = new SquareFittingFactory(planarRegion);
            solarPanel = squareFittingFactory.getSolarPanel();            
         }

         System.out.println(solarPanel.getCenterPose());
         
         // ********************************** get SolarPanel Info ********************************** //
         // *********************************** get Cleaning Path *********************************** //

         SolarPanelCleaningInfo.setSolarPanel(solarPanel);
         SolarPanelCleaningInfo.setCleaningPath(CleaningPathType.HORIZONAL_FIXED);
         SolarPanelCleaningInfo.setDegreesOfRedundancy(DegreesOfRedundancy.THREE);
         
         TimeDomain3DNode.defaultPelvisHeight = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
         
         // *********************************** get Cleaning Path *********************************** //
         
         controlPointOptimizationBehavior.setRootNode(SolarPanelCleaningInfo.getNode());

         PrintTools.info("Exit GetSolarPanelBehavior ");
      }

      @Override
      public boolean isDone()
      {
         return receivedPlanarRegionsList.getBooleanValue();
      }
   }
   
   private class ManuallyPutSolarPanelBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public ManuallyPutSolarPanelBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         numberOfPlanar = 1;
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {   
         PrintTools.info("ManuallyPutSolarPanelBehavior");
         
      }

      @Override
      public void onBehaviorAborted()
      {
      }

      @Override
      public void onBehaviorPaused()
      {
      }

      @Override
      public void onBehaviorResumed()
      {
      }

      @Override
      public void onBehaviorExited()
      {
         // ********************************** get SolarPanel Info ********************************** //  
         Pose poseSolarPanel = new Pose();
         Quaternion quaternionSolarPanel = new Quaternion();
         poseSolarPanel.setPosition(0.72, -0.15, 1.03);
         quaternionSolarPanel.appendYawRotation(Math.PI*0.00);
         quaternionSolarPanel.appendRollRotation(0.0);
         quaternionSolarPanel.appendPitchRotation(-0.380);
         poseSolarPanel.setOrientation(quaternionSolarPanel);
         
         SolarPanel solarPanel = new SolarPanel(poseSolarPanel, 0.64, 0.64);
         
         // ********************************** get SolarPanel Info ********************************** //
         // *********************************** get Cleaning Path *********************************** //

         SolarPanelCleaningInfo.setSolarPanel(solarPanel);
         SolarPanelCleaningInfo.setCleaningPath(CleaningPathType.HORIZONAL_FIXED);
         SolarPanelCleaningInfo.setDegreesOfRedundancy(DegreesOfRedundancy.THREE);
         
         TimeDomain3DNode.defaultPelvisHeight = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
         // *********************************** get Cleaning Path *********************************** //
         controlPointOptimizationBehavior.setRootNode(SolarPanelCleaningInfo.getNode());
         
         
         PrintTools.info("ManuallyPutSolarPanelBehavior Exited");
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
   
   
   private class TestDoneBehavior extends AbstractBehavior
   {
      private boolean isDone = false;
      
      public TestDoneBehavior(CommunicationBridgeInterface communicationBridge)
      {
         super(communicationBridge);
      }

      @Override
      public void doControl()
      {            
         isDone = true;
      }

      @Override
      public void onBehaviorEntered()
      {   
         PrintTools.info("TestDoneBehavior");
         
      }

      @Override
      public void onBehaviorAborted()
      {
      }

      @Override
      public void onBehaviorPaused()
      {
      }

      @Override
      public void onBehaviorResumed()
      {
      }

      @Override
      public void onBehaviorExited()
      {         
      }

      @Override
      public boolean isDone()
      {
         return isDone;
      }
   }
}
