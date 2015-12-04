package us.ihmc.quadrupedRobotics.stateEstimator.kinematicsBased;

import java.util.ArrayList;

import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class CenterOfMassLinearStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final FootSwitchOutputReadOnly footSwitchOutputReadOnly;
   private final FullInverseDynamicsStructure inverseDynamicsStructure;
   
   private final FramePoint comPosition = new FramePoint(worldFrame);
   
   private final QuadrantDependentList<YoFramePoint> footPositions = new QuadrantDependentList<>();
   private final ArrayList<RobotQuadrant> feetInContact = new ArrayList<>();
   private final ArrayList<RobotQuadrant> feetNotInContact = new ArrayList<>();
   
   private final CenterOfMassKinematicBasedCalculator comAndFeetCalculator;

   public CenterOfMassLinearStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, FootSwitchOutputReadOnly footSwitchOutputReadOnly,
         QuadrantDependentList<RigidBody> shinRigidBodies, QuadrantDependentList<ReferenceFrame> footFrames, YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         YoFramePoint footPosition = new YoFramePoint(quadrant.getCamelCaseNameForStartOfExpression() + "EstimatedFootPositionInWorld", worldFrame, registry);
         footPositions.set(quadrant, footPosition);
      }
      
      this.footSwitchOutputReadOnly = footSwitchOutputReadOnly;
      this.inverseDynamicsStructure = inverseDynamicsStructure;
      
      comAndFeetCalculator = new CenterOfMassKinematicBasedCalculator(inverseDynamicsStructure, shinRigidBodies, footFrames, registry);
      
      parentRegistry.addChild(registry);
      
   }
   
   public void initialize()
   {
      updateFeetContactStatus();
      
      //initialize the CoM to be at 0.0 0.0 0.0 in worldFrame
      //XXX: will need to do initialize that more smartly
      comAndFeetCalculator.initialize(new FramePoint(worldFrame), footPositions);
   }
   
   public void updateCenterOfMassLinearState()
   {
      updateFeetContactStatus();
      comAndFeetCalculator.estimateFeetAndComPosition(feetInContact, feetNotInContact, footPositions, comPosition);
      
   }

   private void updateFeetContactStatus()
   {
      feetInContact.clear();
      feetNotInContact.clear();
      for(RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if(footSwitchOutputReadOnly.isFootInContact(quadrant))
            feetInContact.add(quadrant);
         else
            feetNotInContact.add(quadrant);
      }
   }
   
}
