package us.ihmc.quadrupedRobotics.providers;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedTimedStepPacket;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStepInputProvider
{
   private final AtomicReference<QuadrupedTimedStepPacket> timedStepPacket;
   
   private final EnumYoVariable<RobotQuadrant> yoTimedStepQuadrant;
   private final DoubleYoVariable yoTimedStepDuration;
   private final DoubleYoVariable yoTimedStepGroundClearance;
   private final YoFramePoint yoTimedStepGoalPosition;

   public QuadrupedTimedStepInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry registry)
   {
      timedStepPacket = new AtomicReference<>(new QuadrupedTimedStepPacket());

      yoTimedStepQuadrant = new EnumYoVariable<>("timedStepQuadrant", registry, RobotQuadrant.class);
      yoTimedStepDuration = new DoubleYoVariable("timedStepDuration", registry);
      yoTimedStepGroundClearance = new DoubleYoVariable("timedStepGroundClearance", registry);
      yoTimedStepGoalPosition = new YoFramePoint("timedStepGoalPosition", ReferenceFrame.getWorldFrame(), registry);
      
      setupYoVariableChangedListener(yoTimedStepQuadrant);
      setupYoVariableChangedListener(yoTimedStepDuration);
      setupYoVariableChangedListener(yoTimedStepGroundClearance);
      setupYoVariableChangedListener(yoTimedStepGoalPosition.getYoX());
      setupYoVariableChangedListener(yoTimedStepGoalPosition.getYoY());
      setupYoVariableChangedListener(yoTimedStepGoalPosition.getYoZ());
      
      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedTimedStepPacket.class, new PacketConsumer<QuadrupedTimedStepPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedTimedStepPacket packet)
            {
               timedStepPacket.set(packet);
            }
         });
      }
   }
   
   private void setupYoVariableChangedListener(YoVariable<?> yoVariable)
   {
      yoVariable.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            QuadrupedTimedStep quadrupedTimedStep = new QuadrupedTimedStep(yoTimedStepQuadrant.getEnumValue(), yoTimedStepGoalPosition.getPoint3dCopy(),
                                                                           yoTimedStepGroundClearance.getDoubleValue(),
                                                                           new TimeInterval(0.5, 0.5 + yoTimedStepDuration.getDoubleValue()), false);
            timedStepPacket.set(new QuadrupedTimedStepPacket(Collections.singletonList(quadrupedTimedStep)));
         }
      });
   }

   public ArrayList<QuadrupedTimedStep> getAndClearSteps()
   {
      ArrayList<QuadrupedTimedStep> steps = timedStepPacket.get().get();
      timedStepPacket.set(new QuadrupedTimedStepPacket());
      return steps;
   }
}
