package us.ihmc.darpaRoboticsChallenge.scriptEngine;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptFileLoader;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class VariousWalkingProviderFromScriptFactory implements VariousWalkingProviderFactory
{
   private final ScriptFileLoader scriptFileLoader;

   public VariousWalkingProviderFromScriptFactory(String filename)
   {
      try
      {
         this.scriptFileLoader = new ScriptFileLoader(filename);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         throw new RuntimeException("Could not load script file " + filename);
      }
   }

   public VariousWalkingProviderFromScriptFactory(InputStream scriptInputStream)
   {
      try
      {
         this.scriptFileLoader = new ScriptFileLoader(scriptInputStream);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         throw new RuntimeException("Could not load script input stream");
      }
   }

   public VariousWalkingProviders createVariousWalkingProviders(final DoubleYoVariable time, FullHumanoidRobotModel fullRobotModel,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames,
         SideDependentList<? extends ContactablePlaneBody> feet, ConstantTransferTimeCalculator transferTimeCalculator,
         ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposeableRegistry)
   {
      ScriptBasedFootstepProvider footstepProvider = new ScriptBasedFootstepProvider(referenceFrames, scriptFileLoader, time, feet, fullRobotModel,
            walkingControllerParameters, registry);

      updatables.add(footstepProvider);

      CapturabilityBasedStatusProducer capturabilityBasedStatusProducer = null;

      VariousWalkingProviders variousProviders = new VariousWalkingProviders(footstepProvider,
            capturabilityBasedStatusProducer);

      return variousProviders;
   }
}
