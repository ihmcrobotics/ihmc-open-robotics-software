package us.ihmc.avatar.ros.visualizer;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.common.DiscoveryStatus;
import us.ihmc.pubsub.common.Time;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import javax.swing.*;

public class SCSROS2Visualizer
{
   private Participant participant;
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Robot"),
                                                                               new NullGraphics3DAdapter(),
                                                                               new SimulationConstructionSetParameters());
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final YoDouble timeElapsed = new YoDouble("timeElapsed", yoRegistry);
   private final YoLong numberOfParticipants = new YoLong("numberOfParticipants", yoRegistry);
   private final YoLong numberOfPublishers = new YoLong("numberOfPublishers", yoRegistry);
   private final YoLong numberOfSubscribers = new YoLong("numberOfSubscribers", yoRegistry);
   private final YoLong numberOfEndpoints = new YoLong("numberOfEndpoints", yoRegistry);
   private final PausablePeriodicThread updateThread;
   private boolean paused = false;

   public SCSROS2Visualizer()
   {
      ExceptionTools.handle(this::setupROS2Debugger, DefaultExceptionHandler.PRINT_STACKTRACE);

      scs.addYoRegistry(yoRegistry);
      scs.setDT(1.0, 1);
      scs.setupGraph(timeElapsed.getName());
      scs.setupGraph(numberOfParticipants.getName());
      scs.setupGraph(numberOfPublishers.getName());
      scs.setupGraph(numberOfSubscribers.getName());
      scs.setupGraph(numberOfEndpoints.getName());
      scs.skipLoadingDefaultConfiguration();
      scs.hideViewport();
      scs.changeBufferSize(200);
      scs.setScrollGraphsEnabled(false);
      scs.getGUI().getFrame().setSize(1200, 800);
      JToggleButton pauseButton = new JToggleButton("Pause");
      pauseButton.addActionListener(e -> {
         paused = pauseButton.isSelected();
         scs.setScrollGraphsEnabled(paused);
      });
      scs.addButton(pauseButton);
      scs.startOnAThread();
      while (!scs.hasSimulationThreadStarted())
         ThreadTools.sleep(200);

      updateThread = new PausablePeriodicThread(getClass().getSimpleName(), UnitConversions.hertzToSeconds(5.0), this::update);
      updateThread.start();
   }

   private void setupROS2Debugger() throws Exception
   {
      int domainID = NetworkParameters.getRTPSDomainID();
      Domain domain = DomainFactory.getDomain(DomainFactory.PubSubImplementation.FAST_RTPS);
      ParticipantAttributes attributes = domain.createParticipantAttributes();
      attributes.setDomainId(domainID);
      attributes.setLeaseDuration(Time.Infinite);
      attributes.setName(getClass().getSimpleName());

      participant = domain.createParticipant(attributes, (participant, info) ->
      {
         if (!info.getGuid().equals(this.participant.getGuid()))
         {
            if (info.getStatus() == DiscoveryStatus.DISCOVERED_RTPSPARTICIPANT)
            {
               numberOfParticipants.add(1);
               LogTools.info("Discovered participant: {}", info.getName());
            }
            else if (info.getStatus() == DiscoveryStatus.REMOVED_RTPSPARTICIPANT)
            {
               numberOfParticipants.subtract(1);
               LogTools.info("Participant removed: {}", info.getName());
            }
         }
      });
      participant.registerEndpointDiscoveryListeners(
      ((isAlive, guid, unicastLocatorList, multicastLocatorList, participantGuid, typeName,
        topicName, userDefinedId, typeMaxSerialized, topicKind, writerQosHolder) ->
      {
         numberOfPublishers.add(1);
         numberOfEndpoints.add(1);
         LogTools.info("Discovered publisher on topic: {}", topicName);
      }),
      ((isAlive, guid, expectsInlineQos, unicastLocatorList, multicastLocatorList, participantGuid, typeName,
        topicName, userDefinedId, javaTopicKind, readerQosHolder) ->
      {
         numberOfSubscribers.add(1);
         numberOfEndpoints.add(1);
         LogTools.info("Discovered subscriber on topic: {}", topicName);
      }));
   }

   private void update()
   {
      timeElapsed.set(stopwatch.totalElapsed());

      if (!paused)
      {
         int bufferSize = scs.getDataBuffer().getBufferSize();
         if (scs.getDataBuffer().getCurrentIndex() == bufferSize - 2)
         {
            scs.changeBufferSize(bufferSize + bufferSize / 2);
         }
         scs.tickAndUpdate();
      }
   }

   public static void main(String[] args)
   {
      new SCSROS2Visualizer();
   }
}
