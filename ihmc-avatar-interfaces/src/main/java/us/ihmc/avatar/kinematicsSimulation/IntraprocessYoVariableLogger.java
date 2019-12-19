package us.ihmc.avatar.kinematicsSimulation;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandShakeBuilder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.File;

public class IntraprocessYoVariableLogger
{
   public IntraprocessYoVariableLogger(LogModelProvider logModelProvider,
                                       YoVariableRegistry registry,
                                       RigidBodyBasics rootBody,
                                       YoGraphicsListRegistry yoGraphicsListRegistry,
                                       double dt)
   {

      RegistrySendBufferBuilder registrySendBufferBuilder = new RegistrySendBufferBuilder(registry, rootBody, yoGraphicsListRegistry);

      YoVariableHandShakeBuilder handshakeBuilder = new YoVariableHandShakeBuilder("main", dt);  // might not want this
      handshakeBuilder.setFrames(ReferenceFrame.getWorldFrame());
      handshakeBuilder.addRegistryBuffer(registrySendBufferBuilder);
      Handshake handshake = handshakeBuilder.getHandShake();

      LogHandshake logHandshake = new LogHandshake();

      ObjectMapper objectMapper = new ObjectMapper(new YAMLFactory());

      try
      {
         YAMLSerializer<Handshake> serializer = new YAMLSerializer<>(new HandshakePubSubType());
         serializer.serialize(new File("handshake.yml"), handshake);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

}
