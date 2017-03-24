package us.ihmc.robotDataLogger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public abstract class YoVariableHandshakeParser
{

   @SuppressWarnings("deprecation")
   public static YoVariableHandshakeParser create(HandshakeFileType type, String registryPrefix)
   {
      if(type == null)
      {
         System.err.println("Handshake file type is null. Defaulting to PROTOBUFFER");
         type = HandshakeFileType.PROTOBUFFER;
      }
      
      switch(type)
      {
      case IDL_CDR:
      case IDL_YAML:
         return new IDLYoVariableHandshakeParser(type, registryPrefix);
      case PROTOBUFFER:
         return new ProtoBufferYoVariableHandshakeParser(registryPrefix);
      default:
         throw new RuntimeException("Not implemented");
      }
   }
   
   protected final String registryPrefix;
   protected final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   protected final ArrayList<JointState> jointStates = new ArrayList<>();
   protected double dt;
   protected int bufferSize;
   protected List<YoVariableRegistry> registries = new ArrayList<>();
   protected List<YoVariable<?>> variables = new ArrayList<>();

   public abstract void parseFrom(Handshake handshake) throws IOException;
   public abstract void parseFrom(byte[] handShake) throws IOException;
   
   public YoVariableHandshakeParser(String registryPrefix)
   {
      this.registryPrefix = registryPrefix;
   }

   public YoVariableRegistry getRootRegistry()
   {
      return registries.get(0);
   }

   public List<JointState> getJointStates()
   {
      return Collections.unmodifiableList(jointStates);
   }

   public List<YoVariable<?>> getYoVariablesList()
   {
      return Collections.unmodifiableList(variables);
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public double getDt()
   {
      return dt;
   }

   public int getBufferSize()
   {
      return bufferSize;
   }

}