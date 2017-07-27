package us.ihmc.robotDataLogger.handshake;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakeFileType;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class YoVariableHandshakeParser
{

   @SuppressWarnings("deprecation")
   public static YoVariableHandshakeParser create(HandshakeFileType type)
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
         return new IDLYoVariableHandshakeParser(type);
      case PROTOBUFFER:
         return new ProtoBufferYoVariableHandshakeParser();
      default:
         throw new RuntimeException("Not implemented");
      }
   }
   
   public static int getNumberOfStateVariables(HandshakeFileType type, byte[] data) throws IOException
   {
      YoVariableHandshakeParser parser = create(type);
      parser.parseFrom(data);
      return parser.getNumberOfStates();
   }
   
   protected final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   protected final ArrayList<JointState> jointStates = new ArrayList<>();
   protected double dt;
   protected int stateVariables;
   protected int numberOfVariables;
   protected int numberOfJointStateVariables;
   protected List<YoVariableRegistry> registries = new ArrayList<>();
   protected List<YoVariable<?>> variables = new ArrayList<>();

   public abstract void parseFrom(Handshake handshake) throws IOException;
   public abstract void parseFrom(byte[] handShake) throws IOException;
   
   public YoVariableHandshakeParser()
   {
   }

   public YoVariableRegistry getRootRegistry()
   {
      return registries.get(0);
   }
   
   public List<YoVariableRegistry> getRegistries()
   {
      return Collections.unmodifiableList(registries);
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
      return stateVariables * 8;
   }

   public int getNumberOfStates()
   {
      return stateVariables;

   }
   
   public int getNumberOfVariables()
   {
      return numberOfVariables;
   }

   public int getNumberOfJointStateVariables()
   {
      return numberOfJointStateVariables;
   }
   
   
}