package us.ihmc.robotDataLogger.handshake;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphic.RemoteGraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicFactory;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.idl.serializers.extra.AbstractSerializer;
import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.robotDataLogger.EnumType;
import us.ihmc.robotDataLogger.GraphicObjectMessage;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakeFileType;
import us.ihmc.robotDataLogger.HandshakePubSubType;
import us.ihmc.robotDataLogger.JointDefinition;
import us.ihmc.robotDataLogger.YoRegistryDefinition;
import us.ihmc.robotDataLogger.YoType;
import us.ihmc.robotDataLogger.YoVariableDefinition;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

/**
 * Class to decode variable data from handshakes
 * 
 * @author jesper
 *
 */
public class IDLYoVariableHandshakeParser extends YoVariableHandshakeParser
{
   private final AbstractSerializer<Handshake> serializer;

   public IDLYoVariableHandshakeParser(HandshakeFileType type, String registryPrefix)
   {
      super(registryPrefix);
      switch (type)
      {
      case IDL_YAML:
         serializer = new YAMLSerializer<>(new HandshakePubSubType());
         break;
      default:
         serializer = null;
         break;
      }
   }

   public static int getNumberOfVariables(Handshake handShake)
   {
      int jointStateVariables = 0;
      for (int i = 0; i < handShake.getJoints().size(); i++)
      {
         jointStateVariables += JointState.getNumberOfVariables(handShake.getJoints().get(i).getType());
      }

      return 1 + handShake.getVariables().size() + jointStateVariables;
   }

   public void parseFrom(byte[] data) throws IOException
   {
      if(serializer == null)
      {
         throw new RuntimeException();
      }
      Handshake handshake = serializer.deserialize(data);
      parseFrom(handshake);
   }

   public void parseFrom(Handshake handshake)
   {
      this.dt = handshake.getDt();
      List<YoVariableRegistry> regs = parseRegistries(handshake, registryPrefix);

      // don't replace those list objects (it's a big code mess), just populate them with received data
      registries.clear();
      registries.addAll(regs);

      List<YoVariable<?>> vars = parseVariables(handshake, regs);

      // don't replace those list objects (it's a big code mess), just populate them with received data
      variables.clear();
      variables.addAll(vars);

      addJointStates(handshake);
      addGraphicObjects(handshake);

      int numberOfVariables = handshake.getVariables().size();
      int numberOfJointStateVariables = getNumberOfJointStateVariables(handshake);
      this.bufferSize = (1 + numberOfVariables + numberOfJointStateVariables) * 8;
   }

   private static List<YoVariableRegistry> parseRegistries(Handshake handshake, String registryPrefix)
   {
      YoRegistryDefinition rootDefinition = handshake.getRegistries().get(0);
      YoVariableRegistry rootRegistry = new YoVariableRegistry(registryPrefix + rootDefinition.getName());

      List<YoVariableRegistry> registryList = new ArrayList<>();
      registryList.add(rootRegistry);

      for (int i = 1; i < handshake.getRegistries().size(); i++)
      {
         YoRegistryDefinition registryDefinition = handshake.getRegistries().get(i);
         YoVariableRegistry registry = new YoVariableRegistry(registryDefinition.getNameAsString());
         registryList.add(registry);
         registryList.get(registryDefinition.getParent()).addChild(registry);
      }

      return registryList;
   }

   @SuppressWarnings("rawtypes")
   private static List<YoVariable<?>> parseVariables(Handshake handshake, List<YoVariableRegistry> registryList)
   {
      List<YoVariable<?>> variableList = new ArrayList<>();
      for (int i = 0; i < handshake.getVariables().size(); i++)
      {
         YoVariableDefinition yoVariableDefinition = handshake.getVariables().get(i);

         String name = yoVariableDefinition.getNameAsString();
         int registryIndex = yoVariableDefinition.getRegistry();
         YoVariableRegistry parent = registryList.get(registryIndex);

         YoType type = yoVariableDefinition.getType();
         switch (type)
         {
         case DoubleYoVariable:
            DoubleYoVariable doubleVar = new DoubleYoVariable(name, parent);
            variableList.add(doubleVar);
            break;

         case IntegerYoVariable:
            IntegerYoVariable intVar = new IntegerYoVariable(name, parent);
            variableList.add(intVar);
            break;

         case BooleanYoVariable:
            BooleanYoVariable boolVar = new BooleanYoVariable(name, parent);
            variableList.add(boolVar);
            break;

         case LongYoVariable:
            LongYoVariable longVar = new LongYoVariable(name, parent);
            variableList.add(longVar);
            break;

         case EnumYoVariable:
            EnumType enumType = handshake.getEnumTypes().get(yoVariableDefinition.getEnumType());
            String[] names = enumType.getEnumValues().toStringArray();
            boolean allowNullValues = yoVariableDefinition.getAllowNullValues();
            EnumYoVariable enumVar = new EnumYoVariable(name, "", parent, allowNullValues, names);
            variableList.add(enumVar);
            break;

         default:
            throw new RuntimeException("Unknown YoVariable type: " + type.name());
         }
      }

      return variableList;
   }

   private int getNumberOfJointStateVariables(Handshake handshake)
   {
      int numberOfJointStates = 0;
      for (int i = 0; i < handshake.getJoints().size(); i++)
      {
         JointDefinition joint = handshake.getJoints().get(i);
         numberOfJointStates += JointState.getNumberOfVariables(joint.getType());
      }
      return numberOfJointStates;
   }

   private void addJointStates(Handshake handshake)
   {
      for (int i = 0; i < handshake.getJoints().size(); i++)
      {
         JointDefinition joint = handshake.getJoints().get(i);
         jointStates.add(JointState.createJointState(joint.getNameAsString(), joint.getType()));
      }
   }

   private void addGraphicObjects(Handshake yoProtoHandshake)
   {
      HashMap<String, YoGraphicsList> dgoListMap = new HashMap<String, YoGraphicsList>();
      String listName;
      YoGraphicsList dgoList;
      for (int i = 0; i < yoProtoHandshake.getGraphicObjects().size(); i++)
      {
         listName = "default";
         if (!yoProtoHandshake.getGraphicObjects().get(i).getListNameAsString().isEmpty())
         {
            listName = yoProtoHandshake.getGraphicObjects().get(i).getListNameAsString();
         }

         if (dgoListMap.containsKey(listName))
         {
            dgoList = dgoListMap.get(listName);
         }
         else
         {
            dgoList = new YoGraphicsList(listName);
            dgoListMap.put(listName, dgoList);
         }

         try
         {
            dgoList.add((YoGraphic) getRemoteGraphic(yoProtoHandshake.getGraphicObjects().get(i)));
         }
         catch (Exception e)
         {
            PrintTools.error(this, "Got exception: " + e.getClass().getSimpleName() + " when loading a YoGraphic.");
         }
      }

      for (String list : dgoListMap.keySet())
      {
         yoGraphicsListRegistry.registerYoGraphicsList(dgoListMap.get(list));
      }

      ArtifactList artifactList = new ArtifactList("remote");
      for (int i = 0; i < yoProtoHandshake.getArtifacts().size(); i++)
      {
         try
         {
            artifactList.add((Artifact) getRemoteGraphic(yoProtoHandshake.getArtifacts().get(i)));
         }
         catch (Exception e)
         {
            PrintTools.error(this, "Got exception: " + e.getClass().getSimpleName() + " when loading a Artifact.");
         }
      }
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   private RemoteYoGraphic getRemoteGraphic(GraphicObjectMessage graphicObjectMessage)
   {
      RemoteGraphicType type = RemoteGraphicType.values()[graphicObjectMessage.getType()];

      String name = graphicObjectMessage.getNameAsString();
      YoVariable<?>[] vars = new YoVariable[graphicObjectMessage.getYoVariableIndex().size()];
      for (int v = 0; v < vars.length; v++)
         vars[v] = variables.get(graphicObjectMessage.getYoVariableIndex().get(v));

      double[] consts = graphicObjectMessage.getConstants().toArray();

      AppearanceDefinition appearance = new YoAppearanceRGBColor(new MutableColor((float) graphicObjectMessage.getAppearance().getR(), (float) graphicObjectMessage.getAppearance().getG(), (float) graphicObjectMessage.getAppearance().getB()),
            graphicObjectMessage.getAppearance().getTransparency());

      return YoGraphicFactory.yoGraphicFromMessage(type, name, vars, consts, appearance);
   }
}
