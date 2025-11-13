package us.ihmc.rdx.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.behaviorTree.scene.RDXBehaviorTreeScene;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.HashSet;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

public class RDXAvailableBehaviorTreeFile
{
   public static final String[] FRAME_FIELD_NAMES = new String[] {"parentFrame", "objectFrame"};

   private final WorkspaceResourceFile treeFile;
   private final RDXBehaviorTreeScene scene;
   private String name;
   private String notes;
   private final SortedSet<String> referenceFrameNames = new TreeSet<>();
   private final Set<String> referenceFramesInWorld = new HashSet<>();

   public RDXAvailableBehaviorTreeFile(WorkspaceResourceFile treeFile, RDXBehaviorTreeScene scene)
   {
      this.treeFile = treeFile;
      this.scene = scene;

      JSONFileTools.load(treeFile.getFilesystemFile(), this::loadFromFile);
   }

   public void update()
   {
      referenceFramesInWorld.clear();
      for (String referenceFrameName : referenceFrameNames)
      {
         ReferenceFrame frameByName = scene.findFrameByName(referenceFrameName);
         if (frameByName != null && frameByName.getRootFrame() == ReferenceFrame.getWorldFrame())
         {
            referenceFramesInWorld.add(referenceFrameName);
         }
      }
   }

   private void loadFromFile(JsonNode jsonNode)
   {
      name = jsonNode.get("name").asText();
      notes = jsonNode.get("notes").asText();

      loadChildrenData(jsonNode);
   }

   private void loadChildrenData(JsonNode childNode)
   {
      for (String frameFieldName : FRAME_FIELD_NAMES)
      {
         JsonNode frameNameNode = childNode.get(frameFieldName);
         if (frameNameNode != null)
         {
            referenceFrameNames.add(frameNameNode.textValue());
         }
      }

      JSONTools.forEachArrayElement(childNode, "children", this::loadChildrenData);
   }

   public String getName()
   {
      return name;
   }

   public String getNotes()
   {
      return notes;
   }

   public WorkspaceResourceFile getTreeFile()
   {
      return treeFile;
   }

   public SortedSet<String> getReferenceFrameNames()
   {
      return referenceFrameNames;
   }

   public int getNumberOfFramesInWorld()
   {
      return referenceFramesInWorld.size();
   }

   public Set<String> getReferenceFramesInWorld()
   {
      return referenceFramesInWorld;
   }
}
