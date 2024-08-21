package us.ihmc.robotDataVisualizer.logger;

import java.util.List;
import java.util.stream.Collectors;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

import us.ihmc.simulationconstructionset.gui.config.VarGroup;

/**
 * XML class for loading/saving {@link VarGroup}. Example of a valid file format:
 * 
 * @formatter:off
 * <?xml version="1.0" encoding="UTF-8" standalone="yes"?>
 * <SCSVarGroupCollection>
 *    <varGroups name="leftHipRoll">
 *       <varNames> q_leftHipRoll  </varNames>
 *       <varNames> qd_leftHipRoll </varNames>
 *       <varNames> qdd_leftHipRoll</varNames>
 *       <varNames> tau_leftHipRoll</varNames>
 *    </varGroups>
 *    <varGroups name="rightHipRoll">
 *       <varNames> q_rightHipRoll  </varNames>
 *       <varNames> qd_rightHipRoll </varNames>
 *       <varNames> qdd_rightHipRoll</varNames>
 *       <varNames> tau_rightHipRoll</varNames>
 *    </varGroups>
 * </SCSVarGroupCollection>
 * @formatter:on
 */
@XmlRootElement(name = "SCSVarGroupCollection")
public class XMLVarGroupCollection
{
   private List<XMLVarGroup> varGroups;

   public XMLVarGroupCollection()
   {
   }

   @XmlElement
   public void setVarGroups(List<XMLVarGroup> varGroups)
   {
      this.varGroups = varGroups;
   }

   public List<XMLVarGroup> getVarGroups()
   {
      return varGroups;
   }

   public List<VarGroup> toVarGroup()
   {
      return varGroups.stream().map(XMLVarGroup::toVarGroup).collect(Collectors.toList());
   }

   public static class XMLVarGroup
   {
      private String name;
      private List<String> varNames;

      @XmlAttribute
      public void setName(String name)
      {
         this.name = name;
      }

      @XmlElement
      public void setVarNames(List<String> varNames)
      {
         this.varNames = varNames;
      }

      public String getName()
      {
         return name;
      }

      public List<String> getVarNames()
      {
         return varNames;
      }

      public VarGroup toVarGroup()
      {
         VarGroup varGroup = new VarGroup(name.trim());
         varNames.forEach(varName -> varGroup.addVar(varName.trim()));
         return varGroup;
      }
   }
}
