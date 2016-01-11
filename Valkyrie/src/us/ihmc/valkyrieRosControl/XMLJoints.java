package us.ihmc.valkyrieRosControl;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "Joints")
public class XMLJoints
{
   private String robotName;
   private List<XMLJointWithTorqueOffset> joints;

   public String getRobotName()
   {
      return robotName;
   }

   @XmlElement(name = "Robot")
   public void setRobotName(String robotName)
   {
      this.robotName = robotName;
   }

   public List<XMLJointWithTorqueOffset> getJoints()
   {
      return joints;
   }

   @XmlElement(name = "Joint")
   public void setJoints(List<XMLJointWithTorqueOffset> joints)
   {
      this.joints = joints;
   }

   public static class XMLJointWithTorqueOffset
   {
      private String name;
      private String position;
      private String torqueOffset;
      private String type;

      @XmlAttribute(name = "name")
      public String getName()
      {
         return name;
      }

      public void setName(String name)
      {
         this.name = name;
      }

      @XmlAttribute(name = "position")
      public String getPosition()
      {
         return position;
      }

      public void setPosition(String position)
      {
         this.position = position;
      }

      @XmlAttribute(name = "torqueOffset")
      public String getTorqueOffset()
      {
         return torqueOffset;
      }

      public void setTorqueOffset(String torqueOffset)
      {
         this.torqueOffset = torqueOffset;
      }

      @XmlAttribute(name = "type")
      public String getType()
      {
         return type;
      }

      public void setType(String type)
      {
         this.type = type;
      }
   }
}
