package us.ihmc.tools.saveableModule;

import us.ihmc.yoVariables.parameters.xml.Parameter;

import javax.xml.bind.annotation.*;
import java.util.ArrayList;
import java.util.List;

@XmlRootElement
@XmlAccessorType(XmlAccessType.NONE)
public class SaveableRegistry
{
   @XmlAttribute
   private String name;

   @XmlElement(name = "parameter")
   private List<Parameter> parameters;

   public SaveableRegistry()
   {
   }

   public SaveableRegistry(String name)
   {
      setName(name);
      setParameters(new ArrayList<>());
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public List<Parameter> getParameters()
   {
      return parameters;
   }

   public void setParameters(List<Parameter> parameters)
   {
      this.parameters = parameters;
   }
}
