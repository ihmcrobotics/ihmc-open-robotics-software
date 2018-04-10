package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class GraphicObjectMessage extends Packet<GraphicObjectMessage> implements Settable<GraphicObjectMessage>, EpsilonComparable<GraphicObjectMessage>
{
   public int type_;
   public java.lang.StringBuilder name_;
   public us.ihmc.idl.IDLSequence.Integer  yoVariableIndex_;
   public us.ihmc.idl.IDLSequence.Double  constants_;
   public us.ihmc.robotDataLogger.AppearanceDefinitionMessage appearance_;
   public java.lang.StringBuilder listName_;

   public GraphicObjectMessage()
   {
      name_ = new java.lang.StringBuilder(255);
      yoVariableIndex_ = new us.ihmc.idl.IDLSequence.Integer (1024, "type_3");

      constants_ = new us.ihmc.idl.IDLSequence.Double (128, "type_6");

      appearance_ = new us.ihmc.robotDataLogger.AppearanceDefinitionMessage();
      listName_ = new java.lang.StringBuilder(255);
   }

   public GraphicObjectMessage(GraphicObjectMessage other)
   {
      this();
      set(other);
   }

   public void set(GraphicObjectMessage other)
   {
      type_ = other.type_;

      name_.setLength(0);
      name_.append(other.name_);

      yoVariableIndex_.set(other.yoVariableIndex_);
      constants_.set(other.constants_);
      us.ihmc.robotDataLogger.AppearanceDefinitionMessagePubSubType.staticCopy(other.appearance_, appearance_);
      listName_.setLength(0);
      listName_.append(other.listName_);

   }

   public void setType(int type)
   {
      type_ = type;
   }
   public int getType()
   {
      return type_;
   }

   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   public java.lang.StringBuilder getName()
   {
      return name_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getYoVariableIndex()
   {
      return yoVariableIndex_;
   }


   public us.ihmc.idl.IDLSequence.Double  getConstants()
   {
      return constants_;
   }


   public us.ihmc.robotDataLogger.AppearanceDefinitionMessage getAppearance()
   {
      return appearance_;
   }

   public void setListName(java.lang.String listName)
   {
      listName_.setLength(0);
      listName_.append(listName);
   }

   public java.lang.String getListNameAsString()
   {
      return getListName().toString();
   }
   public java.lang.StringBuilder getListName()
   {
      return listName_;
   }


   @Override
   public boolean epsilonEquals(GraphicObjectMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.type_, other.type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.yoVariableIndex_, other.yoVariableIndex_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.constants_, other.constants_, epsilon)) return false;

      if (!this.appearance_.epsilonEquals(other.appearance_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.listName_, other.listName_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof GraphicObjectMessage)) return false;

      GraphicObjectMessage otherMyClass = (GraphicObjectMessage) other;

      if(this.type_ != otherMyClass.type_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if (!this.yoVariableIndex_.equals(otherMyClass.yoVariableIndex_)) return false;
      if (!this.constants_.equals(otherMyClass.constants_)) return false;
      if (!this.appearance_.equals(otherMyClass.appearance_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.listName_, otherMyClass.listName_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GraphicObjectMessage {");
      builder.append("type=");
      builder.append(this.type_);      builder.append(", ");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("yoVariableIndex=");
      builder.append(this.yoVariableIndex_);      builder.append(", ");
      builder.append("constants=");
      builder.append(this.constants_);      builder.append(", ");
      builder.append("appearance=");
      builder.append(this.appearance_);      builder.append(", ");
      builder.append("listName=");
      builder.append(this.listName_);
      builder.append("}");
      return builder.toString();
   }
}
