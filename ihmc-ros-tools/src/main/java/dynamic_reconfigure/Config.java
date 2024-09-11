package dynamic_reconfigure;

import java.util.List;
import org.ros.internal.message.Message;

public interface Config extends Message {
   String _TYPE = "dynamic_reconfigure/Config";
   String _DEFINITION = "BoolParameter[] bools\nIntParameter[] ints\nStrParameter[] strs\nDoubleParameter[] doubles\nGroupState[] groups\n";

   List<BoolParameter> getBools();

   void setBools(List<BoolParameter> var1);

//   List<IntParameter> getInts();
//
//   void setInts(List<IntParameter> var1);

   List<StrParameter> getStrs();

//   void setStrs(List<StrParameter> var1);

   List<DoubleParameter> getDoubles();

   void setDoubles(List<DoubleParameter> var1);

   List<GroupState> getGroups();

   void setGroups(List<GroupState> var1);
}