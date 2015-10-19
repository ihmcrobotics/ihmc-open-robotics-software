package household_objects_database_msgs;

public interface TranslateRecognitionIdResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/TranslateRecognitionIdResponse";
  static final java.lang.String _DEFINITION = "int32 household_objects_id\n\nint32 SUCCESS=0\nint32 ID_NOT_FOUND\nint32 DATABASE_ERROR\nint32 OTHER_ERROR\nint32 result";
  static final int SUCCESS = 0;
  int getHouseholdObjectsId();
  void setHouseholdObjectsId(int value);
  int getIDNOTFOUND();
  void setIDNOTFOUND(int value);
  int getDATABASEERROR();
  void setDATABASEERROR(int value);
  int getOTHERERROR();
  void setOTHERERROR(int value);
  int getResult();
  void setResult(int value);
}
